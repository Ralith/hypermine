mod surface;
mod surface_extraction;

#[cfg(test)]
mod tests;

use std::mem;
use std::sync::Arc;

use ash::{version::DeviceV1_0, vk, Device};

use tracing::warn;

use super::lru_table::LruTable;
use crate::{
    graphics::{Base, Loader},
    sim::VoxelData,
    Config, Sim,
};
use common::{graph::NodeId, world::SUBDIVISION_FACTOR};

use surface::Surface;
use surface_extraction::{DrawBuffer, ScratchBuffer, SurfaceExtraction};

pub struct Voxels {
    // TODO: Remove in favor of explicit destruction
    gfx: Arc<Base>,
    config: Arc<Config>,
    surface_extraction: SurfaceExtraction,
    extraction_scratch: ScratchBuffer,
    surfaces: DrawBuffer,
    states: LruTable<SurfaceState>,
    draw: Surface,
}

impl Voxels {
    pub fn new(gfx: Arc<Base>, config: Arc<Config>, loader: &mut Loader, frames: u32) -> Self {
        let max_supported_chunks = gfx.limits.max_storage_buffer_range
            / (8 * 3 * (SUBDIVISION_FACTOR.pow(3) + SUBDIVISION_FACTOR.pow(2))) as u32;
        let max_chunks = if MAX_CHUNKS > max_supported_chunks {
            warn!(
                "clamping max chunks to {} due to SSBO size limit",
                max_supported_chunks
            );
            max_supported_chunks
        } else {
            MAX_CHUNKS
        };
        let surfaces = DrawBuffer::new(gfx.clone(), max_chunks, SUBDIVISION_FACTOR as u32);
        let draw = Surface::new(&config, loader, &surfaces, frames);
        let surface_extraction = SurfaceExtraction::new(gfx.clone());
        let extraction_scratch = surface_extraction::ScratchBuffer::new(
            &surface_extraction,
            config.chunks_loaded_per_frame * frames,
            SUBDIVISION_FACTOR as u32,
        );
        Self {
            gfx,
            config,
            surface_extraction,
            extraction_scratch,
            surfaces,
            states: LruTable::with_capacity(max_chunks),
            draw,
        }
    }

    /// Determine what to render and record the appropriate transfer commands
    pub unsafe fn prepare(&mut self, frame: &mut Frame, sim: &mut Sim, cmd: vk::CommandBuffer) {
        let device = &*self.gfx.device;

        // Clean up after previous frame
        for i in frame.extracted.drain(..) {
            self.extraction_scratch.free(i);
        }
        for chunk in frame.drawn.drain(..) {
            self.states.peek_mut(chunk.slot).refcount -= 1;
        }

        // Determine what to load/render
        let chunks = sim
            .graph
            .nearby_cubes(sim.view_reference(), self.config.view_distance);
        let mut removed = Vec::new();
        let view_parity = common::math::parity(&sim.view());
        for &(node, cube, parity, ref transform) in &chunks {
            // Fetch existing chunk, or extract surface of new chunk
            let slot = match *sim.graph.get_cube_mut(node, cube) {
                None => continue,
                Some(ref mut value) => match (value.surface, &value.voxels) {
                    (Some(x), _) => {
                        self.states.get_mut(x).refcount += 1;
                        x
                    }
                    (None, &VoxelData::Dense(ref data)) => {
                        if frame.extracted.len() == self.config.chunks_loaded_per_frame as usize {
                            continue;
                        }
                        if self.states.is_full() {
                            if self.states.peek_lru().map_or(false, |x| x.refcount != 0) {
                                warn!("MAX_CHUNKS is too small");
                                break;
                            }
                            let lru = self.states.remove_lru().unwrap();
                            removed.push((lru.node, lru.cube));
                        }
                        let scratch_slot = self.extraction_scratch.alloc().unwrap();
                        frame.extracted.push(scratch_slot);
                        let slot = self
                            .states
                            .insert(SurfaceState {
                                node,
                                cube,
                                refcount: 1,
                            })
                            .unwrap();
                        value.surface = Some(slot);
                        let storage = self.extraction_scratch.storage(scratch_slot);
                        storage.copy_from_slice(&data[..]);
                        self.extraction_scratch.extract(
                            &self.surface_extraction,
                            scratch_slot,
                            cmd,
                            (
                                self.surfaces.indirect_buffer(),
                                self.surfaces.indirect_offset(slot.0),
                            ),
                            (
                                self.surfaces.face_buffer(),
                                self.surfaces.face_offset(slot.0),
                            ),
                        );
                        slot
                    }
                    (None, &VoxelData::Empty) => continue,
                },
            };
            frame.drawn.push(DrawnChunk {
                slot,
                parity: view_parity ^ parity,
            });
            // Transfer transform
            device.cmd_update_buffer(
                cmd,
                frame.surface.transforms(),
                slot.0 as vk::DeviceSize * surface::TRANSFORM_SIZE,
                &mem::transmute::<_, [u8; surface::TRANSFORM_SIZE as usize]>(*transform),
            );
        }
        for (node, cube) in removed {
            sim.graph.get_cube_mut(node, cube).as_mut().unwrap().surface = None;
        }

        device.cmd_pipeline_barrier(
            cmd,
            vk::PipelineStageFlags::TRANSFER,
            vk::PipelineStageFlags::VERTEX_SHADER | vk::PipelineStageFlags::FRAGMENT_SHADER,
            vk::DependencyFlags::default(),
            &[],
            &[vk::BufferMemoryBarrier::builder()
                .src_access_mask(vk::AccessFlags::TRANSFER_WRITE)
                .dst_access_mask(vk::AccessFlags::SHADER_READ)
                .buffer(frame.surface.transforms())
                .size(vk::WHOLE_SIZE)
                .build()],
            &[],
        );
    }

    pub unsafe fn draw(
        &mut self,
        loader: &Loader,
        common_ds: vk::DescriptorSet,
        frame: &Frame,
        cmd: vk::CommandBuffer,
    ) {
        if !self.draw.bind(loader, common_ds, &frame.surface, cmd) {
            return;
        }
        for chunk in &frame.drawn {
            self.draw
                .draw(cmd, &self.surfaces, chunk.slot.0, chunk.parity);
        }
    }
}

pub struct Frame {
    surface: surface::Frame,
    /// Scratch slots completed in this frame
    extracted: Vec<u32>,
    drawn: Vec<DrawnChunk>,
}

impl Frame {
    pub unsafe fn destroy(&mut self, device: &Device) {
        self.surface.destroy(device);
    }
}

struct DrawnChunk {
    slot: super::lru_table::SlotId,
    parity: bool,
}

impl Frame {
    pub fn new(ctx: &Voxels) -> Self {
        Self {
            surface: surface::Frame::new(&ctx.draw, ctx.states.capacity()),
            extracted: Vec::new(),
            drawn: Vec::new(),
        }
    }
}

/// Maximum number of concurrently drawn voxel chunks
const MAX_CHUNKS: u32 = 4096;

struct SurfaceState {
    node: NodeId,
    cube: common::dodeca::Vertex,
    refcount: u32,
}
