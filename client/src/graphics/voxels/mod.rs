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
use common::{dodeca::Vertex, graph::NodeId, world::SUBDIVISION_FACTOR};

use surface::Surface;
use surface_extraction::{DrawBuffer, ScratchBuffer, SurfaceExtraction};

pub struct Voxels {
    config: Arc<Config>,
    surface_extraction: SurfaceExtraction,
    extraction_scratch: ScratchBuffer,
    surfaces: DrawBuffer,
    states: LruTable<SurfaceState>,
    draw: Surface,
}

impl Voxels {
    pub fn new(gfx: &Base, config: Arc<Config>, loader: &mut Loader, frames: u32) -> Self {
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
        let surfaces = DrawBuffer::new(gfx, max_chunks, SUBDIVISION_FACTOR as u32);
        let draw = Surface::new(gfx, loader, &surfaces, frames);
        let surface_extraction = SurfaceExtraction::new(gfx);
        let extraction_scratch = surface_extraction::ScratchBuffer::new(
            gfx,
            &surface_extraction,
            config.chunks_loaded_per_frame * frames,
            SUBDIVISION_FACTOR as u32,
        );
        Self {
            config,
            surface_extraction,
            extraction_scratch,
            surfaces,
            states: LruTable::with_capacity(max_chunks),
            draw,
        }
    }

    /// Determine what to render and record the appropriate transfer commands
    pub unsafe fn prepare(
        &mut self,
        device: &Device,
        frame: &mut Frame,
        sim: &mut Sim,
        cmd: vk::CommandBuffer,
    ) {
        // Clean up after previous frame
        for i in frame.extracted.drain(..) {
            self.extraction_scratch.free(i);
        }
        for chunk in frame.drawn.drain(..) {
            self.states.peek_mut(chunk).refcount -= 1;
        }

        // Determine what to load/render
        let view = sim.view();
        if !sim.graph.contains(view.node) {
            // Graph is temporarily out of sync with the server; we don't know where we are, so
            // there's no point trying to draw.
            return;
        }
        let mut nodes = sim.graph.nearby_nodes(view, self.config.view_distance);
        nodes.sort_unstable_by_key(|&(node, _)| sim.graph.length(node));
        for &(node, ref node_transform) in &nodes {
            for chunk in Vertex::iter() {
                // Fetch existing chunk, or extract surface of new chunk
                let slot = match sim
                    .graph
                    .get_mut(node)
                    .as_mut()
                    .expect("all nodes must be populated before rendering")
                    .chunks[chunk]
                {
                    None => continue,
                    Some(ref mut value) => match (value.surface, &value.voxels) {
                        (Some(x), _) => {
                            self.states.get_mut(x).refcount += 1;
                            x
                        }
                        (None, &VoxelData::Dense(ref data)) => {
                            if frame.extracted.len() == self.config.chunks_loaded_per_frame as usize
                            {
                                continue;
                            }
                            let removed = if self.states.is_full() {
                                let slot = self.states.lru().expect("full LRU table is nonempty");
                                if self.states.peek(slot).refcount != 0 {
                                    warn!("MAX_CHUNKS is too small");
                                    break;
                                }
                                Some(self.states.remove(slot))
                            } else {
                                None
                            };
                            let scratch_slot = self.extraction_scratch.alloc().expect("there are at least chunks_loaded_per_frame scratch slots per frame");
                            frame.extracted.push(scratch_slot);
                            let slot = self
                                .states
                                .insert(SurfaceState {
                                    node,
                                    chunk,
                                    refcount: 1,
                                })
                                .expect("we ensure cache space is available above");
                            value.surface = Some(slot);
                            let storage = self.extraction_scratch.storage(scratch_slot);
                            storage.copy_from_slice(&data[..]);
                            if let Some(lru) = removed {
                                sim.graph.get_mut(lru.node).as_mut().unwrap().chunks[lru.chunk]
                                    .as_mut()
                                    .unwrap()
                                    .surface = None;
                            }
                            let node_is_odd = sim.graph.length(node) & 1 != 0;
                            self.extraction_scratch.extract(
                                device,
                                &self.surface_extraction,
                                scratch_slot,
                                chunk.parity() ^ node_is_odd,
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
                        (None, &VoxelData::Solid(_)) => continue,
                    },
                };
                frame.drawn.push(slot);
                // Transfer transform
                let chunk_transform = node_transform * chunk.chunk_to_node().map(|x| x as f32);
                device.cmd_update_buffer(
                    cmd,
                    frame.surface.transforms(),
                    slot.0 as vk::DeviceSize * surface::TRANSFORM_SIZE,
                    &mem::transmute::<_, [u8; surface::TRANSFORM_SIZE as usize]>(chunk_transform),
                );
            }
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
        device: &Device,
        loader: &Loader,
        common_ds: vk::DescriptorSet,
        frame: &Frame,
        cmd: vk::CommandBuffer,
    ) {
        if !self
            .draw
            .bind(device, loader, common_ds, &frame.surface, cmd)
        {
            return;
        }
        for chunk in &frame.drawn {
            self.draw.draw(device, cmd, &self.surfaces, chunk.0);
        }
    }

    pub unsafe fn destroy(&mut self, device: &Device) {
        self.surface_extraction.destroy(device);
        self.extraction_scratch.destroy(device);
        self.surfaces.destroy(device);
        self.draw.destroy(device);
    }
}

pub struct Frame {
    surface: surface::Frame,
    /// Scratch slots completed in this frame
    extracted: Vec<u32>,
    drawn: Vec<super::lru_table::SlotId>,
}

impl Frame {
    pub unsafe fn destroy(&mut self, device: &Device) {
        self.surface.destroy(device);
    }
}

impl Frame {
    pub fn new(gfx: &Base, ctx: &Voxels) -> Self {
        Self {
            surface: surface::Frame::new(gfx, &ctx.draw, ctx.states.capacity()),
            extracted: Vec::new(),
            drawn: Vec::new(),
        }
    }
}

/// Maximum number of concurrently drawn voxel chunks
const MAX_CHUNKS: u32 = 4096;

struct SurfaceState {
    node: NodeId,
    chunk: common::dodeca::Vertex,
    refcount: u32,
}
