mod surface;
pub mod surface_extraction;

#[cfg(test)]
mod tests;

use std::{sync::Arc, time::Instant};

use ash::{Device, vk};
use metrics::histogram;
use tracing::warn;

use crate::{
    Config, Loader, Sim,
    graphics::{Base, Frustum},
};
use common::{
    LruSlab,
    dodeca::{self, Vertex},
    graph::NodeId,
    lru_slab::SlotId,
    math::{MIsometry, MPoint},
    node::{Chunk, ChunkId, VoxelData},
};

use surface::Surface;
use surface_extraction::{DrawBuffer, ExtractTask, ScratchBuffer, SurfaceExtraction};

pub struct Voxels {
    config: Arc<Config>,
    surface_extraction: SurfaceExtraction,
    extraction_scratch: ScratchBuffer,
    surfaces: DrawBuffer,
    states: LruSlab<SurfaceState>,
    draw: Surface,
    max_chunks: u32,
}

impl Voxels {
    pub fn new(
        gfx: &Base,
        config: Arc<Config>,
        loader: &mut Loader,
        dimension: u32,
        frames: u32,
    ) -> Self {
        let max_faces = 3 * (dimension.pow(3) + dimension.pow(2));
        let max_supported_chunks = gfx.limits.max_storage_buffer_range / (8 * max_faces);
        let max_chunks = if MAX_CHUNKS > max_supported_chunks {
            warn!(
                "clamping max chunks to {} due to SSBO size limit",
                max_supported_chunks
            );
            max_supported_chunks
        } else {
            MAX_CHUNKS
        };
        let surfaces = DrawBuffer::new(gfx, max_chunks, dimension);
        let draw = Surface::new(gfx, loader, &surfaces);
        let surface_extraction = SurfaceExtraction::new(gfx);
        let extraction_scratch = surface_extraction::ScratchBuffer::new(
            gfx,
            &surface_extraction,
            config.chunk_load_parallelism * frames,
            dimension,
        );
        Self {
            config,
            surface_extraction,
            extraction_scratch,
            surfaces,
            states: LruSlab::with_capacity(max_chunks),
            draw,
            max_chunks,
        }
    }

    /// Determine what to render and stage chunk transforms
    ///
    /// Surface extraction commands are written to `cmd`, and will be presumed complete for the next
    /// (not current) frame.
    pub unsafe fn prepare(
        &mut self,
        device: &Device,
        frame: &mut Frame,
        sim: &mut Sim,
        nearby_nodes: &[(NodeId, MIsometry<f32>)],
        cmd: vk::CommandBuffer,
        frustum: &Frustum,
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
        let node_scan_started = Instant::now();
        let frustum_planes = frustum.planes();
        let local_to_view = view.local.inverse();
        let mut extractions = Vec::new();
        for &(node, ref node_transform) in nearby_nodes {
            let node_to_view = local_to_view * node_transform;
            let origin = node_to_view * MPoint::origin();
            if !frustum_planes.contain(&origin, dodeca::BOUNDING_SPHERE_RADIUS) {
                // Don't bother generating or drawing chunks from nodes that are wholly outside the
                // frustum.
                continue;
            }

            use Chunk::*;
            for vertex in Vertex::iter() {
                let chunk = ChunkId::new(node, vertex);

                // Fetch existing chunk, or extract surface of new chunk
                let &mut Populated {
                    ref mut surface,
                    ref mut old_surface,
                    ref voxels,
                } = &mut sim.graph[chunk]
                else {
                    continue;
                };

                if let Some(slot) = surface.or(*old_surface) {
                    // Render an already-extracted surface
                    self.states.get_mut(slot).refcount += 1;
                    frame.drawn.push(slot);
                    // Transfer transform
                    frame.surface.transforms_mut()[slot.0 as usize] =
                        na::Matrix4::from(*node_transform) * vertex.chunk_to_node();
                }
                if let (None, &VoxelData::Dense(ref data)) = (&surface, voxels) {
                    // Extract a surface so it can be drawn in future frames
                    if frame.extracted.len() == self.config.chunk_load_parallelism as usize {
                        continue;
                    }
                    let removed = if self.states.len() == self.max_chunks {
                        let slot = self.states.lru().expect("full LRU table is nonempty");
                        if self.states.peek(slot).refcount != 0 {
                            warn!("MAX_CHUNKS is too small");
                            break;
                        }
                        Some((slot, self.states.remove(slot)))
                    } else {
                        None
                    };
                    let scratch_slot = self.extraction_scratch.alloc().expect(
                        "there are at least chunks_loaded_per_frame scratch slots per frame",
                    );
                    frame.extracted.push(scratch_slot);
                    let slot = self.states.insert(SurfaceState {
                        node,
                        chunk: vertex,
                        refcount: 0,
                    });
                    *surface = Some(slot);
                    let storage = self.extraction_scratch.storage(scratch_slot);
                    storage.copy_from_slice(&data[..]);
                    if let Some((lru_slot, lru)) = removed
                        && let Populated {
                            ref mut surface,
                            ref mut old_surface,
                            ..
                        } = sim.graph[lru.node].chunks[lru.chunk]
                    {
                        // Remove references to released slot IDs
                        if *surface == Some(lru_slot) {
                            *surface = None;
                        }
                        if *old_surface == Some(lru_slot) {
                            *old_surface = None;
                        }
                    }
                    let node_is_odd = sim.graph.depth(node) & 1 != 0;
                    extractions.push(ExtractTask {
                        index: scratch_slot,
                        indirect_offset: self.surfaces.indirect_offset(slot.0),
                        face_offset: self.surfaces.face_offset(slot.0),
                        draw_id: slot.0,
                        reverse_winding: vertex.parity() ^ node_is_odd,
                    });
                }
            }
        }
        unsafe {
            self.extraction_scratch.extract(
                device,
                &self.surface_extraction,
                self.surfaces.indirect_buffer(),
                self.surfaces.face_buffer(),
                cmd,
                &extractions,
            );
        }
        histogram!("frame.cpu.voxels.node_scan").record(node_scan_started.elapsed());
    }

    pub unsafe fn draw(
        &mut self,
        device: &Device,
        loader: &Loader,
        common_ds: vk::DescriptorSet,
        frame: &Frame,
        cmd: vk::CommandBuffer,
    ) {
        unsafe {
            let started = Instant::now();
            if !self.draw.bind(
                device,
                loader,
                self.surfaces.dimension(),
                common_ds,
                &frame.surface,
                cmd,
            ) {
                return;
            }
            for chunk in &frame.drawn {
                self.draw.draw(device, cmd, &self.surfaces, chunk.0);
            }
            histogram!("frame.cpu.voxels.draw").record(started.elapsed());
        }
    }

    pub unsafe fn destroy(&mut self, device: &Device) {
        unsafe {
            self.surface_extraction.destroy(device);
            self.extraction_scratch.destroy(device);
            self.surfaces.destroy(device);
            self.draw.destroy(device);
        }
    }
}

pub struct Frame {
    surface: surface::Frame,
    /// Scratch slots completed in this frame
    extracted: Vec<u32>,
    drawn: Vec<SlotId>,
}

impl Frame {
    pub unsafe fn destroy(&mut self, device: &Device) {
        unsafe {
            self.surface.destroy(device);
        }
    }
}

impl Frame {
    pub fn new(gfx: &Base, ctx: &Voxels) -> Self {
        Self {
            surface: surface::Frame::new(gfx, ctx.states.capacity()),
            extracted: Vec::new(),
            drawn: Vec::new(),
        }
    }
}

/// Maximum number of concurrently drawn voxel chunks
const MAX_CHUNKS: u32 = 8192;

struct SurfaceState {
    node: NodeId,
    chunk: common::dodeca::Vertex,
    refcount: u32,
}
