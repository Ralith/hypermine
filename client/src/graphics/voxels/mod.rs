mod surface;
pub mod surface_extraction;

#[cfg(test)]
mod tests;

use std::{sync::Arc, time::Instant};

use ash::{vk, Device};
use metrics::histogram;
use tracing::warn;

use crate::{
    graphics::{Base, Frustum},
    loader::{Cleanup, LoadCtx, LoadFuture, Loadable, WorkQueue},
    Config, Loader, Sim,
};
use common::{
    dodeca,
    dodeca::Vertex,
    graph::NodeId,
    lru_slab::SlotId,
    math,
    node::{Chunk, ChunkId, VoxelData},
    traversal::nearby_nodes,
    LruSlab,
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
    worldgen: WorkQueue<ChunkDesc>,
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
            worldgen: loader.make_queue(config.chunk_load_parallelism as usize),
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
        while let Some(chunk) = self.worldgen.poll() {
            let chunk_id = ChunkId::new(chunk.node, chunk.chunk);
            sim.graph.populate_chunk(chunk_id, chunk.voxels, false);

            // Now that the block is populated, we can apply any pending block updates the server
            // provided that the client couldn't apply.
            if let Some(block_updates) = sim.preloaded_block_updates.remove(&chunk_id) {
                for block_update in block_updates {
                    // The chunk was just populated, so a block update should always succeed.
                    assert!(sim.graph.update_block(&block_update));
                }
            }
        }

        // Determine what to load/render
        let view = sim.view();
        if !sim.graph.contains(view.node) {
            // Graph is temporarily out of sync with the server; we don't know where we are, so
            // there's no point trying to draw.
            return;
        }
        let graph_traversal_started = Instant::now();
        let mut nodes = nearby_nodes(
            &sim.graph,
            &view,
            f64::from(self.config.local_simulation.view_distance),
        );
        histogram!(
            "frame.cpu.voxels.graph_traversal",
            graph_traversal_started.elapsed()
        );
        // Sort nodes by distance to the view to prioritize loading closer data and improve early Z
        // performance
        let view_pos = view.local * math::origin();
        nodes.sort_unstable_by(|&(_, ref xf_a), &(_, ref xf_b)| {
            math::distance(&view_pos, &(xf_a * math::origin()))
                .partial_cmp(&math::distance(&view_pos, &(xf_b * math::origin())))
                .unwrap_or(std::cmp::Ordering::Less)
        });
        let node_scan_started = Instant::now();
        let frustum_planes = frustum.planes();
        let local_to_view = math::mtranspose(&view.local);
        let mut extractions = Vec::new();
        for &(node, ref node_transform) in &nodes {
            let node_to_view = local_to_view * node_transform;
            let origin = node_to_view * math::origin();
            if !frustum_planes.contain(&origin, dodeca::BOUNDING_SPHERE_RADIUS as f32) {
                // Don't bother generating or drawing chunks from nodes that are wholly outside the
                // frustum.
                continue;
            }

            use Chunk::*;
            for vertex in Vertex::iter() {
                let chunk = ChunkId::new(node, vertex);
                // Fetch existing chunk, or extract surface of new chunk
                match sim
                    .graph
                    .get_chunk_mut(chunk)
                    .expect("all nodes must be populated before rendering")
                {
                    Generating => continue,
                    Fresh => {
                        // Generate voxel data
                        if let Some(params) = common::worldgen::ChunkParams::new(
                            self.surfaces.dimension() as u8,
                            &sim.graph,
                            chunk,
                        ) {
                            if self.worldgen.load(ChunkDesc { node, params }).is_ok() {
                                sim.graph[chunk] = Generating;
                            }
                        }
                        continue;
                    }
                    Populated {
                        ref mut surface,
                        ref mut old_surface,
                        ref voxels,
                        ..
                    } => {
                        if let Some(slot) = surface.or(*old_surface) {
                            // Render an already-extracted surface
                            self.states.get_mut(slot).refcount += 1;
                            frame.drawn.push(slot);
                            // Transfer transform
                            frame.surface.transforms_mut()[slot.0 as usize] =
                                node_transform * vertex.chunk_to_node().map(|x| x as f32);
                        }
                        if let (None, &VoxelData::Dense(ref data)) = (&surface, voxels) {
                            // Extract a surface so it can be drawn in future frames
                            if frame.extracted.len() == self.config.chunk_load_parallelism as usize
                            {
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
                            let scratch_slot = self.extraction_scratch.alloc().expect("there are at least chunks_loaded_per_frame scratch slots per frame");
                            frame.extracted.push(scratch_slot);
                            let slot = self.states.insert(SurfaceState {
                                node,
                                chunk: vertex,
                                refcount: 0,
                            });
                            *surface = Some(slot);
                            let storage = self.extraction_scratch.storage(scratch_slot);
                            storage.copy_from_slice(&data[..]);
                            if let Some((lru_slot, lru)) = removed {
                                if let Populated {
                                    ref mut surface,
                                    ref mut old_surface,
                                    ..
                                } =
                                    sim.graph.get_mut(lru.node).as_mut().unwrap().chunks[lru.chunk]
                                {
                                    // Remove references to released slot IDs
                                    if surface.map_or(false, |slot| lru_slot == slot) {
                                        *surface = None;
                                    }
                                    if old_surface.map_or(false, |slot| lru_slot == slot) {
                                        *old_surface = None;
                                    }
                                }
                            }
                            let node_is_odd = sim.graph.length(node) & 1 != 0;
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
            }
        }
        self.extraction_scratch.extract(
            device,
            &self.surface_extraction,
            self.surfaces.indirect_buffer(),
            self.surfaces.face_buffer(),
            cmd,
            &extractions,
        );
        histogram!("frame.cpu.voxels.node_scan", node_scan_started.elapsed());
    }

    pub unsafe fn draw(
        &mut self,
        device: &Device,
        loader: &Loader,
        common_ds: vk::DescriptorSet,
        frame: &Frame,
        cmd: vk::CommandBuffer,
    ) {
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
        histogram!("frame.cpu.voxels.draw", started.elapsed());
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
    drawn: Vec<SlotId>,
}

impl Frame {
    pub unsafe fn destroy(&mut self, device: &Device) {
        self.surface.destroy(device);
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

struct ChunkDesc {
    node: NodeId,
    params: common::worldgen::ChunkParams,
}

struct LoadedChunk {
    node: NodeId,
    chunk: Vertex,
    voxels: VoxelData,
}

impl Cleanup for LoadedChunk {
    unsafe fn cleanup(self, _gfx: &Base) {}
}

impl Loadable for ChunkDesc {
    type Output = LoadedChunk;
    fn load(self, _ctx: &LoadCtx) -> LoadFuture<'_, Self::Output> {
        Box::pin(async move {
            Ok(LoadedChunk {
                node: self.node,
                chunk: self.params.chunk(),
                voxels: self.params.generate_voxels(),
            })
        })
    }
}
