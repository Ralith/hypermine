mod surface;
mod surface_extraction;

#[cfg(test)]
mod tests;

use std::{sync::Arc, time::Instant};

use ash::{vk, Device};
use metrics::timing;
use tracing::warn;

use crate::{
    graphics::{Base, Frustum},
    loader::{Cleanup, LoadCtx, LoadFuture, Loadable, WorkQueue},
    sim::VoxelData,
    worldgen, Config, Loader, Sim,
};
use common::{dodeca, dodeca::Vertex, graph::NodeId, lru_slab::SlotId, math, LruSlab};

use surface::Surface;
use surface_extraction::{DrawBuffer, ScratchBuffer, SurfaceExtraction};

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

    /// Determine what to render and record the appropriate transfer commands
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
            sim.graph.get_mut(chunk.node).as_mut().unwrap().chunks[chunk.chunk] =
                Chunk::Populated {
                    surface: None,
                    voxels: chunk.voxels,
                };
        }

        // Determine what to load/render
        let view = sim.view();
        if !sim.graph.contains(view.node) {
            // Graph is temporarily out of sync with the server; we don't know where we are, so
            // there's no point trying to draw.
            return;
        }
        let graph_traversal_started = Instant::now();
        let mut nodes = sim
            .graph
            .nearby_nodes(&view, f64::from(self.config.local_simulation.view_distance));
        timing!(
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
        let local_to_view = view.local.try_inverse().unwrap();
        for &(node, ref node_transform) in &nodes {
            let node_to_view = local_to_view * node_transform;
            let origin = node_to_view * math::origin();
            if !frustum_planes.contain(&origin, dodeca::BOUNDING_SPHERE_RADIUS as f32) {
                // Don't bother generating or drawing chunks from nodes that are wholly outside the
                // frustum.
                continue;
            }

            use Chunk::*;
            for chunk in Vertex::iter() {
                // Fetch existing chunk, or extract surface of new chunk
                let slot = match sim
                    .graph
                    .get_mut(node)
                    .as_mut()
                    .expect("all nodes must be populated before rendering")
                    .chunks[chunk]
                {
                    Generating => continue,
                    Fresh => {
                        // Generate voxel data
                        if let Some(params) = worldgen::ChunkParams::new(
                            self.surfaces.dimension() as u8,
                            &sim.graph,
                            node,
                            chunk,
                        ) {
                            if self.worldgen.load(ChunkDesc { node, params }).is_ok() {
                                sim.graph.get_mut(node).as_mut().unwrap().chunks[chunk] =
                                    Generating;
                            }
                        }
                        continue;
                    }
                    Populated {
                        ref mut surface,
                        ref voxels,
                    } => match (surface, voxels) {
                        (&mut Some(x), _) => {
                            // Render an extracted surface
                            self.states.get_mut(x).refcount += 1;
                            x
                        }
                        (&mut ref mut surface @ None, &VoxelData::Dense(ref data)) => {
                            // Extract a surface
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
                                Some(self.states.remove(slot))
                            } else {
                                None
                            };
                            let scratch_slot = self.extraction_scratch.alloc().expect("there are at least chunks_loaded_per_frame scratch slots per frame");
                            frame.extracted.push(scratch_slot);
                            let slot = self.states.insert(SurfaceState {
                                node,
                                chunk,
                                refcount: 1,
                            });
                            *surface = Some(slot);
                            let storage = self.extraction_scratch.storage(scratch_slot);
                            storage.copy_from_slice(&data[..]);
                            if let Some(lru) = removed {
                                if let Populated {
                                    ref mut surface, ..
                                } =
                                    sim.graph.get_mut(lru.node).as_mut().unwrap().chunks[lru.chunk]
                                {
                                    *surface = None;
                                }
                            }
                            let node_is_odd = sim.graph.length(node) & 1 != 0;
                            self.extraction_scratch.extract(
                                device,
                                &self.surface_extraction,
                                scratch_slot,
                                chunk.parity() ^ node_is_odd,
                                slot.0,
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
                frame.surface.transforms_mut()[slot.0 as usize] =
                    node_transform * chunk.chunk_to_node().map(|x| x as f32);
            }
        }
        timing!("frame.cpu.voxels.node_scan", node_scan_started.elapsed());
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
        timing!("frame.cpu.voxels.draw", started.elapsed());
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
    params: worldgen::ChunkParams,
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

pub enum Chunk {
    Fresh,
    Generating,
    Populated {
        voxels: VoxelData,
        surface: Option<SlotId>,
    },
}

impl Default for Chunk {
    fn default() -> Self {
        Chunk::Fresh
    }
}
