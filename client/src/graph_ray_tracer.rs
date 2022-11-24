use std::collections::{HashSet, VecDeque};

use common::dodeca::Vertex;
use common::graph::{Graph, NodeId};
use common::node::{Chunk, Node};

use crate::chunk_ray_tracer::{ChunkRayTracer, RayTracingResultHandle, VoxelDataWrapper};

// Returns whether a definitive answer was found
pub fn trace_ray(
    graph: &Graph<Node>,
    dimension: usize,
    chunk_ray_tracer: &impl ChunkRayTracer,
    node: NodeId,
    pos: &na::Vector4<f64>,
    dir: &na::Vector4<f64>,
    handle: &mut RayTracingResultHandle,
) -> bool {
    let mut visited_chunks: HashSet<ChunkHandle> = HashSet::new();
    let mut chunk_queue: VecDeque<(ChunkHandle, na::Matrix4<f64>)> = VecDeque::new();

    let start_chunk = ChunkHandle::new(node, Vertex::A);
    visited_chunks.insert(start_chunk);
    chunk_queue.push_back((start_chunk, na::Matrix4::identity()));

    const EPSILON: f64 = 1e-5;
    let klein_boundary0 = (chunk_ray_tracer.max_radius() + EPSILON).tanh();
    let klein_boundary1 =
        (Vertex::chunk_to_dual_factor().atanh() - (chunk_ray_tracer.max_radius() + EPSILON)).tanh();

    while let Some((chunk, transform)) = chunk_queue.pop_front() {
        let Chunk::Populated {
            voxels: ref voxel_data,
            ..
        } = graph.get(chunk.node).as_ref().unwrap().chunks[chunk.vertex] else {
            // Collision checking on unpopulated chunk
            return false;
        };
        let square_pos = chunk.vertex.node_to_dual() * transform * pos;
        let square_dir = chunk.vertex.node_to_dual() * transform * dir;
        chunk_ray_tracer.trace_ray_in_chunk(
            VoxelDataWrapper::new(voxel_data, dimension),
            &square_pos,
            &square_dir,
            &mut handle.dependent_handle(
                transform.try_inverse().unwrap() * chunk.vertex.dual_to_node().cast(),
            ),
        );

        // If pos or pos+dir*max_t lies beyond the chunk boundary, with a buffer to account for radius, repeat
        // ray tracing with the neighboring chunk unless it has already been visited. We start at vertex
        // AB for simplicity even if that's not where pos is, although this should be optimized later.
        for coord_boundary in 0..3 {
            let klein_pos0_val = square_pos[coord_boundary] / square_pos[3];
            let klein_pos1_val = (square_pos[coord_boundary]
                + square_dir[coord_boundary] * handle.t())
                / (square_pos[3] + square_dir[3] * handle.t());

            // Check for neighboring nodes. TODO: The use of unwrap here will cause a crash if you arrive at an ungenerated chunk.
            if klein_pos0_val <= klein_boundary0 || klein_pos1_val <= klein_boundary0 {
                let side = chunk.vertex.canonical_sides()[coord_boundary];
                let next_chunk =
                    ChunkHandle::new(graph.neighbor(chunk.node, side).unwrap(), chunk.vertex);
                if visited_chunks.insert(next_chunk) {
                    chunk_queue.push_back((next_chunk, side.reflection() * transform));
                }
            }

            // Check for neighboring chunks within the same node
            if klein_pos0_val >= klein_boundary1 || klein_pos1_val >= klein_boundary1 {
                let vertex = chunk.vertex.adjacent_vertices()[coord_boundary];
                let next_chunk = ChunkHandle::new(chunk.node, vertex);
                if visited_chunks.insert(next_chunk) {
                    chunk_queue.push_back((next_chunk, transform));
                }
            }
        }
    }

    true
}

#[derive(Clone, Copy, PartialEq, Eq, Hash)]
struct ChunkHandle {
    node: NodeId,
    vertex: Vertex,
}

impl ChunkHandle {
    fn new(node: NodeId, vertex: Vertex) -> Self {
        ChunkHandle { node, vertex }
    }
}
