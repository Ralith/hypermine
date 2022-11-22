use std::collections::{HashSet, VecDeque};

use crate::{
    chunk_ray_tracer::{ChunkRayTracer, RayTracingResultHandle},
    math::HyperboloidMatrix,
    penta::Vertex,
    tessellation::{NodeHandle, Tessellation},
};

pub fn trace_ray(
    tessellation: &Tessellation,
    chunk_ray_tracer: &impl ChunkRayTracer,
    node: NodeHandle,
    pos: &na::Vector3<f64>,
    dir: &na::Vector3<f64>,
    handle: &mut RayTracingResultHandle,
) {
    let mut visited_chunks: HashSet<ChunkHandle> = HashSet::new();
    let mut chunk_queue: VecDeque<(ChunkHandle, na::Matrix3<f64>)> = VecDeque::new();

    let start_chunk = ChunkHandle::new(node, Vertex::AB);
    visited_chunks.insert(start_chunk);
    chunk_queue.push_back((start_chunk, na::Matrix3::identity()));

    const EPSILON: f64 = 1e-5;
    let klein_boundary0 = (chunk_ray_tracer.max_radius() + EPSILON).tanh();
    let klein_boundary1 = (Vertex::voxel_to_square_factor().atanh()
        - (chunk_ray_tracer.max_radius() + EPSILON))
        .tanh();

    while let Some((chunk, transform)) = chunk_queue.pop_front() {
        let chunk_data = tessellation.get_chunk_data(chunk.node, chunk.vertex);
        let square_pos = chunk.vertex.penta_to_square() * transform * pos;
        let square_dir = chunk.vertex.penta_to_square() * transform * dir;
        chunk_ray_tracer.trace_ray_in_chunk(
            chunk_data,
            &square_pos,
            &square_dir,
            &mut handle.dependent_handle(transform.iso_inverse() * chunk.vertex.square_to_penta()),
        );

        // If pos or pos+dir*max_t lies beyond the chunk boundary, with a buffer to account for radius, repeat
        // ray tracing with the neighboring chunk unless it has already been visited. We start at vertex
        // AB for simplicity even if that's not where pos is, although this should be optimized later.
        for coord_boundary in 0..2 {
            let klein_pos0_val = square_pos[coord_boundary] / square_pos[2];
            let klein_pos1_val = (square_pos[coord_boundary]
                + square_dir[coord_boundary] * handle.t())
                / (square_pos[2] + square_dir[2] * handle.t());

            // Check for neighboring nodes. TODO: The use of unwrap here will cause a crash if you arrive at an ungenerated chunk.
            if klein_pos0_val <= klein_boundary0 || klein_pos1_val <= klein_boundary0 {
                let side = chunk.vertex.sides()[coord_boundary];
                let next_chunk = ChunkHandle::new(
                    tessellation.get_neighbor(chunk.node, side).unwrap(),
                    chunk.vertex,
                );
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
}

#[derive(Clone, Copy, PartialEq, Eq, Hash)]
struct ChunkHandle {
    node: NodeHandle,
    vertex: Vertex,
}

impl ChunkHandle {
    fn new(node: NodeHandle, vertex: Vertex) -> Self {
        ChunkHandle { node, vertex }
    }
}
