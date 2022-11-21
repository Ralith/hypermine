use std::collections::{HashSet, VecDeque};

use crate::{
    math::HyperboloidMatrix,
    penta::Vertex,
    tessellation::{ChunkData, NodeHandle, Tessellation},
};

pub trait ChunkRayTracer {
    fn trace_ray_in_chunk(
        &self,
        chunk_data: ChunkData,
        pos: &na::Vector3<f64>,
        dir: &na::Vector3<f64>,
        handle: &mut RayTracingResultHandle,
    );

    fn max_radius(&self) -> f64;

    fn trace_ray_in_tessellation(
        &self,
        tessellation: &Tessellation,
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
        let klein_boundary0 = (self.max_radius() + EPSILON).tanh();
        let klein_boundary1 =
            (Vertex::voxel_to_square_factor().atanh() - (self.max_radius() + EPSILON)).tanh();

        while let Some((chunk, transform)) = chunk_queue.pop_front() {
            let chunk_data = tessellation.get_chunk_data(chunk.node, chunk.vertex);
            let square_pos = chunk.vertex.penta_to_square() * transform * pos;
            let square_dir = chunk.vertex.penta_to_square() * transform * dir;
            self.trace_ray_in_chunk(
                chunk_data,
                &square_pos,
                &square_dir,
                &mut handle
                    .dependent_handle(transform.iso_inverse() * chunk.vertex.square_to_penta()),
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
}

pub struct RayTracingResult {
    pub t: f64,
    pub intersection: Option<RayTracingIntersection>,
}

impl RayTracingResult {
    pub fn new(t: f64) -> Self {
        RayTracingResult {
            t,
            intersection: None,
        }
    }
}

pub struct RayTracingIntersection {
    pub voxel_coords: [usize; 2],
    pub normal: na::Vector3<f64>,
}

pub struct RayTracingResultHandle<'a> {
    result: &'a mut RayTracingResult,
    transform: na::Matrix3<f64>,
}

impl<'a> RayTracingResultHandle<'a> {
    pub fn t(&self) -> f64 {
        self.result.t
    }

    pub fn new(result: &'a mut RayTracingResult, transform: na::Matrix3<f64>) -> Self {
        RayTracingResultHandle { result, transform }
    }

    pub fn update(&mut self, t: f64, voxel_coords: [usize; 2], normal: na::Vector3<f64>) {
        self.result.t = t;
        self.result.intersection = Some(RayTracingIntersection {
            voxel_coords,
            normal: self.transform * normal,
        });
    }

    pub fn dependent_handle(&mut self, transform: na::Matrix3<f64>) -> RayTracingResultHandle {
        RayTracingResultHandle {
            result: self.result,
            transform: self.transform * transform,
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
