use common::{dodeca::Vertex, graph::NodeId};

use crate::chunk_ray_tracer::{ChunkRayTracer, RayTracingResultHandle, VoxelDataWrapper};

/// This is an abuse of the ChunkRayTracer trait, where t is always set to 0, and collisions are
/// checked with a single block location to ensure that a block can be placed without causing clipping.
pub struct SingleBlockSphereCollisionChecker {
    pub node: NodeId,
    pub vertex: Vertex,
    pub coords: [usize; 3],
    pub radius: f64,
}

impl ChunkRayTracer for SingleBlockSphereCollisionChecker {
    fn trace_ray_in_chunk(
        &self,
        voxel_data: VoxelDataWrapper,
        pos: &na::Vector4<f64>,
        _dir: &na::Vector4<f64>,
        handle: &mut RayTracingResultHandle,
    ) {
        if handle.node() == self.node && handle.vertex() == self.vertex {
            SingleBlockSphereCollisionCheckingPass::new(
                pos,
                handle,
                self.coords,
                voxel_data.dimension(),
            )
            .trace_ray_in_chunk();
        }
    }

    fn max_radius(&self) -> f64 {
        self.radius
    }
}

struct SingleBlockSphereCollisionCheckingPass<'a, 'b> {
    pos: &'a na::Vector4<f64>,
    handle: &'a mut RayTracingResultHandle<'b>,
    coords: [usize; 3],
    dimension: usize,
}

impl SingleBlockSphereCollisionCheckingPass<'_, '_> {
    fn new<'a, 'b>(
        pos: &'a na::Vector4<f64>,
        handle: &'a mut RayTracingResultHandle<'b>,
        coords: [usize; 3],
        dimension: usize,
    ) -> SingleBlockSphereCollisionCheckingPass<'a, 'b> {
        SingleBlockSphereCollisionCheckingPass {
            pos,
            handle,
            coords,
            dimension,
        }
    }

    fn trace_ray_in_chunk(&mut self) {
        // TODO: Check for sphere intersection instead of point intersection
        let chunk_coords =
            self.pos.xyz() / self.pos.w * Vertex::dual_to_chunk_factor() * self.dimension as f64;
        if (0..3).all(|coord| {
            chunk_coords[coord] > self.coords[coord] as f64
                && chunk_coords[coord] < self.coords[coord] as f64 + 1.0
        }) {
            self.handle
                .update(0.0, self.coords, 0, 0, na::Vector4::zeros());
        }
    }
}
