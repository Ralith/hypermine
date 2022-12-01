use common::{dodeca::Vertex, graph::NodeId, math};

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
        if handle.node() == self.node
            && handle.vertex() == self.vertex
            && SingleBlockSphereCollisionCheckingPass::new(
                pos,
                self.radius,
                self.coords,
                voxel_data.dimension(),
            )
            .any_intersection()
        {
            handle.update(0.0, self.coords, 0, 0, na::Vector4::zeros());
        }
    }

    fn max_radius(&self) -> f64 {
        self.radius
    }
}

struct SingleBlockSphereCollisionCheckingPass<'a> {
    pos: &'a na::Vector4<f64>,
    radius: f64,
    coords: [usize; 3],
    dimension: usize,
}

impl SingleBlockSphereCollisionCheckingPass<'_> {
    fn new(
        pos: &na::Vector4<f64>,
        radius: f64,
        coords: [usize; 3],
        dimension: usize,
    ) -> SingleBlockSphereCollisionCheckingPass<'_> {
        SingleBlockSphereCollisionCheckingPass {
            pos,
            radius,
            coords,
            dimension,
        }
    }

    fn any_intersection(&mut self) -> bool {
        if self.volume_intersection() {
            return true;
        }

        for i in 0..3 {
            if self.side_intersection(i) {
                return true;
            }
        }

        for i in 0..3 {
            if self.edge_intersection(i) {
                return true;
            }
        }

        false
    }

    fn volume_intersection(&mut self) -> bool {
        let chunk_coords =
            self.pos.xyz() / self.pos.w * Vertex::dual_to_chunk_factor() * self.dimension as f64;
        (0..3).all(|coord| {
            chunk_coords[coord] > self.coords[coord] as f64
                && chunk_coords[coord] < self.coords[coord] as f64 + 1.0
        })
    }

    fn side_intersection(&mut self, coord_axis: usize) -> bool {
        let float_size = self.dimension as f64;
        let coord_plane0 = (coord_axis + 1) % 3;
        let coord_plane1 = (coord_axis + 2) % 3;

        for i in self.coords[coord_axis]..=self.coords[coord_axis] + 1 {
            let mut normal = na::Vector4::zeros();
            normal[coord_axis] = 1.0;
            normal[3] = i as f64 / float_size * Vertex::chunk_to_dual_factor();
            let normal = math::lorentz_normalize(&normal);

            let sinh_distance = math::mip(self.pos, &normal);

            if sinh_distance.powi(2) <= self.radius.sinh().powi(2) {
                let projected_pos = math::project_ortho(self.pos, &normal);
                let projected_pos = projected_pos.xyz() / projected_pos.w;
                let j0 = projected_pos[coord_plane0] * Vertex::dual_to_chunk_factor() * float_size;
                let j1 = projected_pos[coord_plane1] * Vertex::dual_to_chunk_factor() * float_size;

                if j0 >= self.coords[coord_plane0] as f64
                    && j0 <= self.coords[coord_plane0] as f64 + 1.0
                    && j1 >= self.coords[coord_plane1] as f64
                    && j1 <= self.coords[coord_plane1] as f64 + 1.0
                {
                    return true;
                }
            }
        }

        false
    }

    fn edge_intersection(&mut self, coord_axis: usize) -> bool {
        let float_size = self.dimension as f64;
        let coord_plane0 = (coord_axis + 1) % 3;
        let coord_plane1 = (coord_axis + 2) % 3;

        for i in self.coords[coord_plane0]..=self.coords[coord_plane0] + 1 {
            for j in self.coords[coord_plane1]..=self.coords[coord_plane1] + 1 {
                let mut edge_pos = na::Vector4::zeros();
                let mut edge_dir = na::Vector4::zeros();
                edge_pos[coord_plane0] = i as f64 / float_size * Vertex::chunk_to_dual_factor();
                edge_pos[coord_plane1] = j as f64 / float_size * Vertex::chunk_to_dual_factor();
                edge_pos[3] = 1.0;
                edge_pos = math::lorentz_normalize(&edge_pos);
                edge_dir[coord_axis] = 1.0;

                let cosh_distance_squared =
                    math::mip(self.pos, &edge_pos).powi(2) - math::mip(self.pos, &edge_dir).powi(2);

                if cosh_distance_squared <= self.radius.cosh().powi(2) {
                    let projected_pos = -edge_pos * math::mip(self.pos, &edge_pos)
                        + edge_dir * math::mip(self.pos, &edge_dir);
                    let projected_pos = projected_pos / projected_pos.w;

                    let k = projected_pos[coord_axis] * Vertex::dual_to_chunk_factor() * float_size;
                    if k >= self.coords[coord_axis] as f64
                        && k <= self.coords[coord_axis] as f64 + 1.0
                    {
                        return true;
                    }
                }
            }
        }

        false
    }
}
