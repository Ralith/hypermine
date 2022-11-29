use common::{dodeca::Vertex, math, world::Material};

use crate::chunk_ray_tracer::{ChunkRayTracer, RayTracingResultHandle, VoxelDataWrapper};

pub struct PointChunkRayTracer {}

impl ChunkRayTracer for PointChunkRayTracer {
    fn trace_ray_in_chunk(
        &self,
        voxel_data: VoxelDataWrapper,
        pos: &na::Vector4<f64>,
        dir: &na::Vector4<f64>,
        handle: &mut RayTracingResultHandle,
    ) {
        PointChunkRayTracingPass::new(voxel_data, pos, dir, handle).trace_ray_in_chunk();
    }

    fn max_radius(&self) -> f64 {
        0.0
    }
}

struct PointChunkRayTracingPass<'a, 'b> {
    voxel_data: VoxelDataWrapper<'a>,
    pos: &'a na::Vector4<f64>,
    dir: &'a na::Vector4<f64>,
    handle: &'a mut RayTracingResultHandle<'b>,

    // Start and end of region to check in voxel coordinates
    // TODO: These can be used for more fine-tuned pre-collision-check filtering
    #[allow(dead_code)]
    voxel_start: na::Vector3<f64>,
    #[allow(dead_code)]
    voxel_end: na::Vector3<f64>,

    // Bounding box of all voxels that can be collided with
    // [[xmin, xmax], [ymin, ymax], [zmin, zmax]]
    bbox: [[usize; 2]; 3],
}

impl PointChunkRayTracingPass<'_, '_> {
    fn new<'a, 'b>(
        voxel_data: VoxelDataWrapper<'a>,
        pos: &'a na::Vector4<f64>,
        dir: &'a na::Vector4<f64>,
        handle: &'a mut RayTracingResultHandle<'b>,
    ) -> PointChunkRayTracingPass<'a, 'b> {
        let float_size = voxel_data.dimension() as f64;
        let voxel_start = (pos / pos[3]).xyz() * Vertex::dual_to_chunk_factor() * float_size;
        let end_pos = pos + dir * handle.t();
        let voxel_end = (end_pos / end_pos[3]).xyz() * Vertex::dual_to_chunk_factor() * float_size;
        let bbox = [0, 1, 2].map(|coord| {
            get_usize_range(
                0,
                voxel_data.dimension(),
                voxel_start[coord],
                voxel_end[coord],
                0.0,
            )
        });

        PointChunkRayTracingPass {
            voxel_data,
            pos,
            dir,
            handle,
            voxel_start,
            voxel_end,
            bbox,
        }
    }

    fn trace_ray_in_chunk(&mut self) {
        for coord_axis in 0..3 {
            self.trace_ray_for_sides(coord_axis);
        }
    }

    fn trace_ray_for_sides(&mut self, coord_axis: usize) {
        let float_size = self.voxel_data.dimension() as f64;
        let coord_plane0 = (coord_axis + 1) % 3;
        let coord_plane1 = (coord_axis + 2) % 3;

        for i in self.bbox[coord_axis][0]..=self.bbox[coord_axis][1] {
            let mut normal = na::Vector4::zeros();
            normal[coord_axis] = 1.0;
            normal[3] = i as f64 / float_size * Vertex::chunk_to_dual_factor();
            let normal = math::lorentz_normalize(&normal);
            let t_candidate = -math::mip(self.pos, &normal) / math::mip(self.dir, &normal);

            // If t_candidate is out of range or NaN, don't continue collision checking
            if !(t_candidate >= 0.0 && t_candidate < self.handle.t()) {
                continue;
            }

            let mip_dir_norm = math::mip(self.dir, &normal);
            let i_with_offset = if mip_dir_norm < 0.0 { i } else { i + 1 };
            if !(i_with_offset > 0 && i_with_offset <= self.voxel_data.dimension()) {
                continue;
            }
            let i_with_offset = i_with_offset - 1;

            let translated_square_pos = self.pos + self.dir * t_candidate;
            let translated_square_pos = translated_square_pos.xyz() / translated_square_pos.w;
            let j0 =
                (translated_square_pos[coord_plane0] * Vertex::dual_to_chunk_factor() * float_size)
                    .floor();
            let j1 =
                (translated_square_pos[coord_plane1] * Vertex::dual_to_chunk_factor() * float_size)
                    .floor();
            if j0 >= 0.0 && j0 < float_size && j1 >= 0.0 && j1 < float_size {
                let j0 = j0 as usize;
                let j1 = j1 as usize;
                let mut coords = [0; 3];
                coords[coord_axis] = i_with_offset;
                coords[coord_plane0] = j0;
                coords[coord_plane1] = j1;
                if self.voxel_data.get(coords) != Material::Void {
                    self.handle.update(
                        t_candidate,
                        coords,
                        coord_axis,
                        -mip_dir_norm.signum() as isize,
                        normal * -mip_dir_norm.signum(),
                    );
                }
            }
        }
    }
}

fn get_usize_range(min: usize, max: usize, point0: f64, point1: f64, width: f64) -> [usize; 2] {
    if !point0.is_finite() || !point1.is_finite() {
        return [min, max];
    }
    let result_min = (point0.min(point1) - width).max(min as f64);
    let result_max = (point0.max(point1) + width).min(max as f64);

    if result_min > result_max {
        // Empty range
        return [1, 0];
    }

    [result_min.ceil() as usize, result_max.floor() as usize]
}
