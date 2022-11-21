use crate::{
    chunk_ray_tracer::{ChunkRayTracer, RayTracingResultHandle},
    math::HyperboloidVector,
    penta::Vertex,
    tessellation::ChunkData,
};

pub struct SphereChunkRayTracer {
    pub radius: f64,
}

impl SphereChunkRayTracer {
    fn trace_ray_for_sides(
        &self,
        chunk_data: ChunkData,
        pos: &na::Vector3<f64>,
        dir: &na::Vector3<f64>,
        handle: &mut RayTracingResultHandle,
        coord_axis: usize,
    ) {
        let float_size = chunk_data.chunk_size() as f64;
        let coord_plane0 = (coord_axis + 1) % 2;

        for i in 0..=chunk_data.chunk_size() {
            // Factor a in plane equation x/z == a => x == a*z
            let a = i as f64 / float_size * Vertex::voxel_to_square_factor();

            // Solve quadratic equation
            let mut normal = na::Vector3::new(0.0, 0.0, a);
            normal[coord_axis] = 1.0;
            let normal = normal.m_normalized_vector();
            let mip_pos_norm = pos.mip(&normal);
            let mip_dir_norm = dir.mip(&normal);
            let sinh_radius_squared = self.radius.sinh().powi(2);
            let quadratic_term = mip_dir_norm * mip_dir_norm + sinh_radius_squared;
            let double_linear_term = mip_pos_norm * mip_dir_norm;
            let constant_term = mip_pos_norm * mip_pos_norm - sinh_radius_squared;
            let discriminant =
                double_linear_term * double_linear_term - quadratic_term * constant_term;

            if discriminant < 0.0 {
                continue;
            }

            let t_candidate = (-double_linear_term - discriminant.sqrt()) / quadratic_term;

            if t_candidate >= 0.0 && t_candidate < handle.t() {
                let i_with_offset = if mip_dir_norm < 0.0 { i } else { i + 1 };
                if i_with_offset > 0 && i_with_offset <= chunk_data.chunk_size() {
                    let i_with_offset = i_with_offset - 1;
                    let translated_square_pos = pos + dir * t_candidate;
                    let b = translated_square_pos[coord_plane0] * (1.0 - a * a)
                        / (translated_square_pos.z - translated_square_pos[coord_axis] * a);
                    let j = (b * Vertex::square_to_voxel_factor() * float_size).floor();
                    if j >= 0.0 && j < float_size {
                        let j = j as usize;
                        if chunk_data.get2(coord_axis, i_with_offset, coord_plane0, j) != 0 {
                            handle.update(
                                t_candidate,
                                [0, 0], /* TODO */
                                normal * -mip_dir_norm.signum(),
                            );
                        }
                    }
                }
            }
        }
    }

    fn trace_ray_for_vertices(
        &self,
        chunk_data: ChunkData,
        pos: &na::Vector3<f64>,
        dir: &na::Vector3<f64>,
        handle: &mut RayTracingResultHandle,
    ) {
        let size = chunk_data.chunk_size();
        let float_size = size as f64;

        for i in 0..=chunk_data.chunk_size() {
            for j in 0..=chunk_data.chunk_size() {
                if (i == 0 || j == 0 || chunk_data.get(i - 1, j - 1) == 0)
                    && (i == size || j == 0 || chunk_data.get(i, j - 1) == 0)
                    && (i == 0 || j == size || chunk_data.get(i - 1, j) == 0)
                    && (i == size || j == size || chunk_data.get(i, j) == 0)
                {
                    continue;
                }

                let vert = na::Vector3::new(
                    i as f64 / float_size * Vertex::voxel_to_square_factor(),
                    j as f64 / float_size * Vertex::voxel_to_square_factor(),
                    1.0,
                )
                .m_normalized_point();

                let mip_pos_vert = pos.mip(&vert);
                let mip_dir_vert = dir.mip(&vert);
                let cosh_radius_squared = self.radius.cosh().powi(2);
                let quadratic_term = mip_dir_vert * mip_dir_vert + cosh_radius_squared;
                let double_linear_term = mip_pos_vert * mip_dir_vert;
                let constant_term = mip_pos_vert * mip_pos_vert - cosh_radius_squared;
                let discriminant =
                    double_linear_term * double_linear_term - quadratic_term * constant_term;

                if discriminant < 0.0 {
                    continue;
                }

                let t_candidate = (-double_linear_term - discriminant.sqrt()) / quadratic_term;

                if t_candidate >= 0.0 && t_candidate < handle.t() {
                    let translated_square_pos = pos + dir * t_candidate;
                    handle.update(
                        t_candidate,
                        [0, 0], /* TODO */
                        translated_square_pos - vert,
                    );
                }
            }
        }
    }
}

impl ChunkRayTracer for SphereChunkRayTracer {
    fn trace_ray_in_chunk(
        &self,
        chunk_data: ChunkData,
        pos: &na::Vector3<f64>,
        dir: &na::Vector3<f64>,
        handle: &mut RayTracingResultHandle,
    ) {
        for coord_axis in 0..2 {
            self.trace_ray_for_sides(chunk_data, pos, dir, handle, coord_axis);
        }

        self.trace_ray_for_vertices(chunk_data, pos, dir, handle);
    }

    fn max_radius(&self) -> f64 {
        self.radius
    }
}
