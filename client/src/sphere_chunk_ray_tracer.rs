use common::{dodeca::Vertex, math, world::Material};

use crate::chunk_ray_tracer::{ChunkRayTracer, RayTracingResultHandle, VoxelDataWrapper};

pub struct SphereChunkRayTracer {
    pub radius: f64,
}

impl SphereChunkRayTracer {
    fn trace_ray_for_sides(
        &self,
        voxel_data: VoxelDataWrapper,
        pos: &na::Vector4<f64>,
        dir: &na::Vector4<f64>,
        handle: &mut RayTracingResultHandle,
        coord_axis: usize,
    ) {
        let float_size = voxel_data.dimension() as f64;
        let coord_plane0 = (coord_axis + 1) % 3;
        let coord_plane1 = (coord_axis + 2) % 3;

        for i in 0..=voxel_data.dimension() {
            let mut normal = na::Vector4::new(
                0.0,
                0.0,
                0.0,
                i as f64 / float_size * Vertex::chunk_to_dual_factor(),
            );
            normal[coord_axis] = 1.0;
            let normal = math::lorentz_normalize(&normal);
            let t_candidate =
                Self::find_intersection_one_vector(pos, dir, &normal, self.radius.sinh());

            // If t_candidate is out of range or NaN, don't continue collision checking
            if !(t_candidate >= 0.0 && t_candidate < handle.t()) {
                continue;
            }

            let mip_dir_norm = math::mip(dir, &normal);
            let i_with_offset = if mip_dir_norm < 0.0 { i } else { i + 1 };
            if !(i_with_offset > 0 && i_with_offset <= voxel_data.dimension()) {
                continue;
            }
            let i_with_offset = i_with_offset - 1;

            let translated_square_pos = pos + dir * t_candidate;
            let projected_pos = math::project_ortho(&translated_square_pos, &normal);
            let projected_pos = projected_pos.xyz() / projected_pos.w;
            let j0 =
                (projected_pos[coord_plane0] * Vertex::dual_to_chunk_factor() * float_size).floor();
            let j1 =
                (projected_pos[coord_plane1] * Vertex::dual_to_chunk_factor() * float_size).floor();
            if j0 >= 0.0 && j0 < float_size && j1 >= 0.0 && j1 < float_size {
                let j0 = j0 as usize;
                let j1 = j1 as usize;
                let mut coords = [0; 3];
                coords[coord_axis] = i_with_offset;
                coords[coord_plane0] = j0;
                coords[coord_plane1] = j1;
                if voxel_data.get(coords) != Material::Void {
                    handle.update(
                        t_candidate,
                        [0, 0, 0], /* TODO */
                        normal * -mip_dir_norm.signum(),
                    );
                }
            }
        }
    }

    /*fn trace_ray_for_vertices(
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
                if (i == 0 || j == 0 || chunk_data.get([i - 1, j - 1]) == 0)
                    && (i == size || j == 0 || chunk_data.get([i, j - 1]) == 0)
                    && (i == 0 || j == size || chunk_data.get([i - 1, j]) == 0)
                    && (i == size || j == size || chunk_data.get([i, j]) == 0)
                {
                    continue;
                }

                let vert = na::Vector3::new(
                    i as f64 / float_size * Vertex::voxel_to_square_factor(),
                    j as f64 / float_size * Vertex::voxel_to_square_factor(),
                    1.0,
                )
                .m_normalized_point();

                let t_candidate =
                    Self::find_intersection_one_vector(pos, dir, &vert, self.radius.sinh());

                // If t_candidate is out of range or NaN, don't continue collision checking
                if !(t_candidate >= 0.0 && t_candidate < handle.t()) {
                    continue;
                }

                let translated_square_pos = pos + dir * t_candidate;
                handle.update(
                    t_candidate,
                    [0, 0], /* TODO */
                    translated_square_pos - vert,
                );
            }
        }
    }*/

    /// Find the smallest value of `t` where the point in the pos-dir line (v=pos+dir*t) satisfies
    /// `<v,a>^2 / <v,v> == c^2`
    ///
    /// If `a` is direction-like, this finds intersections with a surface that is `sinh(c)` units
    /// away from the plane whose normal is `a`.  If `a` is point-like, this finds intersections
    /// with a sphere centered at `a` with radius `cosh(c)`.
    fn find_intersection_one_vector(
        pos: &na::Vector4<f64>,
        dir: &na::Vector4<f64>,
        a: &na::Vector4<f64>,
        c: f64,
    ) -> f64 {
        let mip_pos_a = math::mip(pos, a);
        let mip_dir_a = math::mip(dir, a);

        // The following 3 variables are terms of the quadratic formula. We use double the linear
        // term because it removes the annoying constants from that formula.
        let quadratic_term = mip_dir_a.powi(2) + c.powi(2);
        let double_linear_term = mip_pos_a * mip_dir_a;
        let constant_term = mip_pos_a.powi(2) - c.powi(2);

        let discriminant = double_linear_term * double_linear_term - quadratic_term * constant_term;

        // While discriminant can be negative, NaNs propagate the way we want to, so we don't have
        // to check for this.
        (-double_linear_term - discriminant.sqrt()) / quadratic_term
    }

    /// Find the smallest value of `t` where the point in the pos-dir line (v=pos+dir*t) satisfies
    /// `(<v,a>^2 - <v,b>^2) / <v,v> == c`
    ///
    /// This finds intersections with a surface that is `cosh^2(c)` units away from the line
    /// with a point at `a` with direction `b`, where `<a,b>==0`. This method is redundant in 2D,
    /// but it will be needed in 3D.
    ///
    /// Returns NaN if there's no such intersection
    #[allow(dead_code)]
    fn find_intersection_two_vectors(
        pos: &na::Vector4<f64>,
        dir: &na::Vector4<f64>,
        a: &na::Vector4<f64>,
        b: &na::Vector4<f64>,
        c: f64,
    ) -> f64 {
        // TODO: Improve numerical stability
        let mip_pos_a = math::mip(pos, a);
        let mip_dir_a = math::mip(dir, a);
        let mip_pos_b = math::mip(pos, b);
        let mip_dir_b = math::mip(dir, b);

        // The following 3 variables are terms of the quadratic formula. We use double the linear
        // term because it removes the annoying constants from that formula.
        let quadratic_term = mip_dir_a.powi(2) - mip_dir_b.powi(2) + c;
        let double_linear_term = mip_pos_a * mip_dir_a - mip_pos_b * mip_dir_b;
        let constant_term = mip_pos_a.powi(2) - mip_pos_b.powi(2) - c;

        let discriminant = double_linear_term * double_linear_term - quadratic_term * constant_term;

        // While discriminant can be negative, NaNs propagate the way we want to, so we don't have
        // to check for this.
        (-double_linear_term - discriminant.sqrt()) / quadratic_term
    }
}

impl ChunkRayTracer for SphereChunkRayTracer {
    fn trace_ray_in_chunk(
        &self,
        voxel_data: VoxelDataWrapper,
        pos: &na::Vector4<f64>,
        dir: &na::Vector4<f64>,
        handle: &mut RayTracingResultHandle,
    ) {
        for coord_axis in 0..3 {
            self.trace_ray_for_sides(voxel_data, pos, dir, handle, coord_axis);
        }

        //self.trace_ray_for_vertices(chunk_data, pos, dir, handle);
    }

    fn max_radius(&self) -> f64 {
        self.radius
    }
}
