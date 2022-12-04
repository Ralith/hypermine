use common::{dodeca::Vertex, math, world::Material};

use crate::chunk_ray_tracer::{ChunkRayTracer, RayTracingResultHandle, VoxelDataWrapper};

pub struct SphereChunkRayTracer {
    pub radius: f64,
}

impl ChunkRayTracer for SphereChunkRayTracer {
    fn trace_ray_in_chunk(
        &self,
        voxel_data: VoxelDataWrapper,
        pos: &na::Vector4<f64>,
        dir: &na::Vector4<f64>,
        handle: &mut RayTracingResultHandle,
    ) {
        SphereChunkRayTracingPass::new(self.radius, voxel_data, pos, dir, handle)
            .trace_ray_in_chunk();
    }

    fn max_radius(&self) -> f64 {
        self.radius
    }
}

struct SphereChunkRayTracingPass<'a, 'b> {
    radius: f64,
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

impl SphereChunkRayTracingPass<'_, '_> {
    fn new<'a, 'b>(
        radius: f64,
        voxel_data: VoxelDataWrapper<'a>,
        pos: &'a na::Vector4<f64>,
        dir: &'a na::Vector4<f64>,
        handle: &'a mut RayTracingResultHandle<'b>,
    ) -> SphereChunkRayTracingPass<'a, 'b> {
        let float_size = voxel_data.dimension() as f64;
        let voxel_start = (pos / pos[3]).xyz() * Vertex::dual_to_chunk_factor() * float_size;
        let end_pos = pos + dir * handle.t();
        let voxel_end = (end_pos / end_pos[3]).xyz() * Vertex::dual_to_chunk_factor() * float_size;
        let max_voxel_radius = radius * Vertex::dual_to_chunk_factor() * float_size;
        let bbox = [0, 1, 2].map(|coord| {
            get_usize_range(
                0,
                voxel_data.dimension(),
                voxel_start[coord],
                voxel_end[coord],
                max_voxel_radius,
            )
        });

        SphereChunkRayTracingPass {
            radius,
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

        for coord_axis in 0..3 {
            self.trace_ray_for_edges(coord_axis);
        }

        self.trace_ray_for_vertices();
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
            let t_candidate =
                find_intersection_one_vector(self.pos, self.dir, &normal, self.radius.sinh());

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
                if self.voxel_data.get(coords) != Material::Void {
                    self.handle.update(
                        t_candidate,
                        [0, 0, 0], /* TODO */
                        0,         /* TODO */
                        0,         /* TODO */
                        normal * -mip_dir_norm.signum(),
                    );
                }
            }
        }
    }

    fn trace_ray_for_edges(&mut self, coord_axis: usize) {
        let float_size = self.voxel_data.dimension() as f64;
        let coord_plane0 = (coord_axis + 1) % 3;
        let coord_plane1 = (coord_axis + 2) % 3;

        for i in self.bbox[coord_plane0][0]..=self.bbox[coord_plane0][1] {
            'main_loop: for j in self.bbox[coord_plane1][0]..=self.bbox[coord_plane1][1] {
                let mut edge_pos = na::Vector4::zeros();
                let mut edge_dir = na::Vector4::zeros();
                edge_pos[coord_plane0] = i as f64 / float_size * Vertex::chunk_to_dual_factor();
                edge_pos[coord_plane1] = j as f64 / float_size * Vertex::chunk_to_dual_factor();
                edge_pos[3] = 1.0;
                edge_pos = math::lorentz_normalize(&edge_pos);
                edge_dir[coord_axis] = 1.0;

                let t_candidate = find_intersection_two_vectors(
                    self.pos,
                    self.dir,
                    &edge_pos,
                    &edge_dir,
                    self.radius.cosh(),
                );

                // If t_candidate is out of range or NaN, don't continue collision checking
                if !(t_candidate >= 0.0 && t_candidate < self.handle.t()) {
                    continue;
                }

                let translated_square_pos = self.pos + self.dir * t_candidate;
                let projected_pos = -edge_pos * math::mip(&translated_square_pos, &edge_pos)
                    + edge_dir * math::mip(&translated_square_pos, &edge_dir);
                let projected_pos = projected_pos / projected_pos.w;
                let k = (projected_pos[coord_axis] * Vertex::dual_to_chunk_factor() * float_size)
                    .floor();

                if k >= 0.0 && k < float_size {
                    let k = k as usize;

                    for i_with_offset in
                        (i.saturating_sub(1))..=(i.min(self.voxel_data.dimension() - 1))
                    {
                        for j_with_offset in
                            (j.saturating_sub(1))..=(j.min(self.voxel_data.dimension() - 1))
                        {
                            let mut coords = [0; 3];
                            coords[coord_axis] = k;
                            coords[coord_plane0] = i_with_offset;
                            coords[coord_plane1] = j_with_offset;
                            if self.voxel_data.get(coords) != Material::Void {
                                self.handle.update(
                                    t_candidate,
                                    [0, 0, 0], /* TODO */
                                    0,         /* TODO */
                                    0,         /* TODO */
                                    translated_square_pos - projected_pos,
                                );
                                continue 'main_loop;
                            }
                        }
                    }
                }
            }
        }
    }

    #[rustfmt::skip]
    fn trace_ray_for_vertices(&mut self) {
        let size = self.voxel_data.dimension();
        let float_size = size as f64;

        for i in self.bbox[0][0]..=self.bbox[0][1] {
            for j in self.bbox[1][0]..=self.bbox[1][1] {
                for k in self.bbox[2][0]..=self.bbox[2][1] {
                    if (i == 0 || j == 0 || k == 0 || self.voxel_data.get([i - 1, j - 1, k - 1]) == Material::Void)
                        && (i == size || j == 0 || k == 0 || self.voxel_data.get([i, j - 1, k - 1]) == Material::Void)
                        && (i == 0 || j == size || k == 0 || self.voxel_data.get([i - 1, j, k - 1]) == Material::Void)
                        && (i == size || j == size || k == 0 || self.voxel_data.get([i, j, k - 1]) == Material::Void)
                        && (i == 0 || j == 0 || k == size || self.voxel_data.get([i - 1, j - 1, k]) == Material::Void)
                        && (i == size || j == 0 || k == size || self.voxel_data.get([i, j - 1, k]) == Material::Void)
                        && (i == 0 || j == size || k == size || self.voxel_data.get([i - 1, j, k]) == Material::Void)
                        && (i == size || j == size || k == size || self.voxel_data.get([i, j, k]) == Material::Void)
                    {
                        continue;
                    }

                    let vert = math::lorentz_normalize(&na::Vector4::new(
                        i as f64 / float_size * Vertex::chunk_to_dual_factor(),
                        j as f64 / float_size * Vertex::chunk_to_dual_factor(),
                        k as f64 / float_size * Vertex::chunk_to_dual_factor(),
                        1.0,
                    ));

                    let t_candidate =
                        find_intersection_one_vector(self.pos, self.dir, &vert, self.radius.cosh());

                    // If t_candidate is out of range or NaN, don't continue collision checking
                    if !(t_candidate >= 0.0 && t_candidate < self.handle.t()) {
                        continue;
                    }

                    let translated_square_pos = self.pos + self.dir * t_candidate;
                    self.handle.update(
                        t_candidate,
                        [0, 0, 0], /* TODO */
                        0, /* TODO */
                        0, /* TODO */
                        translated_square_pos - vert,
                    );
                }
            }
        }
    }
}

/// Find the smallest value of `t` where the point in the pos-dir line (v=pos+dir*t) satisfies
/// `<v,a>^2 / <v,v> == c^2`
///
/// If `a` is direction-like, this finds intersections with a surface that is `sinh(c)` units
/// away from the plane whose normal is `a`.  If `a` is point-like, this finds intersections
/// with a sphere centered at `a` with radius `cosh(c)`.
///
/// Returns NaN if there's no such intersection
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
/// `(<v,a>^2 - <v,b>^2) / <v,v> == c^2`
///
/// This finds intersections with a surface that is `cosh(c)` units away from the line
/// with a point at `a` with direction `b`, where `<a,b>==0`.
///
/// Returns NaN if there's no such intersection
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
    let quadratic_term = mip_dir_a.powi(2) - mip_dir_b.powi(2) + c.powi(2);
    let double_linear_term = mip_pos_a * mip_dir_a - mip_pos_b * mip_dir_b;
    let constant_term = mip_pos_a.powi(2) - mip_pos_b.powi(2) - c.powi(2);

    let discriminant = double_linear_term * double_linear_term - quadratic_term * constant_term;

    // While discriminant can be negative, NaNs propagate the way we want to, so we don't have
    // to check for this.
    (-double_linear_term - discriminant.sqrt()) / quadratic_term
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
