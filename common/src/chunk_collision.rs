use crate::{
    graph_collision::Ray,
    math,
    node::{ChunkLayout, VoxelData},
    world::Material,
};

pub struct ChunkCastHit {
    /// The tanh of the distance traveled along the ray to result in this hit.
    pub tanh_distance: f32,

    /// Represents the normal vector of the hit surface in the dual coordinate system of the chunk.
    /// To get the actual normal vector, project it so that it is orthogonal to the endpoint in Lorentz space.
    pub normal: na::Vector4<f32>,
}

/// Performs sphere casting (swept collision query) against the voxels in the chunk with the given `voxel_data`
///
/// The `ray` parameter is given and any resulting hit normals are given in the chunk's dual coordinate system.
///
/// The `tanh_distance` is the hyperbolic tangent of the distance along the ray to check for hits.
pub fn chunk_sphere_cast(
    collider_radius: f32,
    voxel_data: &VoxelData,
    layout: &ChunkLayout,
    ray: &Ray,
    tanh_distance: f32,
) -> Option<ChunkCastHit> {
    let mut hit: Option<ChunkCastHit> = None;

    let Some(bounding_box) = VoxelAABB::from_ray_segment_and_radius(
        layout,
        ray,
        tanh_distance,
        collider_radius,
    ) else {
        return None;
    };

    for t_axis in 0..3 {
        hit = find_face_collision(
            collider_radius,
            voxel_data,
            layout,
            &bounding_box,
            t_axis,
            ray,
            hit.as_ref().map_or(tanh_distance, |hit| hit.tanh_distance),
        )
        .or(hit);
    }

    for t_axis in 0..3 {
        hit = find_edge_collision(
            collider_radius,
            voxel_data,
            layout,
            &bounding_box,
            t_axis,
            ray,
            hit.as_ref().map_or(tanh_distance, |hit| hit.tanh_distance),
        )
        .or(hit);
    }

    hit = find_vertex_collision(
        collider_radius,
        voxel_data,
        layout,
        &bounding_box,
        ray,
        hit.as_ref().map_or(tanh_distance, |hit| hit.tanh_distance),
    )
    .or(hit);

    hit
}

/// Detect collisions where a sphere contacts the front side of a voxel face
fn find_face_collision(
    collider_radius: f32,
    voxel_data: &VoxelData,
    layout: &ChunkLayout,
    bounding_box: &VoxelAABB,
    t_axis: usize,
    ray: &Ray,
    tanh_distance: f32,
) -> Option<ChunkCastHit> {
    let mut hit: Option<ChunkCastHit> = None;

    let u_axis = (t_axis + 1) % 3;
    let v_axis = (t_axis + 2) % 3;

    // Loop through all grid planes overlapping the bounding box
    for t in bounding_box.grid_planes(t_axis) {
        // Find a normal to the grid plane. Note that (t, 0, 0, x) is a normal of the plane whose closest point
        // to the origin is (x, 0, 0, t), and we use that fact here.
        let normal = math::lorentz_normalize(&tuv_to_xyz(
            t_axis,
            na::Vector4::new(1.0, 0.0, 0.0, layout.grid_to_dual(t)),
        ));

        let Some(new_tanh_distance) =
            solve_sphere_plane_intersection(ray, &normal, collider_radius.sinh()) else {
                continue;
            };

        // If new_tanh_distance is out of range, no collision occurred.
        if new_tanh_distance >= hit.as_ref().map_or(tanh_distance, |hit| hit.tanh_distance) {
            continue;
        }

        // Whether we are approaching the front or back of the face. An approach from the positive t direction
        // is 1, and an approach from the negative t direction is -1.
        let collision_side = -math::mip(&ray.direction, &normal).signum();

        // Which side we approach the plane from affects which voxel we want to use for collision checking
        let voxel_t = if collision_side > 0.0 { t } else { t + 1 };

        let ray_endpoint = ray.ray_point(new_tanh_distance);
        let contact_point = ray_endpoint - normal * math::mip(&ray_endpoint, &normal);

        // Compute the u and v-coordinates of the voxels at the contact point
        let Some(voxel_u) = layout.dual_to_voxel(contact_point[u_axis] / contact_point.w) else {
            continue;
        };
        let Some(voxel_v) = layout.dual_to_voxel(contact_point[v_axis] / contact_point.w) else {
            continue;
        };

        // Ensure that the relevant voxel is solid
        if !voxel_is_solid(
            voxel_data,
            layout,
            tuv_to_xyz(t_axis, [voxel_t, voxel_u, voxel_v]),
        ) {
            continue;
        }

        // A collision was found. Update the hit.
        hit = Some(ChunkCastHit {
            tanh_distance: new_tanh_distance,
            normal: normal * collision_side,
        });
    }

    hit
}

/// Detect collisions where a sphere contacts a voxel edge
fn find_edge_collision(
    collider_radius: f32,
    voxel_data: &VoxelData,
    layout: &ChunkLayout,
    bounding_box: &VoxelAABB,
    t_axis: usize,
    ray: &Ray,
    tanh_distance: f32,
) -> Option<ChunkCastHit> {
    let mut hit: Option<ChunkCastHit> = None;

    let u_axis = (t_axis + 1) % 3;
    let v_axis = (t_axis + 2) % 3;

    // Loop through all grid lines overlapping the bounding box
    for (u, v) in bounding_box.grid_lines(u_axis, v_axis) {
        // Compute vectors Lorentz-orthogonal to the edge and to each other
        let edge_normal0 = math::lorentz_normalize(&tuv_to_xyz(
            t_axis,
            na::Vector4::new(0.0, 1.0, 0.0, layout.grid_to_dual(u)),
        ));

        let edge_normal1 = tuv_to_xyz(
            t_axis,
            na::Vector4::new(0.0, 0.0, 1.0, layout.grid_to_dual(v)),
        );
        let edge_normal1 = math::lorentz_normalize(
            &(edge_normal1 - edge_normal0 * math::mip(&edge_normal0, &edge_normal1)),
        );

        let Some(new_tanh_distance) = solve_sphere_line_intersection(
            ray,
            &edge_normal0,
            &edge_normal1,
            collider_radius.sinh(),
        ) else {
            continue;
        };

        // If new_tanh_distance is out of range, no collision occurred.
        if new_tanh_distance >= hit.as_ref().map_or(tanh_distance, |hit| hit.tanh_distance) {
            continue;
        }

        let ray_endpoint = ray.ray_point(new_tanh_distance);
        let contact_point = ray_endpoint
            - edge_normal0 * math::mip(&ray_endpoint, &edge_normal0)
            - edge_normal1 * math::mip(&ray_endpoint, &edge_normal1);

        // Compute the t-coordinate of the voxels at the contact point
        let Some(voxel_t) = layout.dual_to_voxel(contact_point[t_axis] / contact_point.w) else {
            continue;
        };

        // Ensure that the edge has a solid voxel adjacent to it
        if (0..2).all(|du| {
            (0..2).all(|dv| {
                !voxel_is_solid(
                    voxel_data,
                    layout,
                    tuv_to_xyz(t_axis, [voxel_t, u + du, v + dv]),
                )
            })
        }) {
            continue;
        }

        // A collision was found. Update the hit.
        hit = Some(ChunkCastHit {
            tanh_distance: new_tanh_distance,
            normal: ray_endpoint - contact_point,
        });
    }

    hit
}

/// Detect collisions where a sphere contacts a voxel vertex
fn find_vertex_collision(
    collider_radius: f32,
    voxel_data: &VoxelData,
    layout: &ChunkLayout,
    bounding_box: &VoxelAABB,
    ray: &Ray,
    tanh_distance: f32,
) -> Option<ChunkCastHit> {
    let mut hit: Option<ChunkCastHit> = None;

    // Loop through all grid points contained in the bounding box
    for (x, y, z) in bounding_box.grid_points(0, 1, 2) {
        // Skip vertices that have no solid voxels adjacent to them
        if (0..2).all(|dx| {
            (0..2).all(|dy| {
                (0..2).all(|dz| !voxel_is_solid(voxel_data, layout, [x + dx, y + dy, z + dz]))
            })
        }) {
            continue;
        }

        // Compute vectors Lorentz-orthogonal to the vertex and to each other
        let vertex_normal0 =
            math::lorentz_normalize(&na::Vector4::new(1.0, 0.0, 0.0, layout.grid_to_dual(x)));

        let vertex_normal1 = na::Vector4::new(0.0, 1.0, 0.0, layout.grid_to_dual(y));
        let vertex_normal1 = math::lorentz_normalize(
            &(vertex_normal1 - vertex_normal0 * math::mip(&vertex_normal0, &vertex_normal1)),
        );

        let vertex_normal2 = na::Vector4::new(0.0, 0.0, 1.0, layout.grid_to_dual(z));
        let vertex_normal2 = math::lorentz_normalize(
            &(vertex_normal2
                - vertex_normal0 * math::mip(&vertex_normal0, &vertex_normal2)
                - vertex_normal1 * math::mip(&vertex_normal1, &vertex_normal2)),
        );

        let Some(new_tanh_distance) =
            solve_sphere_point_intersection(ray, &vertex_normal0, &vertex_normal1, &vertex_normal2, collider_radius.sinh()) else {
                continue;
            };

        // If new_tanh_distance is out of range, no collision occurred.
        if new_tanh_distance >= hit.as_ref().map_or(tanh_distance, |hit| hit.tanh_distance) {
            continue;
        }

        // Determine the cube-centric coordinates of the vertex
        let vertex_position = math::lorentz_normalize(&na::Vector4::new(
            layout.grid_to_dual(x),
            layout.grid_to_dual(y),
            layout.grid_to_dual(z),
            1.0,
        ));

        // A collision was found. Update the hit.
        let ray_endpoint = ray.ray_point(new_tanh_distance);
        hit = Some(ChunkCastHit {
            tanh_distance: new_tanh_distance,
            normal: ray_endpoint - vertex_position,
        });
    }

    hit
}

/// Finds the tanh of the distance a sphere will have to travel along a ray before it
/// intersects the given plane.
fn solve_sphere_plane_intersection(
    ray: &Ray,
    plane_normal: &na::Vector4<f32>,
    sinh_radius: f32,
) -> Option<f32> {
    let mip_pos_a = math::mip(&ray.position, plane_normal);
    let mip_dir_a = math::mip(&ray.direction, plane_normal);

    solve_quadratic(
        mip_pos_a.powi(2) - sinh_radius.powi(2),
        mip_pos_a * mip_dir_a,
        mip_dir_a.powi(2) + sinh_radius.powi(2),
    )
}

/// Finds the tanh of the distance a sphere will have to travel along a ray before it
/// intersects the given line.
fn solve_sphere_line_intersection(
    ray: &Ray,
    line_normal0: &na::Vector4<f32>,
    line_normal1: &na::Vector4<f32>,
    sinh_radius: f32,
) -> Option<f32> {
    let mip_pos_a = math::mip(&ray.position, line_normal0);
    let mip_dir_a = math::mip(&ray.direction, line_normal0);
    let mip_pos_b = math::mip(&ray.position, line_normal1);
    let mip_dir_b = math::mip(&ray.direction, line_normal1);

    solve_quadratic(
        mip_pos_a.powi(2) + mip_pos_b.powi(2) - sinh_radius.powi(2),
        mip_pos_a * mip_dir_a + mip_pos_b * mip_dir_b,
        mip_dir_a.powi(2) + mip_dir_b.powi(2) + sinh_radius.powi(2),
    )
}

/// Finds the tanh of the distance a sphere will have to travel along a ray before it
/// intersects the given point.
fn solve_sphere_point_intersection(
    ray: &Ray,
    point_normal0: &na::Vector4<f32>,
    point_normal1: &na::Vector4<f32>,
    point_normal2: &na::Vector4<f32>,
    sinh_radius: f32,
) -> Option<f32> {
    let mip_pos_a = math::mip(&ray.position, point_normal0);
    let mip_dir_a = math::mip(&ray.direction, point_normal0);
    let mip_pos_b = math::mip(&ray.position, point_normal1);
    let mip_dir_b = math::mip(&ray.direction, point_normal1);
    let mip_pos_c = math::mip(&ray.position, point_normal2);
    let mip_dir_c = math::mip(&ray.direction, point_normal2);

    solve_quadratic(
        mip_pos_a.powi(2) + mip_pos_b.powi(2) + mip_pos_c.powi(2) - sinh_radius.powi(2),
        mip_pos_a * mip_dir_a + mip_pos_b * mip_dir_b + mip_pos_c * mip_dir_c,
        mip_dir_a.powi(2) + mip_dir_b.powi(2) + mip_dir_c.powi(2) + sinh_radius.powi(2),
    )
}

/// Finds the lower solution `x` of `constant_term + 2 * half_linear_term * x + quadratic_term * x * x == 0`
/// if such a solution exists and is non-negative. Assumes that `quadratic_term` is positive. Double-roots are
/// ignored.
///
/// If the lower solution is negative, but a small perturbation to the constant term would make it 0, this function
/// returns 0.
fn solve_quadratic(constant_term: f32, half_linear_term: f32, quadratic_term: f32) -> Option<f32> {
    const EPSILON: f32 = 1e-4;

    // If the linear term is positive, the lower solution is negative, and we're not interested. If the
    // linear term is zero, the solution can only be non-negative if the constant term is also zero,
    // which results in a double-root, which we also ignore.
    if half_linear_term >= 0.0 {
        return None;
    }

    // If the constant term is negative, the lower solution must also be negative. To avoid precision issues
    // allowing a collider to clip through a surface, we treat small negative constant terms as zero, which
    // results in a lower solution of zero.
    if constant_term <= 0.0 {
        return if constant_term > -EPSILON {
            Some(0.0)
        } else {
            None
        };
    }

    let discriminant = half_linear_term * half_linear_term - quadratic_term * constant_term;
    if discriminant <= 0.0 {
        return None;
    }

    // We use an alternative quadratic formula to ensure that we return a positive number if `constant_term > 0.0`.
    // Otherwise, the edge case of a small positive `constant_term` could be mishandled.
    // The denominator cannot be zero because both of its terms are positive.
    Some(constant_term / (-half_linear_term + discriminant.sqrt()))
}

/// Converts from t-u-v coordinates to x-y-z coordinates. t-u-v coordinates are a permuted version of x-y-z coordinates.
/// `t_axis` determines which of the three x-y-z coordinates corresponds to the t-coordinate. This function works with
/// any indexable entity with at least three entries. Any entry after the third entry is ignored.
fn tuv_to_xyz<T: std::ops::IndexMut<usize, Output = N>, N: Copy>(t_axis: usize, tuv: T) -> T {
    let mut result = tuv;
    (
        result[t_axis],
        result[(t_axis + 1) % 3],
        result[(t_axis + 2) % 3],
    ) = (result[0], result[1], result[2]);
    result
}

/// Checks whether a voxel can be collided with. Any non-void voxel falls under this category.
fn voxel_is_solid(voxel_data: &VoxelData, layout: &ChunkLayout, coords: [usize; 3]) -> bool {
    let dimension_with_margin = layout.dimension() + 2;
    debug_assert!(coords[0] < dimension_with_margin);
    debug_assert!(coords[1] < dimension_with_margin);
    debug_assert!(coords[2] < dimension_with_margin);
    voxel_data.get(
        coords[0] + coords[1] * dimension_with_margin + coords[2] * dimension_with_margin.pow(2),
    ) != Material::Void
}

/// Represents a discretized region in the voxel grid contained by an axis-aligned bounding box.
struct VoxelAABB {
    // The bounds are of the form [[x_min, x_max], [y_min, y_max], [z_min, z_max]], using voxel coordinates with margins.
    // Any voxel that intersects the cube of interest is included in these bounds. By adding or subtracting 1 in the right
    // places, these bounds can be used to find other useful info related to the cube of interset, such as what grid points
    // it contains.
    bounds: [[usize; 2]; 3],
}

impl VoxelAABB {
    /// Returns a bounding box that is guaranteed to cover a given radius around a ray segment. Returns None if the
    /// bounding box lies entirely outside the chunk, including its margins.
    pub fn from_ray_segment_and_radius(
        layout: &ChunkLayout,
        ray: &Ray,
        tanh_distance: f32,
        radius: f32,
    ) -> Option<VoxelAABB> {
        // Convert the ray to grid coordinates
        let grid_start =
            na::Point3::from_homogeneous(ray.position).unwrap() * layout.dual_to_grid_factor();
        let grid_end = na::Point3::from_homogeneous(ray.ray_point(tanh_distance)).unwrap()
            * layout.dual_to_grid_factor();
        // Convert the radius to grid coordinates using a crude conservative estimate
        let max_grid_radius = radius * layout.dual_to_grid_factor();
        let mut bounds = [[0; 2]; 3];
        for axis in 0..3 {
            let grid_min = grid_start[axis].min(grid_end[axis]) - max_grid_radius;
            let grid_max = grid_start[axis].max(grid_end[axis]) + max_grid_radius;
            let voxel_min = (grid_min + 1.0).floor().max(0.0);
            let voxel_max = (grid_max + 1.0)
                .floor()
                .min(layout.dimension() as f32 + 1.0);

            // This will happen when voxel_min is greater than dimension+1 or voxel_max is less than 0, which
            // occurs when the cube is out of range.
            if voxel_min > voxel_max {
                return None;
            }

            // We convert to usize here instead of earlier because out-of-range voxel coordinates can be negative.
            bounds[axis] = [voxel_min.floor() as usize, voxel_max.floor() as usize];
        }

        Some(VoxelAABB { bounds })
    }

    /// Iterator over grid points contained in the region, represented as ordered triples
    pub fn grid_points(
        &self,
        axis0: usize,
        axis1: usize,
        axis2: usize,
    ) -> impl Iterator<Item = (usize, usize, usize)> {
        let bounds = self.bounds;
        (bounds[axis0][0]..bounds[axis0][1]).flat_map(move |i| {
            (bounds[axis1][0]..bounds[axis1][1])
                .flat_map(move |j| (bounds[axis2][0]..bounds[axis2][1]).map(move |k| (i, j, k)))
        })
    }

    /// Iterator over grid lines intersecting the region, represented as ordered pairs determining the line's two fixed coordinates
    pub fn grid_lines(&self, axis0: usize, axis1: usize) -> impl Iterator<Item = (usize, usize)> {
        let bounds = self.bounds;
        (bounds[axis0][0]..bounds[axis0][1])
            .flat_map(move |i| (bounds[axis1][0]..bounds[axis1][1]).map(move |j| (i, j)))
    }

    /// Iterator over grid planes intersecting the region, represented as integers determining the plane's fixed coordinate
    pub fn grid_planes(&self, axis: usize) -> impl Iterator<Item = usize> {
        self.bounds[axis][0]..self.bounds[axis][1]
    }
}

#[cfg(test)]
mod tests {
    use std::collections::HashSet;

    use crate::node::VoxelData;

    use super::*;
    use approx::*;

    /// Helper structure used to reduce the number of parameters to pass around with tests
    struct TestSphereCastContext {
        collider_radius: f32,
        layout: ChunkLayout,
        voxel_data: VoxelData,
    }

    impl TestSphereCastContext {
        fn new(collider_radius: f32) -> Self {
            let dimension: usize = 12;

            let mut ctx = TestSphereCastContext {
                collider_radius,
                layout: ChunkLayout::new(dimension),
                voxel_data: VoxelData::Solid(Material::Void),
            };

            // Populate voxels. Consists of a single voxel with voxel coordinates (2, 2, 2). The cube corresponding
            // to this voxel has grid coordinates from (1, 1, 1) to (2, 2, 2)
            ctx.set_voxel([2, 2, 2], Material::Dirt);

            ctx
        }

        fn set_voxel(&mut self, coords: [usize; 3], material: Material) {
            let dimension_with_margin = self.layout.dimension() + 2;
            debug_assert!(coords[0] < dimension_with_margin);
            debug_assert!(coords[1] < dimension_with_margin);
            debug_assert!(coords[2] < dimension_with_margin);
            self.voxel_data.data_mut(self.layout.dimension() as u8)[coords[0]
                + coords[1] * dimension_with_margin
                + coords[2] * dimension_with_margin.pow(2)] = material;
        }
    }

    /// Helper method to create a `ChunkSphereCastContext` that can be used
    /// in a closure to call sphere casting methods.
    fn cast_with_test_ray(
        ctx: &TestSphereCastContext,
        ray_start_grid_coords: [f32; 3],
        ray_end_grid_coords: [f32; 3],
        wrapped_fn: impl FnOnce(&Ray, f32),
    ) {
        let ray_start = math::lorentz_normalize(&na::Vector4::new(
            ray_start_grid_coords[0] / ctx.layout.dual_to_grid_factor(),
            ray_start_grid_coords[1] / ctx.layout.dual_to_grid_factor(),
            ray_start_grid_coords[2] / ctx.layout.dual_to_grid_factor(),
            1.0,
        ));

        let ray_end = math::lorentz_normalize(&na::Vector4::new(
            ray_end_grid_coords[0] / ctx.layout.dual_to_grid_factor(),
            ray_end_grid_coords[1] / ctx.layout.dual_to_grid_factor(),
            ray_end_grid_coords[2] / ctx.layout.dual_to_grid_factor(),
            1.0,
        ));

        let ray = Ray::new(
            ray_start,
            math::lorentz_normalize(
                &((ray_end - ray_start)
                    + ray_start * math::mip(&ray_start, &(ray_end - ray_start))),
            ),
        );

        let tanh_distance = (-math::mip(&ray_start, &ray_end)).acosh();

        wrapped_fn(&ray, tanh_distance)
    }

    fn chunk_sphere_cast_wrapper(
        ctx: &TestSphereCastContext,
        ray: &Ray,
        tanh_distance: f32,
    ) -> Option<ChunkCastHit> {
        chunk_sphere_cast(
            ctx.collider_radius,
            &ctx.voxel_data,
            &ctx.layout,
            ray,
            tanh_distance,
        )
    }

    fn find_face_collision_wrapper(
        ctx: &TestSphereCastContext,
        ray: &Ray,
        t_axis: usize,
        tanh_distance: f32,
    ) -> Option<ChunkCastHit> {
        find_face_collision(
            ctx.collider_radius,
            &ctx.voxel_data,
            &ctx.layout,
            &VoxelAABB::from_ray_segment_and_radius(
                &ctx.layout,
                ray,
                tanh_distance,
                ctx.collider_radius,
            )
            .unwrap(),
            t_axis,
            ray,
            tanh_distance,
        )
    }

    fn find_edge_collision_wrapper(
        ctx: &TestSphereCastContext,
        ray: &Ray,
        t_axis: usize,
        tanh_distance: f32,
    ) -> Option<ChunkCastHit> {
        find_edge_collision(
            ctx.collider_radius,
            &ctx.voxel_data,
            &ctx.layout,
            &VoxelAABB::from_ray_segment_and_radius(
                &ctx.layout,
                ray,
                tanh_distance,
                ctx.collider_radius,
            )
            .unwrap(),
            t_axis,
            ray,
            tanh_distance,
        )
    }

    fn find_vertex_collision_wrapper(
        ctx: &TestSphereCastContext,
        ray: &Ray,
        tanh_distance: f32,
    ) -> Option<ChunkCastHit> {
        find_vertex_collision(
            ctx.collider_radius,
            &ctx.voxel_data,
            &ctx.layout,
            &VoxelAABB::from_ray_segment_and_radius(
                &ctx.layout,
                ray,
                tanh_distance,
                ctx.collider_radius,
            )
            .unwrap(),
            ray,
            tanh_distance,
        )
    }

    fn test_face_collision(
        ctx: &TestSphereCastContext,
        ray: &Ray,
        t_axis: usize,
        tanh_distance: f32,
    ) {
        let hit = chunk_sphere_cast_wrapper(ctx, ray, tanh_distance);
        assert_hits_exist_and_eq(
            &hit,
            &find_face_collision_wrapper(ctx, ray, t_axis, tanh_distance),
        );
        sanity_check_normal(ray, &hit.unwrap());
    }

    fn test_edge_collision(
        ctx: &TestSphereCastContext,
        ray: &Ray,
        t_axis: usize,
        tanh_distance: f32,
    ) {
        let hit = chunk_sphere_cast_wrapper(ctx, ray, tanh_distance);
        assert_hits_exist_and_eq(
            &hit,
            &find_edge_collision_wrapper(ctx, ray, t_axis, tanh_distance),
        );
        sanity_check_normal(ray, &hit.unwrap());
    }

    fn test_vertex_collision(ctx: &TestSphereCastContext, ray: &Ray, tanh_distance: f32) {
        let hit = chunk_sphere_cast_wrapper(ctx, ray, tanh_distance);
        assert_hits_exist_and_eq(
            &hit,
            &find_vertex_collision_wrapper(ctx, ray, tanh_distance),
        );
        sanity_check_normal(ray, &hit.unwrap());
    }

    /// Check that the two hits exist and are equal to each other. Useful for ensuring that
    /// a particular intersection type is detected by the general `chunk_sphere_cast` method.
    fn assert_hits_exist_and_eq(hit0: &Option<ChunkCastHit>, hit1: &Option<ChunkCastHit>) {
        assert!(hit0.is_some());
        assert!(hit1.is_some());
        assert_eq!(
            hit0.as_ref().unwrap().tanh_distance,
            hit1.as_ref().unwrap().tanh_distance
        );
        assert_eq!(hit0.as_ref().unwrap().normal, hit1.as_ref().unwrap().normal);
    }

    /// Ensures that the normal is pointing outward, opposite the ray direction.
    fn sanity_check_normal(ray: &Ray, hit: &ChunkCastHit) {
        // The ray we care about is after its start point has moved to the contact point.
        let ray = math::translate(
            &ray.position,
            &math::lorentz_normalize(&ray.ray_point(hit.tanh_distance)),
        ) * ray;

        // Project normal to be perpendicular to the ray's position
        let corrected_normal = math::lorentz_normalize(
            &(hit.normal + ray.position * math::mip(&hit.normal, &ray.position)),
        );

        // Check that the normal and ray are pointing opposite directions
        assert!(math::mip(&corrected_normal, &ray.direction) < 0.0);
    }

    /// Tests that a suitable collision is found when approaching a single voxel from various angles and that
    /// no collision is found in paths that don't reach that voxel.
    #[test]
    fn chunk_sphere_cast_examples() {
        let collider_radius = 0.02;
        let ctx = TestSphereCastContext::new(collider_radius);

        // Approach a single voxel from various angles. Ensure that a suitable collision is found each time.
        // Note: The voxel is centered at (1.5, 1.5, 1.5) in the grid coordinates used in this test.

        // Face collisions
        cast_with_test_ray(
            &ctx,
            [0.0, 1.5, 1.5],
            [1.5, 1.5, 1.5],
            |ray, tanh_distance| {
                test_face_collision(&ctx, ray, 0, tanh_distance);
            },
        );

        cast_with_test_ray(
            &ctx,
            [1.5, 1.5, 3.0],
            [1.5, 1.5, 1.5],
            |ray, tanh_distance| {
                test_face_collision(&ctx, ray, 2, tanh_distance);
            },
        );

        // Edge collisions
        cast_with_test_ray(
            &ctx,
            [1.5, 3.0, 0.0],
            [1.5, 1.5, 1.5],
            |ray, tanh_distance| {
                test_edge_collision(&ctx, ray, 0, tanh_distance);
            },
        );

        cast_with_test_ray(
            &ctx,
            [3.0, 1.5, 3.0],
            [1.5, 1.5, 1.5],
            |ray, tanh_distance| {
                test_edge_collision(&ctx, ray, 1, tanh_distance);
            },
        );

        // Vertex collisions
        cast_with_test_ray(
            &ctx,
            [0.0, 0.0, 0.0],
            [1.5, 1.5, 1.5],
            |ray, tanh_distance| {
                test_vertex_collision(&ctx, ray, tanh_distance);
            },
        );

        cast_with_test_ray(
            &ctx,
            [3.0, 3.0, 0.0],
            [1.5, 1.5, 1.5],
            |ray, tanh_distance| {
                test_vertex_collision(&ctx, ray, tanh_distance);
            },
        );

        // No collision: Going sideways relative to a face
        cast_with_test_ray(
            &ctx,
            [3.0, 1.5, 1.5],
            [3.0, 3.0, 1.5],
            |ray, tanh_distance| {
                assert!(chunk_sphere_cast_wrapper(&ctx, ray, tanh_distance).is_none());
            },
        );

        // No collision: Going away from a face
        cast_with_test_ray(
            &ctx,
            [3.0, 1.5, 1.5],
            [4.5, 1.5, 1.5],
            |ray, tanh_distance| {
                assert!(chunk_sphere_cast_wrapper(&ctx, ray, tanh_distance).is_none());
            },
        );

        // No collision: Past cast endpoint
        cast_with_test_ray(
            &ctx,
            [8.0, 1.5, 1.5],
            [3.0, 1.5, 1.5],
            |ray, tanh_distance| {
                assert!(chunk_sphere_cast_wrapper(&ctx, ray, tanh_distance).is_none());
            },
        );
    }

    /// Tests that colliding with a face from the back side is impossible. Note that colliding
    /// with the back side of an edge or vertex is still possible. Getting rid of these collisions
    /// is a possible future enhancement.
    #[test]
    fn face_collisions_one_sided() {
        let collider_radius = 0.01;
        let ctx = TestSphereCastContext::new(collider_radius);

        cast_with_test_ray(
            &ctx,
            [1.5, 1.5, 1.5],
            [4.5, 1.5, 1.5],
            |ray, tanh_distance| {
                assert!(chunk_sphere_cast_wrapper(&ctx, ray, tanh_distance).is_none());
            },
        )
    }

    #[test]
    fn solve_sphere_plane_intersection_example() {
        // Hit the z=0 plane with a radius of 0.2
        let ray = math::translate_along(&na::Vector3::new(0.0, 0.0, -0.5))
            * &Ray::new(math::origin(), na::Vector4::new(0.8, 0.0, 0.6, 0.0));
        let normal = -na::Vector4::z();
        let hit_point = math::lorentz_normalize(
            &ray.ray_point(solve_sphere_plane_intersection(&ray, &normal, 0.2_f32.sinh()).unwrap()),
        );
        assert_abs_diff_eq!(
            math::mip(&hit_point, &normal),
            0.2_f32.sinh(),
            epsilon = 1e-4
        );
    }

    #[test]
    fn solve_sphere_plane_intersection_direct_hit() {
        // Directly hit the z=0 plane with a ray 0.5 units away and a radius of 0.2.
        let ray = math::translate_along(&na::Vector3::new(0.0, 0.0, -0.5))
            * &Ray::new(math::origin(), na::Vector4::z());
        let normal = -na::Vector4::z();
        assert_abs_diff_eq!(
            solve_sphere_plane_intersection(&ray, &normal, 0.2_f32.sinh()).unwrap(),
            0.3_f32.tanh(),
            epsilon = 1e-4
        );
    }

    #[test]
    fn solve_sphere_plane_intersection_miss() {
        // No collision with the plane anywhere along the ray's line
        let ray = math::translate_along(&na::Vector3::new(0.0, 0.0, -0.5))
            * &Ray::new(math::origin(), na::Vector4::x());
        let normal = -na::Vector4::z();
        assert!(solve_sphere_plane_intersection(&ray, &normal, 0.2_f32.sinh()).is_none());
    }

    #[test]
    fn solve_sphere_plane_intersection_margin() {
        // Sphere is already contacting the plane, with some error
        let ray = math::translate_along(&na::Vector3::new(0.0, 0.0, -0.2))
            * &Ray::new(math::origin(), na::Vector4::z());
        let normal = -na::Vector4::z();
        assert_eq!(
            solve_sphere_plane_intersection(&ray, &normal, 0.2001_f32.sinh()).unwrap(),
            0.0
        );
    }

    #[test]
    fn solve_sphere_line_intersection_example() {
        // Hit the x=z=0 line with a radius of 0.2
        let ray = math::translate_along(&na::Vector3::new(0.0, 0.0, -0.5))
            * &Ray::new(
                math::origin(),
                na::Vector4::new(1.0, 2.0, 3.0, 0.0).normalize(),
            );
        let line_normal0 = na::Vector4::x();
        let line_normal1 = na::Vector4::z();
        let hit_point = math::lorentz_normalize(
            &ray.ray_point(
                solve_sphere_line_intersection(&ray, &line_normal0, &line_normal1, 0.2_f32.sinh())
                    .unwrap(),
            ),
        );
        // Measue the distance from hit_point to the line and ensure it's equal to the radius
        assert_abs_diff_eq!(
            (math::mip(&hit_point, &line_normal0).powi(2)
                + math::mip(&hit_point, &line_normal1).powi(2))
            .sqrt(),
            0.2_f32.sinh(),
            epsilon = 1e-4
        );
    }

    #[test]
    fn solve_sphere_line_intersection_direct_hit() {
        // Directly hit the x=z=0 line with a ray 0.5 units away and a radius of 0.2.

        // Ensure the ray is slightly off-center so that the distance math is shown to be correct
        let ray = math::translate_along(&na::Vector3::new(0.0, 0.7, 0.0))
            * math::translate_along(&na::Vector3::new(0.0, 0.0, -0.5))
            * &Ray::new(math::origin(), na::Vector4::z());
        let line_normal0 = na::Vector4::x();
        let line_normal1 = na::Vector4::z();
        assert_abs_diff_eq!(
            solve_sphere_line_intersection(&ray, &line_normal0, &line_normal1, 0.2_f32.sinh())
                .unwrap(),
            0.3_f32.tanh(),
            epsilon = 1e-4
        );
    }

    #[test]
    fn solve_sphere_line_intersection_miss() {
        // No collision with the line anywhere along the ray's line
        let ray = math::translate_along(&na::Vector3::new(0.0, 0.0, -0.5))
            * &Ray::new(math::origin(), na::Vector4::x());
        let line_normal0 = na::Vector4::x();
        let line_normal1 = na::Vector4::z();
        assert!(
            solve_sphere_line_intersection(&ray, &line_normal0, &line_normal1, 0.2_f32.sinh())
                .is_none()
        );
    }

    #[test]
    fn solve_sphere_line_intersection_margin() {
        // Sphere is already contacting the line, with some error
        let ray = math::translate_along(&na::Vector3::new(0.0, 0.0, -0.2))
            * &Ray::new(math::origin(), na::Vector4::z());
        let line_normal0 = na::Vector4::x();
        let line_normal1 = na::Vector4::z();
        assert_eq!(
            solve_sphere_line_intersection(&ray, &line_normal0, &line_normal1, 0.2001_f32.sinh())
                .unwrap(),
            0.0
        );
    }

    #[test]
    fn solve_sphere_line_intersection_precision() {
        // Using ray coordinates determined empirically from manual playtesting, show that the
        // current implementation of `solve_sphere_line_intersection` provides better results
        // than an arguably-simpler implementation involving the line's position and direction.
        // Similar reasoning can also apply to `solve_sphere_point_intersection` even though it is
        // not tested explicitly in the same way.
        let ray = Ray::new(
            na::Vector4::new(-0.019093871, -0.0014823675, 0.059645057, 1.0019588),
            na::Vector4::new(-0.02954007, 0.9965602, 0.07752046, 0.003702946),
        );
        let line_normal0 = na::Vector4::<f32>::x();
        let line_normal1 = na::Vector4::<f32>::y();
        let radius = 0.019090926_f32;
        // The following returns wrong results in the other implementation, so we test this case
        // to make sure there are no regressions.
        assert!(
            solve_sphere_line_intersection(&ray, &line_normal0, &line_normal1, radius.sinh())
                .is_none()
        );
    }

    #[test]
    fn solve_sphere_point_intersection_example() {
        // Hit the origin with a radius of 0.2
        let ray = math::translate_along(&na::Vector3::new(0.0, 0.0, -0.5))
            * &Ray::new(
                math::origin(),
                na::Vector4::new(1.0, 2.0, 6.0, 0.0).normalize(),
            );
        let point_position = math::origin();
        let point_normal0 = na::Vector4::x();
        let point_normal1 = na::Vector4::y();
        let point_normal2 = na::Vector4::z();
        let hit_point = math::lorentz_normalize(
            &ray.ray_point(
                solve_sphere_point_intersection(
                    &ray,
                    &point_normal0,
                    &point_normal1,
                    &point_normal2,
                    0.2_f32.sinh(),
                )
                .unwrap(),
            ),
        );
        assert_abs_diff_eq!(
            -math::mip(&hit_point, &point_position),
            0.2_f32.cosh(),
            epsilon = 1e-4
        );
    }

    #[test]
    fn solve_sphere_point_intersection_direct_hit() {
        // Directly hit the origin with a ray 0.5 units away and a radius of 0.2.
        let ray = math::translate_along(&na::Vector3::new(0.0, 0.0, -0.5))
            * &Ray::new(math::origin(), na::Vector4::z());
        let point_normal0 = na::Vector4::x();
        let point_normal1 = na::Vector4::y();
        let point_normal2 = na::Vector4::z();
        assert_abs_diff_eq!(
            solve_sphere_point_intersection(
                &ray,
                &point_normal0,
                &point_normal1,
                &point_normal2,
                0.2_f32.sinh()
            )
            .unwrap(),
            0.3_f32.tanh(),
            epsilon = 1e-4
        );
    }

    #[test]
    fn solve_sphere_point_intersection_miss() {
        // No collision with the point anywhere along the ray's line
        let ray = math::translate_along(&na::Vector3::new(0.0, 0.0, -0.5))
            * &Ray::new(math::origin(), na::Vector4::x());
        let point_normal0 = na::Vector4::x();
        let point_normal1 = na::Vector4::y();
        let point_normal2 = na::Vector4::z();
        assert!(solve_sphere_point_intersection(
            &ray,
            &point_normal0,
            &point_normal1,
            &point_normal2,
            0.2_f32.sinh()
        )
        .is_none());
    }

    #[test]
    fn solve_sphere_point_intersection_margin() {
        // Sphere is already contacting the point, with some error
        let ray = math::translate_along(&na::Vector3::new(0.0, 0.0, -0.2))
            * &Ray::new(math::origin(), na::Vector4::z());
        let point_normal0 = na::Vector4::x();
        let point_normal1 = na::Vector4::y();
        let point_normal2 = na::Vector4::z();
        assert_eq!(
            solve_sphere_point_intersection(
                &ray,
                &point_normal0,
                &point_normal1,
                &point_normal2,
                0.2001_f32.sinh()
            )
            .unwrap(),
            0.0
        );
    }

    #[test]
    fn solve_quadratic_example() {
        let a = 1.0;
        let b = -2.0;
        let c = 0.2;
        let x = solve_quadratic(c, b / 2.0, a).unwrap();

        // x should be a solution
        assert_abs_diff_eq!(a * x * x + b * x + c, 0.0, epsilon = 1e-4);

        // x should be the smallest solution, less than the parabola's vertex.
        assert!(x < -b / (2.0 * a));
    }

    #[test]
    fn tuv_to_xyz_example() {
        assert_eq!(tuv_to_xyz(0, [2, 4, 6]), [2, 4, 6]);
        assert_eq!(tuv_to_xyz(1, [2, 4, 6]), [6, 2, 4]);
        assert_eq!(tuv_to_xyz(2, [2, 4, 6]), [4, 6, 2]);

        assert_eq!(tuv_to_xyz(1, [2, 4, 6, 8]), [6, 2, 4, 8]);
    }

    /// Any voxel AABB should at least cover a capsule-shaped region consisting of all points
    /// `radius` units away from the ray's line segment. This region consists of two spheres
    /// and a cylinder. We only test planes because covered lines and points are a strict subset.
    #[test]
    fn voxel_aabb_coverage() {
        let dimension = 12;
        let layout = ChunkLayout::new(dimension);

        // Pick an arbitrary ray by transforming the positive-x-axis ray.
        let ray = na::Rotation3::from_axis_angle(&na::Vector3::z_axis(), 0.4).to_homogeneous()
            * math::translate_along(&na::Vector3::new(0.2, 0.3, 0.1))
            * &Ray::new(na::Vector4::w(), na::Vector4::x());

        let tanh_distance = 0.2;
        let radius = 0.1;

        // We want to test that the whole capsule-shaped region around the ray segment is covered by
        // the AABB. However, the math to test for this is complicated, so we instead check a bunch of
        // spheres along this ray segment.
        let num_ray_test_points = 20;
        let ray_test_points: Vec<_> = (0..num_ray_test_points)
            .map(|i| {
                math::lorentz_normalize(
                    &ray.ray_point(tanh_distance * (i as f32 / (num_ray_test_points - 1) as f32)),
                )
            })
            .collect();

        let aabb =
            VoxelAABB::from_ray_segment_and_radius(&layout, &ray, tanh_distance, radius).unwrap();

        // For variable names and further comments, we use a tuv coordinate system, which
        // is a permuted xyz coordinate system.

        // Test planes in all 3 axes.
        for t_axis in 0..3 {
            let covered_planes: HashSet<_> = aabb.grid_planes(t_axis).collect();

            // Check that all uv-aligned planes that should be covered are covered
            for t in 0..=dimension {
                if covered_planes.contains(&t) {
                    continue;
                }

                let mut plane_normal = na::Vector4::zeros();
                plane_normal[t_axis] = 1.0;
                plane_normal[3] = layout.grid_to_dual(t);
                let plane_normal = math::lorentz_normalize(&plane_normal);

                for test_point in &ray_test_points {
                    assert!(
                        math::mip(test_point, &plane_normal).abs() > radius.sinh(),
                        "Plane not covered: t_axis={t_axis}, t={t}, test_point={test_point:?}",
                    );
                }
            }
        }

        // Test lines in all 3 axes
        for t_axis in 0..3 {
            let u_axis = (t_axis + 1) % 3;
            let v_axis = (u_axis + 1) % 3;
            let covered_lines: HashSet<_> = aabb.grid_lines(u_axis, v_axis).collect();

            // For a given axis, all lines have the same direction, so set up the appropriate vector
            // in advance.
            let mut line_direction = na::Vector4::zeros();
            line_direction[t_axis] = 1.0;
            let line_direction = line_direction;

            // Check that all t-aligned lines that should be covered are covered
            for u in 0..=dimension {
                for v in 0..=dimension {
                    if covered_lines.contains(&(u, v)) {
                        continue;
                    }

                    let mut line_position = na::Vector4::zeros();
                    line_position[u_axis] = layout.grid_to_dual(u);
                    line_position[v_axis] = layout.grid_to_dual(v);
                    line_position[3] = 1.0;
                    let line_position = math::lorentz_normalize(&line_position);

                    for test_point in &ray_test_points {
                        assert!(
                            (math::mip(test_point, &line_position).powi(2)
                                - math::mip(test_point, &line_direction).powi(2))
                            .sqrt()
                                > radius.cosh(),
                            "Line not covered: t_axis={t_axis}, u={u}, v={v}, test_point={test_point:?}",
                        );
                    }
                }
            }
        }

        // Test points
        let covered_points: HashSet<_> = aabb.grid_points(0, 1, 2).collect();

        // Check that all points that should be covered are covered
        for x in 0..=dimension {
            for y in 0..=dimension {
                for z in 0..=dimension {
                    if covered_points.contains(&(x, y, z)) {
                        continue;
                    }

                    let point_position = math::lorentz_normalize(&na::Vector4::new(
                        layout.grid_to_dual(x),
                        layout.grid_to_dual(y),
                        layout.grid_to_dual(z),
                        1.0,
                    ));

                    for test_point in &ray_test_points {
                        assert!(
                            -math::mip(test_point, &point_position) > radius.cosh(),
                            "Point not covered: x={x}, y={y}, z={z}, test_point={test_point:?}",
                        );
                    }
                }
            }
        }
    }
}
