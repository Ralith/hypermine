use crate::{
    collision_math::Ray,
    math,
    node::{ChunkLayout, CoordAxis, CoordDirection, Coords, VoxelAABB, VoxelData},
    world::Material,
};

pub struct ChunkCastHit {
    /// The tanh of the distance traveled along the ray to result in this hit.
    pub tanh_distance: f32,

    /// The coordinates of the block that was hit, including margins.
    pub voxel_coords: Coords,

    /// Which of the three axes is orthogonal to the face of the block that was hit.
    pub face_axis: CoordAxis,

    /// The direction along `face_axis` corresponding to the outside of the face that was hit.
    pub face_direction: CoordDirection,
}

/// Performs ray casting against the voxels in the chunk with the given `voxel_data`
///
/// The `ray` parameter is given and any resulting hit normals are given in the chunk's dual coordinate system.
///
/// The `tanh_distance` is the hyperbolic tangent of the distance along the ray to check for hits.
pub fn chunk_ray_cast(
    voxel_data: &VoxelData,
    layout: &ChunkLayout,
    ray: &Ray,
    tanh_distance: f32,
) -> Option<ChunkCastHit> {
    let mut hit: Option<ChunkCastHit> = None;

    let Some(bounding_box) =
        VoxelAABB::from_ray_segment_and_radius(layout, ray, tanh_distance, 0.0)
    else {
        return None;
    };

    for t_axis in 0..3 {
        hit = find_face_collision(
            voxel_data,
            layout,
            &bounding_box,
            t_axis,
            ray,
            hit.as_ref().map_or(tanh_distance, |hit| hit.tanh_distance),
        )
        .or(hit);
    }

    hit
}

/// Detect intersections between a ray and the front side of a voxel face
fn find_face_collision(
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
        let normal = math::lorentz_normalize(&math::tuv_to_xyz(
            t_axis,
            na::Vector4::new(1.0, 0.0, 0.0, layout.grid_to_dual(t)),
        ));

        let Some(new_tanh_distance) = ray.solve_point_plane_intersection(&normal) else {
            continue;
        };

        // If new_tanh_distance is out of range, no collision occurred.
        if new_tanh_distance >= hit.as_ref().map_or(tanh_distance, |hit| hit.tanh_distance) {
            continue;
        }

        // Which side we approach the plane from affects which voxel we want to use for hit detection.
        // If exiting a chunk via a chunk boundary, hit detection is handled by a different chunk.
        // We also want to retain this face_direction for reporting the hit result later.
        let (face_direction, voxel_t) = if math::mip(&ray.direction, &normal) < 0.0 {
            if t == 0 {
                continue;
            }
            (CoordDirection::Plus, t - 1)
        } else {
            if t == layout.dimension() {
                continue;
            }
            (CoordDirection::Minus, t)
        };

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
            math::tuv_to_xyz(t_axis, [voxel_t, voxel_u, voxel_v]),
        ) {
            continue;
        }

        // A collision was found. Update the hit.
        hit = Some(ChunkCastHit {
            tanh_distance: new_tanh_distance,
            voxel_coords: Coords(math::tuv_to_xyz(t_axis, [voxel_t, voxel_u, voxel_v])),
            face_axis: CoordAxis::try_from(t_axis).unwrap(),
            face_direction,
        });
    }

    hit
}

/// Checks whether a voxel can be collided with. Any non-void voxel falls under this category.
fn voxel_is_solid(voxel_data: &VoxelData, layout: &ChunkLayout, coords: [u8; 3]) -> bool {
    debug_assert!(coords[0] < layout.dimension());
    debug_assert!(coords[1] < layout.dimension());
    debug_assert!(coords[2] < layout.dimension());
    voxel_data.get(Coords(coords).to_index(layout.dimension())) != Material::Void
}

#[cfg(test)]
mod tests {
    use crate::node::VoxelData;

    use super::*;

    /// Helper structure used to reduce the number of parameters to pass around with tests
    struct TestRayCastContext {
        layout: ChunkLayout,
        voxel_data: VoxelData,
    }

    impl TestRayCastContext {
        fn new() -> Self {
            let dimension: u8 = 12;

            let mut ctx = TestRayCastContext {
                layout: ChunkLayout::new(dimension),
                voxel_data: VoxelData::Solid(Material::Void),
            };

            // Populate voxels. Consists of a single voxel with voxel coordinates (1, 1, 1). The cube corresponding
            // to this voxel has grid coordinates from (1, 1, 1) to (2, 2, 2)
            ctx.set_voxel([1, 1, 1], Material::Dirt);

            ctx
        }

        fn set_voxel(&mut self, coords: [u8; 3], material: Material) {
            debug_assert!(coords[0] < self.layout.dimension());
            debug_assert!(coords[1] < self.layout.dimension());
            debug_assert!(coords[2] < self.layout.dimension());
            self.voxel_data.data_mut(self.layout.dimension())
                [Coords(coords).to_index(self.layout.dimension())] = material;
        }
    }

    /// Helper method to set up common parameters that are used
    /// in a passed-in closure to call ray casting methods.
    fn cast_with_test_ray(
        ctx: &TestRayCastContext,
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

    fn chunk_ray_cast_wrapper(
        ctx: &TestRayCastContext,
        ray: &Ray,
        tanh_distance: f32,
    ) -> Option<ChunkCastHit> {
        chunk_ray_cast(&ctx.voxel_data, &ctx.layout, ray, tanh_distance)
    }

    fn test_face_collision(
        ctx: &TestRayCastContext,
        ray: &Ray,
        tanh_distance: f32,
        expected_face_axis: CoordAxis,
        expected_face_direction: CoordDirection,
    ) {
        let hit = chunk_ray_cast_wrapper(ctx, ray, tanh_distance);
        let hit = hit.expect("collision expected");
        assert_eq!(hit.voxel_coords, Coords([1, 1, 1]));
        assert_eq!(hit.face_axis, expected_face_axis);
        assert_eq!(hit.face_direction, expected_face_direction);
        // sanity_check_normal(ray, &hit.unwrap()); TODO: Check other results
    }

    /// Tests that a suitable collision is found when approaching a single voxel from various angles and that
    /// no collision is found in paths that don't reach that voxel.
    #[test]
    fn chunk_ray_cast_examples() {
        let ctx = TestRayCastContext::new();

        // Approach a single voxel from various angles. Ensure that a suitable collision is found each time.
        // Note: The voxel is centered at (1.5, 1.5, 1.5) in the grid coordinates used in this test.

        cast_with_test_ray(
            &ctx,
            [0.0, 1.5, 1.5],
            [1.5, 1.5, 1.5],
            |ray, tanh_distance| {
                test_face_collision(
                    &ctx,
                    ray,
                    tanh_distance,
                    CoordAxis::X,
                    CoordDirection::Minus,
                );
            },
        );

        cast_with_test_ray(
            &ctx,
            [1.5, 1.5, 3.0],
            [1.5, 1.5, 1.5],
            |ray, tanh_distance| {
                test_face_collision(&ctx, ray, tanh_distance, CoordAxis::Z, CoordDirection::Plus);
            },
        );

        // No collision: Going sideways relative to a face
        cast_with_test_ray(
            &ctx,
            [3.0, 1.5, 1.5],
            [3.0, 3.0, 1.5],
            |ray, tanh_distance| {
                assert!(chunk_ray_cast_wrapper(&ctx, ray, tanh_distance).is_none());
            },
        );

        // No collision: Going away from a face
        cast_with_test_ray(
            &ctx,
            [3.0, 1.5, 1.5],
            [4.5, 1.5, 1.5],
            |ray, tanh_distance| {
                assert!(chunk_ray_cast_wrapper(&ctx, ray, tanh_distance).is_none());
            },
        );

        // No collision: Past cast endpoint
        cast_with_test_ray(
            &ctx,
            [8.0, 1.5, 1.5],
            [3.0, 1.5, 1.5],
            |ray, tanh_distance| {
                assert!(chunk_ray_cast_wrapper(&ctx, ray, tanh_distance).is_none());
            },
        );
    }

    /// Tests that colliding with a face from the back side is impossible. Note that colliding
    /// with the back side of an edge or vertex is still possible. Getting rid of these collisions
    /// is a possible future enhancement.
    #[test]
    fn face_collisions_one_sided() {
        let ctx = TestRayCastContext::new();

        cast_with_test_ray(
            &ctx,
            [1.5, 1.5, 1.5],
            [4.5, 1.5, 1.5],
            |ray, tanh_distance| {
                assert!(chunk_ray_cast_wrapper(&ctx, ray, tanh_distance).is_none());
            },
        )
    }
}
