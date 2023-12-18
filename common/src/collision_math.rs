use crate::math;

/// A ray in hyperbolic space. The fields must be lorentz normalized, with `mip(position, position) == -1`,
/// `mip(direction, direction) == 1`, and `mip(position, direction) == 0`.
#[derive(Debug)]
pub struct Ray {
    pub position: na::Vector4<f32>,
    pub direction: na::Vector4<f32>,
}

impl Ray {
    pub fn new(position: na::Vector4<f32>, direction: na::Vector4<f32>) -> Ray {
        Ray {
            position,
            direction,
        }
    }

    /// Returns a point along this ray `atanh(tanh_distance)` units away from the origin. This point
    /// is _not_ lorentz normalized.
    pub fn ray_point(&self, tanh_distance: f32) -> na::Vector4<f32> {
        self.position + self.direction * tanh_distance
    }

    /// Finds the tanh of the distance a sphere will have to travel along the ray before it
    /// intersects the given plane.
    pub fn solve_sphere_plane_intersection(
        &self,
        plane_normal: &na::Vector4<f32>,
        sinh_radius: f32,
    ) -> Option<f32> {
        let mip_pos_a = math::mip(&self.position, plane_normal);
        let mip_dir_a = math::mip(&self.direction, plane_normal);

        solve_quadratic(
            mip_pos_a.powi(2) - sinh_radius.powi(2),
            mip_pos_a * mip_dir_a,
            mip_dir_a.powi(2) + sinh_radius.powi(2),
        )
    }

    /// Finds the tanh of the distance a sphere will have to travel along the ray before it
    /// intersects the given line.
    pub fn solve_sphere_line_intersection(
        &self,
        line_normal0: &na::Vector4<f32>,
        line_normal1: &na::Vector4<f32>,
        sinh_radius: f32,
    ) -> Option<f32> {
        let mip_pos_a = math::mip(&self.position, line_normal0);
        let mip_dir_a = math::mip(&self.direction, line_normal0);
        let mip_pos_b = math::mip(&self.position, line_normal1);
        let mip_dir_b = math::mip(&self.direction, line_normal1);

        solve_quadratic(
            mip_pos_a.powi(2) + mip_pos_b.powi(2) - sinh_radius.powi(2),
            mip_pos_a * mip_dir_a + mip_pos_b * mip_dir_b,
            mip_dir_a.powi(2) + mip_dir_b.powi(2) + sinh_radius.powi(2),
        )
    }

    /// Finds the tanh of the distance a sphere will have to travel along the ray before it
    /// intersects the given point.
    pub fn solve_sphere_point_intersection(
        &self,
        point_normal0: &na::Vector4<f32>,
        point_normal1: &na::Vector4<f32>,
        point_normal2: &na::Vector4<f32>,
        sinh_radius: f32,
    ) -> Option<f32> {
        let mip_pos_a = math::mip(&self.position, point_normal0);
        let mip_dir_a = math::mip(&self.direction, point_normal0);
        let mip_pos_b = math::mip(&self.position, point_normal1);
        let mip_dir_b = math::mip(&self.direction, point_normal1);
        let mip_pos_c = math::mip(&self.position, point_normal2);
        let mip_dir_c = math::mip(&self.direction, point_normal2);

        solve_quadratic(
            mip_pos_a.powi(2) + mip_pos_b.powi(2) + mip_pos_c.powi(2) - sinh_radius.powi(2),
            mip_pos_a * mip_dir_a + mip_pos_b * mip_dir_b + mip_pos_c * mip_dir_c,
            mip_dir_a.powi(2) + mip_dir_b.powi(2) + mip_dir_c.powi(2) + sinh_radius.powi(2),
        )
    }

    /// Finds the tanh of the distance a point will have to travel along a ray before it
    /// intersects the given plane.
    pub fn solve_point_plane_intersection(&self, plane_normal: &na::Vector4<f32>) -> Option<f32> {
        let mip_pos_a = math::mip(&self.position, plane_normal);
        let mip_dir_a = math::mip(&self.direction, plane_normal);

        let result = -mip_pos_a / mip_dir_a;
        if result.is_finite() && result > 0.0 {
            Some(result)
        } else {
            None
        }
    }
}

impl std::ops::Mul<&Ray> for na::Matrix4<f32> {
    type Output = Ray;

    #[inline]
    fn mul(self, rhs: &Ray) -> Self::Output {
        Ray {
            position: self * rhs.position,
            direction: self * rhs.direction,
        }
    }
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

#[cfg(test)]
mod tests {
    use approx::assert_abs_diff_eq;

    use super::*;

    #[test]
    fn solve_sphere_plane_intersection_example() {
        // Hit the z=0 plane with a radius of 0.2
        let ray = math::translate_along(&na::Vector3::new(0.0, 0.0, -0.5))
            * &Ray::new(math::origin(), na::Vector4::new(0.8, 0.0, 0.6, 0.0));
        let normal = -na::Vector4::z();
        let hit_point = math::lorentz_normalize(
            &ray.ray_point(
                ray.solve_sphere_plane_intersection(&normal, 0.2_f32.sinh())
                    .unwrap(),
            ),
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
            ray.solve_sphere_plane_intersection(&normal, 0.2_f32.sinh())
                .unwrap(),
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
        assert!(ray
            .solve_sphere_plane_intersection(&normal, 0.2_f32.sinh())
            .is_none());
    }

    #[test]
    fn solve_sphere_plane_intersection_margin() {
        // Sphere is already contacting the plane, with some error
        let ray = math::translate_along(&na::Vector3::new(0.0, 0.0, -0.2))
            * &Ray::new(math::origin(), na::Vector4::z());
        let normal = -na::Vector4::z();
        assert_eq!(
            ray.solve_sphere_plane_intersection(&normal, 0.2001_f32.sinh())
                .unwrap(),
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
                ray.solve_sphere_line_intersection(&line_normal0, &line_normal1, 0.2_f32.sinh())
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
            ray.solve_sphere_line_intersection(&line_normal0, &line_normal1, 0.2_f32.sinh())
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
        assert!(ray
            .solve_sphere_line_intersection(&line_normal0, &line_normal1, 0.2_f32.sinh())
            .is_none());
    }

    #[test]
    fn solve_sphere_line_intersection_margin() {
        // Sphere is already contacting the line, with some error
        let ray = math::translate_along(&na::Vector3::new(0.0, 0.0, -0.2))
            * &Ray::new(math::origin(), na::Vector4::z());
        let line_normal0 = na::Vector4::x();
        let line_normal1 = na::Vector4::z();
        assert_eq!(
            ray.solve_sphere_line_intersection(&line_normal0, &line_normal1, 0.2001_f32.sinh())
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
        assert!(ray
            .solve_sphere_line_intersection(&line_normal0, &line_normal1, radius.sinh())
            .is_none());
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
                ray.solve_sphere_point_intersection(
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
            ray.solve_sphere_point_intersection(
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
        assert!(ray
            .solve_sphere_point_intersection(
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
            ray.solve_sphere_point_intersection(
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
    fn foo() {
        // Hit the z=0 plane
        let ray = math::translate_along(&na::Vector3::new(0.0, 0.0, -0.5))
            * &Ray::new(math::origin(), na::Vector4::new(0.8, 0.0, 0.6, 0.0));
        let normal = -na::Vector4::z();
        let hit_point = math::lorentz_normalize(
            &ray.ray_point(ray.solve_point_plane_intersection(&normal).unwrap()),
        );
        assert_abs_diff_eq!(math::mip(&hit_point, &normal), 0.0, epsilon = 1e-4);
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
}
