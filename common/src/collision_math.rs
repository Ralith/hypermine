use crate::math::{MDirection, MIsometry, MPoint, MVector};

/// A ray in hyperbolic space. The position and direction must be orthogonal:
/// `mip(position, direction) == 0`.
#[derive(Debug)]
pub struct Ray {
    pub position: MPoint<f32>,
    pub direction: MDirection<f32>,
}

impl Ray {
    /// Constructs a new `Ray`. It is the caller's responsibility to ensure that
    /// `position` and `direction` are orthogonal.
    pub fn new(position: MPoint<f32>, direction: MDirection<f32>) -> Ray {
        Ray {
            position,
            direction,
        }
    }

    /// Returns an unnormalized vector representing a point along this ray
    /// `atanh(tanh_distance)` units away from the origin.
    pub fn ray_point(&self, tanh_distance: f32) -> MVector<f32> {
        self.position.as_ref() + self.direction.as_ref() * tanh_distance
    }

    /// Finds the tanh of the distance a sphere will have to travel along the ray before it
    /// intersects the given plane.
    pub fn solve_sphere_plane_intersection(
        &self,
        plane_normal: &MDirection<f32>,
        sinh_radius: f32,
    ) -> Option<f32> {
        let mip_pos_a = self.position.mip(plane_normal);
        let mip_dir_a = self.direction.mip(plane_normal);

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
        line_normal1: &MDirection<f32>,
        line_normal0: &MDirection<f32>,
        sinh_radius: f32,
    ) -> Option<f32> {
        let mip_pos_a = self.position.mip(line_normal0);
        let mip_dir_a = self.direction.mip(line_normal0);
        let mip_pos_b = self.position.mip(line_normal1);
        let mip_dir_b = self.direction.mip(line_normal1);

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
        point_normal0: &MDirection<f32>,
        point_normal1: &MDirection<f32>,
        point_normal2: &MDirection<f32>,
        sinh_radius: f32,
    ) -> Option<f32> {
        let mip_pos_a = self.position.mip(point_normal0);
        let mip_dir_a = self.direction.mip(point_normal0);
        let mip_pos_b = self.position.mip(point_normal1);
        let mip_dir_b = self.direction.mip(point_normal1);
        let mip_pos_c = self.position.mip(point_normal2);
        let mip_dir_c = self.direction.mip(point_normal2);

        solve_quadratic(
            mip_pos_a.powi(2) + mip_pos_b.powi(2) + mip_pos_c.powi(2) - sinh_radius.powi(2),
            mip_pos_a * mip_dir_a + mip_pos_b * mip_dir_b + mip_pos_c * mip_dir_c,
            mip_dir_a.powi(2) + mip_dir_b.powi(2) + mip_dir_c.powi(2) + sinh_radius.powi(2),
        )
    }

    /// Finds the tanh of the distance a point will have to travel along a ray before it
    /// intersects the given plane.
    pub fn solve_point_plane_intersection(&self, plane_normal: &MDirection<f32>) -> Option<f32> {
        let mip_pos_a = self.position.mip(plane_normal);
        let mip_dir_a = self.direction.mip(plane_normal);

        let result = -mip_pos_a / mip_dir_a;
        if result.is_finite() && result > 0.0 {
            Some(result)
        } else {
            None
        }
    }
}

impl std::ops::Mul<&Ray> for MIsometry<f32> {
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
        let ray = MIsometry::translation_along(&na::Vector3::new(0.0, 0.0, -0.5))
            * &Ray::new(
                MPoint::origin(),
                MDirection::new_unchecked(0.8, 0.0, 0.6, 0.0),
            );
        let normal = -MDirection::z();
        let hit_point = ray
            .ray_point(
                ray.solve_sphere_plane_intersection(&normal, 0.2_f32.sinh())
                    .unwrap(),
            )
            .normalized_point();

        assert_abs_diff_eq!(hit_point.mip(&normal), 0.2_f32.sinh(), epsilon = 1e-4);
    }

    #[test]
    fn solve_sphere_plane_intersection_direct_hit() {
        // Directly hit the z=0 plane with a ray 0.5 units away and a radius of 0.2.
        let ray = MIsometry::translation_along(&na::Vector3::new(0.0, 0.0, -0.5))
            * &Ray::new(MPoint::origin(), MDirection::z());
        let normal = -MDirection::z();
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
        let ray = MIsometry::translation_along(&na::Vector3::new(0.0, 0.0, -0.5))
            * &Ray::new(MPoint::origin(), MDirection::x());
        let normal = -MDirection::z();
        assert!(
            ray.solve_sphere_plane_intersection(&normal, 0.2_f32.sinh())
                .is_none()
        );
    }

    #[test]
    fn solve_sphere_plane_intersection_margin() {
        // Sphere is already contacting the plane, with some error
        let ray = MIsometry::translation_along(&na::Vector3::new(0.0, 0.0, -0.2))
            * &Ray::new(MPoint::origin(), MDirection::z());
        let normal = -MDirection::z();
        assert_eq!(
            ray.solve_sphere_plane_intersection(&normal, 0.2001_f32.sinh())
                .unwrap(),
            0.0
        );
    }

    #[test]
    fn solve_sphere_line_intersection_example() {
        // Hit the x=z=0 line with a radius of 0.2
        let ray = MIsometry::translation_along(&na::Vector3::new(0.0, 0.0, -0.5))
            * &Ray::new(
                MPoint::origin(),
                MVector::new(1.0, 2.0, 3.0, 0.0).normalized_direction(),
            );
        let line_normal0 = MDirection::x();
        let line_normal1 = MDirection::z();
        let hit_point = ray
            .ray_point(
                ray.solve_sphere_line_intersection(&line_normal0, &line_normal1, 0.2_f32.sinh())
                    .unwrap(),
            )
            .normalized_point();
        // Measue the distance from hit_point to the line and ensure it's equal to the radius
        assert_abs_diff_eq!(
            (hit_point.mip(&line_normal0).powi(2) + hit_point.mip(&line_normal1).powi(2)).sqrt(),
            0.2_f32.sinh(),
            epsilon = 1e-4
        );
    }

    #[test]
    fn solve_sphere_line_intersection_direct_hit() {
        // Directly hit the x=z=0 line with a ray 0.5 units away and a radius of 0.2.

        // Ensure the ray is slightly off-center so that the distance math is shown to be correct
        let ray = MIsometry::translation_along(&na::Vector3::new(0.0, 0.7, 0.0))
            * MIsometry::translation_along(&na::Vector3::new(0.0, 0.0, -0.5))
            * &Ray::new(MPoint::origin(), MDirection::z());
        let line_normal0 = MDirection::x();
        let line_normal1 = MDirection::z();
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
        let ray = MIsometry::translation_along(&na::Vector3::new(0.0, 0.0, -0.5))
            * &Ray::new(MPoint::origin(), MDirection::x());
        let line_normal0 = MDirection::x();
        let line_normal1 = MDirection::z();
        assert!(
            ray.solve_sphere_line_intersection(&line_normal0, &line_normal1, 0.2_f32.sinh())
                .is_none()
        );
    }

    #[test]
    fn solve_sphere_line_intersection_margin() {
        // Sphere is already contacting the line, with some error
        let ray = MIsometry::translation_along(&na::Vector3::new(0.0, 0.0, -0.2))
            * &Ray::new(MPoint::origin(), MDirection::z());
        let line_normal0 = MDirection::x();
        let line_normal1 = MDirection::z();
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
            MPoint::new_unchecked(-0.019093871, -0.0014823675, 0.059645057, 1.0019588),
            MDirection::new_unchecked(-0.02954007, 0.9965602, 0.07752046, 0.003702946),
        );
        let line_normal0 = MDirection::<f32>::x();
        let line_normal1 = MDirection::<f32>::y();
        let radius = 0.019090926_f32;
        // The following returns wrong results in the other implementation, so we test this case
        // to make sure there are no regressions.
        assert!(
            ray.solve_sphere_line_intersection(&line_normal0, &line_normal1, radius.sinh())
                .is_none()
        );
    }

    #[test]
    fn solve_sphere_point_intersection_example() {
        // Hit the origin with a radius of 0.2
        let ray = MIsometry::translation_along(&na::Vector3::new(0.0, 0.0, -0.5))
            * &Ray::new(
                MPoint::origin(),
                MVector::new(1.0, 2.0, 6.0, 0.0).normalized_direction(),
            );
        let point_position = MPoint::origin();
        let point_normal0 = MDirection::x();
        let point_normal1 = MDirection::y();
        let point_normal2 = MDirection::z();
        let hit_point = ray
            .ray_point(
                ray.solve_sphere_point_intersection(
                    &point_normal0,
                    &point_normal1,
                    &point_normal2,
                    0.2_f32.sinh(),
                )
                .unwrap(),
            )
            .normalized_point();
        assert_abs_diff_eq!(
            -hit_point.mip(&point_position),
            0.2_f32.cosh(),
            epsilon = 1e-4
        );
    }

    #[test]
    fn solve_sphere_point_intersection_direct_hit() {
        // Directly hit the origin with a ray 0.5 units away and a radius of 0.2.
        let ray = MIsometry::translation_along(&na::Vector3::new(0.0, 0.0, -0.5))
            * &Ray::new(MPoint::origin(), MDirection::z());
        let point_normal0 = MDirection::x();
        let point_normal1 = MDirection::y();
        let point_normal2 = MDirection::z();
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
        let ray = MIsometry::translation_along(&na::Vector3::new(0.0, 0.0, -0.5))
            * &Ray::new(MPoint::origin(), MDirection::x());
        let point_normal0 = MDirection::x();
        let point_normal1 = MDirection::y();
        let point_normal2 = MDirection::z();
        assert!(
            ray.solve_sphere_point_intersection(
                &point_normal0,
                &point_normal1,
                &point_normal2,
                0.2_f32.sinh()
            )
            .is_none()
        );
    }

    #[test]
    fn solve_sphere_point_intersection_margin() {
        // Sphere is already contacting the point, with some error
        let ray = MIsometry::translation_along(&na::Vector3::new(0.0, 0.0, -0.2))
            * &Ray::new(MPoint::origin(), MDirection::z());
        let point_normal0 = MDirection::x();
        let point_normal1 = MDirection::y();
        let point_normal2 = MDirection::z();
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
        let ray = MIsometry::translation_along(&na::Vector3::new(0.0, 0.0, -0.5))
            * &Ray::new(
                MPoint::origin(),
                MDirection::new_unchecked(0.8, 0.0, 0.6, 0.0),
            );
        let normal = -MDirection::z();
        let hit_point = ray
            .ray_point(ray.solve_point_plane_intersection(&normal).unwrap())
            .normalized_point();
        assert_abs_diff_eq!(hit_point.mip(&normal), 0.0, epsilon = 1e-4);
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
