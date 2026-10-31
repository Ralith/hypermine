use crate::math::{MPoint, MVector};

/// A finite geodesic segment in hyperbolic space.
#[derive(Clone, Copy)]
pub struct LineSegment {
    start: MPoint<f32>,
    end: MPoint<f32>,
    tangent: MVector<f32>,
    tanh_length: f32,
}

impl LineSegment {
    /// The endpoints must be separated enough to define a tangent at f32 precision.
    pub fn new(start: MPoint<f32>, end: MPoint<f32>) -> Self {
        let cosh_length = -start.mip(&end);
        let sinh_length_squared = (cosh_length - 1.0) * (cosh_length + 1.0);
        assert!(
            sinh_length_squared > f32::EPSILON,
            "line segment is too short"
        );
        let sinh_length = sinh_length_squared.sqrt();
        let tangent = (end.as_ref() + start.as_ref() * start.mip(&end)) / sinh_length;
        let tanh_length = sinh_length / cosh_length;
        Self {
            start,
            end,
            tangent,
            tanh_length,
        }
    }

    /// Returns the shortest hyperbolic distance from `point` to this segment.
    pub fn distance_to(&self, point: &MPoint<f32>) -> f32 {
        let along_start = -point.mip(&self.start);
        let along_tangent = point.mip(&self.tangent);
        if along_tangent <= 0.0 {
            point.distance(&self.start)
        } else if along_tangent >= along_start * self.tanh_length {
            point.distance(&self.end)
        } else {
            (along_start * along_start - along_tangent * along_tangent)
                .max(1.0)
                .sqrt()
                .acosh()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::MIsometry;
    use approx::assert_abs_diff_eq;

    #[test]
    // The segment distance uses the nearer endpoint outside the segment and perpendicular
    // projection for points whose closest point lies in its interior.
    fn segment_distance_clamps_to_endpoints_and_projects_inside() {
        let start = MPoint::origin();
        let along =
            |distance| MIsometry::translation_along(&na::Vector3::new(distance, 0.0, 0.0)) * start;
        let segment = LineSegment::new(start, along(1.0));
        assert_abs_diff_eq!(segment.distance_to(&along(-0.5)), 0.5, epsilon = 1e-5);
        assert_abs_diff_eq!(segment.distance_to(&along(0.5)), 0.0, epsilon = 1e-5);
        assert_abs_diff_eq!(segment.distance_to(&along(1.5)), 0.5, epsilon = 1e-5);
    }
}
