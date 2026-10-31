use crate::math::{MPoint, MVector};

/// A finite geodesic segment in hyperbolic space.
#[derive(Clone, Copy)]
pub struct LineSegment {
    start: MPoint<f32>,
    start_to_end: MVector<f32>,
}

impl LineSegment {
    pub fn new(start: MPoint<f32>, end: MPoint<f32>) -> Self {
        let start_to_end = (end.as_ref() / -end.mip(&start)) - start.as_ref();
        Self {
            start,
            start_to_end,
        }
    }

    /// Returns the shortest hyperbolic distance from `point` to this segment.
    pub fn distance_to(&self, point: &MPoint<f32>) -> f32 {
        // Work in the plane tangent to the hyperboloid at `start` and find the vectors to
        // `point` and the segment end.
        let start_to_point = (point.as_ref() / -point.mip(&self.start)) - self.start.as_ref();
        // Project onto the segment and clamp the closest point to its endpoints.
        let progress_along_segment =
            start_to_point.mip(&self.start_to_end) / self.start_to_end.mip(&self.start_to_end);
        let h = if progress_along_segment.is_nan() {
            0.0
        } else {
            progress_along_segment.clamp(0.0, 1.0)
        };
        let closest_point_on_segment =
            (self.start.as_ref() + self.start_to_end * h).normalized_point();
        (-closest_point_on_segment.mip(point)).acosh()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::MIsometry;
    use approx::assert_abs_diff_eq;

    #[test]
    fn segment_distance_clamps_to_endpoints_and_projects_inside() {
        // Check endpoint clamping, interior projection, and the degenerate-segment case.
        let start = MPoint::origin();
        let along =
            |distance| MIsometry::translation_along(&na::Vector3::new(distance, 0.0, 0.0)) * start;
        let segment = LineSegment::new(start, along(1.0));
        assert_abs_diff_eq!(segment.distance_to(&along(-0.5)), 0.5, epsilon = 1e-5);
        assert_abs_diff_eq!(segment.distance_to(&along(0.5)), 0.0, epsilon = 1e-5);
        assert_abs_diff_eq!(segment.distance_to(&along(1.5)), 0.5, epsilon = 1e-5);
        let degenerate = LineSegment::new(start, start);
        assert_abs_diff_eq!(degenerate.distance_to(&along(0.5)), 0.5, epsilon = 1e-5);
    }
}
