use crate::math::{MPoint, MVector};

/// A finite geodesic segment in hyperbolic space.
#[derive(Clone, Copy)]
pub struct LineSegment {
    start: MPoint<f32>,
    end: MPoint<f32>,
    tangent: Option<MVector<f32>>,
    tanh_length: f32,
}

impl LineSegment {
    pub fn new(start: MPoint<f32>, end: MPoint<f32>) -> Self {
        let cosh_length = -start.mip(&end);
        let sinh_length_squared = (cosh_length - 1.0) * (cosh_length + 1.0);
        let (tangent, tanh_length) = if sinh_length_squared > f32::EPSILON {
            let sinh_length = sinh_length_squared.sqrt();
            (
                Some((end.as_ref() + start.as_ref() * start.mip(&end)) / sinh_length),
                sinh_length / cosh_length,
            )
        } else {
            (None, 0.0)
        };
        Self {
            start,
            end,
            tangent,
            tanh_length,
        }
    }

    /// Returns the shortest hyperbolic distance from `point` to this segment.
    pub fn distance_to(&self, point: &MPoint<f32>) -> f32 {
        let Some(tangent) = &self.tangent else {
            return point.distance(&self.start).min(point.distance(&self.end));
        };
        let along_start = -point.mip(&self.start);
        let along_tangent = point.mip(tangent);
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
    fn segment_distance_clamps_to_endpoints_and_projects_inside() {
        let start = MPoint::origin();
        let along =
            |distance| MIsometry::translation_along(&na::Vector3::new(distance, 0.0, 0.0)) * start;
        let segment = LineSegment::new(start, along(1.0));
        assert_abs_diff_eq!(segment.distance_to(&along(-0.5)), 0.5, epsilon = 1e-5);
        assert_abs_diff_eq!(segment.distance_to(&along(0.5)), 0.0, epsilon = 1e-5);
        assert_abs_diff_eq!(segment.distance_to(&along(1.5)), 0.5, epsilon = 1e-5);
        let short = LineSegment::new(start, start);
        assert_abs_diff_eq!(short.distance_to(&along(0.5)), 0.5, epsilon = 1e-5);
    }
}
