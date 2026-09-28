use crate::math::MPoint;

/// A finite geodesic segment in hyperbolic space.
#[derive(Clone, Copy)]
pub(super) struct LineSegment {
    start: MPoint<f32>,
    end: MPoint<f32>,
}

impl LineSegment {
    pub(super) fn new(start: MPoint<f32>, end: MPoint<f32>) -> Self {
        Self { start, end }
    }

    /// Returns the shortest hyperbolic distance from `point` to this segment.
    pub(super) fn distance_to(&self, point: &MPoint<f32>) -> f32 {
        let length = self.start.distance(&self.end);
        if length == 0.0 {
            return point.distance(&self.start);
        }

        // The tangent at start in the direction of end is the component of
        // `end` orthogonal to `start` in Minkowski space.
        let tangent = (self.end.as_ref() + self.start.as_ref() * self.start.mip(&self.end))
            .normalized_direction();
        let along_start = -point.mip(&self.start);
        let along_tangent = point.mip(&tangent);
        let along = (along_tangent / along_start).atanh();

        if along <= 0.0 {
            return point.distance(&self.start);
        }
        if along >= length {
            return point.distance(&self.end);
        }

        let projection = (self.start.as_ref() * along_start + tangent.as_ref() * along_tangent)
            .normalized_point();
        point.distance(&projection)
    }
}
