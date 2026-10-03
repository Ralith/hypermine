use crate::math::MPoint;

/// A finite geodesic segment in hyperbolic space.
#[derive(Clone, Copy)]
pub struct LineSegment {
    start: MPoint<f32>,
    end: MPoint<f32>,
}

impl LineSegment {
    pub fn new(start: MPoint<f32>, end: MPoint<f32>) -> Self {
        Self { start, end }
    }

    /// Returns the shortest hyperbolic distance from `point` to this segment.
    pub fn distance_to(&self, point: &MPoint<f32>) -> f32 {
        // The tangent at start in the direction of end, left unnormalized.
        let tangent = self.end.as_ref() + self.start.as_ref() * self.start.mip(&self.end);
        let tangent_norm_squared = tangent.mip(&tangent);
        if tangent_norm_squared <= f32::EPSILON {
            // The segment is shorter than the precision needed to project onto
            // its direction. Either endpoint is within this tiny segment length
            // of the closest point, so use the nearer one instead.
            return point.distance(&self.start).min(point.distance(&self.end));
        }

        let along_start = -point.mip(&self.start);
        let along_tangent = point.mip(&tangent);

        if along_tangent <= 0.0 {
            return point.distance(&self.start);
        }

        // At the end of the segment, the tangent coordinate is
        // along_start * tanh(length) * sinh(length). This comparison avoids
        // computing the segment length or normalizing the tangent.
        let cosh_length = -self.start.mip(&self.end);
        if along_tangent * cosh_length >= along_start * tangent_norm_squared {
            return point.distance(&self.end);
        }

        // This is the Minkowski-orthogonal projection onto the geodesic span,
        // scaled by tangent_norm_squared so the tangent itself stays raw.
        let projection = (self.start.as_ref() * (along_start * tangent_norm_squared)
            + tangent * along_tangent)
            .normalized_point();
        point.distance(&projection)
    }
}
