use std::ops::{Mul, Neg};

use crate::{
    dodeca::{Side, Vertex},
    math::{MDirection, MIsometry, MPoint, MVector},
};

/// A hyperbolic plane. This data structure uses a separate "exponent" field to
/// allow for planes very far from the origin. This struct is meant to be used
/// with world generation.
#[derive(Debug, Copy, Clone)]
pub struct Plane {
    scaled_normal: MVector<f64>,
    exponent: f64, // Multiply "normal" by e^exponent to get the actual normal vector
}

impl From<Side> for Plane {
    /// A surface overlapping with a particular dodecahedron side
    fn from(side: Side) -> Self {
        Plane::from(*side.normal_f64())
    }
}

impl From<MDirection<f64>> for Plane {
    fn from(normal: MDirection<f64>) -> Self {
        Plane {
            scaled_normal: normal.into(),
            exponent: 0.0,
        }
    }
}

impl From<na::Unit<na::Vector3<f64>>> for Plane {
    /// A plane passing through the origin
    fn from(x: na::UnitVector3<f64>) -> Self {
        Self::from(MDirection::from(x))
    }
}

impl Neg for Plane {
    type Output = Self;
    fn neg(self) -> Self {
        Self {
            scaled_normal: -self.scaled_normal,
            exponent: self.exponent,
        }
    }
}

impl Mul<Plane> for Side {
    type Output = Plane;
    /// Reflect a plane across the side
    fn mul(self, rhs: Plane) -> Plane {
        self.reflection_f64() * rhs
    }
}

impl Mul<Plane> for &MIsometry<f64> {
    type Output = Plane;
    fn mul(self, rhs: Plane) -> Plane {
        Plane {
            scaled_normal: self * rhs.scaled_normal,
            exponent: rhs.exponent,
        }
        .update_exponent()
    }
}

impl Plane {
    /// Hyperbolic normal vector identifying the plane, possibly scaled to avoid
    /// being too large to represent in an f64.
    pub fn scaled_normal(&self) -> &MVector<f64> {
        &self.scaled_normal
    }

    /// Shortest distance between the plane and a point
    pub fn distance_to(&self, point: &MPoint<f64>) -> f64 {
        if self.exponent == 0.0 {
            libm::asinh(self.scaled_normal.mip(point))
        } else {
            let mip_2 = self.scaled_normal.mip(point) * 2.0;
            (libm::log(mip_2.abs()) + self.exponent) * mip_2.signum()
        }
    }

    /// Like `distance_to`, but using chunk coordinates for a chunk in the same node space
    pub fn distance_to_chunk(&self, chunk: Vertex, coord: &na::Vector3<f64>) -> f64 {
        let pos = (MVector::from(chunk.chunk_to_node_f64() * coord.push(1.0))).normalized_point();
        self.distance_to(&pos)
    }

    fn update_exponent(mut self) -> Self {
        // For simplicity, we use the basic approximation of sinh whenever the
        // exponent is nonzero, so we want to only update the exponent when
        // we're sure such an approximation will be good enough. Once the
        // vector's w-coordinate is above 1.0e8, the error becomes unnoticeable
        // in double-precision floating point.

        // Note that this is a one-way operation. Since trying to use matrices
        // to transform a plane closer to the origin would result in a kind of
        // catastrophic cancellation, Plane is not designed to handle that kind
        // of use case.
        while self.scaled_normal.w.abs() > 1.0e8 {
            self.scaled_normal *= libm::exp(-16.0);
            self.exponent += 16.0;
        }
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::*;

    #[test]
    fn distance_sanity() {
        for &axis in &[
            na::Vector3::x_axis(),
            na::Vector3::y_axis(),
            na::Vector3::z_axis(),
        ] {
            for &distance in &[-1.5, 0.0, 1.5] {
                let plane = Plane::from(axis);
                assert_abs_diff_eq!(
                    plane.distance_to(
                        &(MIsometry::translation_along(&(axis.into_inner() * distance))
                            * MPoint::origin())
                    ),
                    distance
                );
            }
        }
    }

    #[test]
    fn check_surface_flipped() {
        let root = Plane::from(Side::A);
        assert_abs_diff_eq!(
            root.distance_to_chunk(Vertex::A, &na::Vector3::new(-1.0, 1.0, 1.0)),
            root.distance_to_chunk(Vertex::J, &na::Vector3::new(-1.0, 1.0, 1.0)) * -1.0,
            epsilon = 1e-5
        );
    }

    #[test]
    fn check_surface_on_plane() {
        assert_abs_diff_eq!(
            Plane::from(Side::A).distance_to_chunk(
                Vertex::from_sides([Side::A, Side::B, Side::C]).unwrap(),
                &na::Vector3::new(0.0, 0.7, 0.1), // The first 0.0 is important, the plane is the midplane of the cube in Side::A direction
            ),
            0.0,
            epsilon = 1e-8,
        );
    }

    #[test]
    fn check_elevation_consistency() {
        let abc = Vertex::from_sides([Side::A, Side::B, Side::C]).unwrap();

        // A cube corner should have the same elevation seen from different cubes
        assert_abs_diff_eq!(
            Plane::from(Side::A).distance_to_chunk(abc, &na::Vector3::new(1.0, 1.0, 1.0)),
            Plane::from(Side::A).distance_to_chunk(
                Vertex::from_sides([Side::F, Side::H, Side::J]).unwrap(),
                &na::Vector3::new(1.0, 1.0, 1.0),
            ),
            epsilon = 1e-8,
        );

        // The same corner should have the same distance_to_chunk when represented from the same cube at different corners
        assert_abs_diff_eq!(
            Plane::from(Side::A).distance_to_chunk(abc, &na::Vector3::new(0.0, 1.0, 1.0)),
            (Side::A * Plane::from(Side::A))
                .distance_to_chunk(abc, &na::Vector3::new(0.0, 1.0, 1.0),),
            epsilon = 1e-8,
        );

        // Corners of midplane cubes separated by the midplane should have the same distance_to_chunk with a different sign
        assert_abs_diff_eq!(
            Plane::from(Side::A).distance_to_chunk(abc, &na::Vector3::new(1.0, 1.0, 1.0)),
            -Plane::from(Side::A).distance_to_chunk(abc, &na::Vector3::new(-1.0, 1.0, 1.0)),
            epsilon = 1e-8,
        );

        // Corners of midplane cubes not separated by the midplane should have the same distance_to_chunk
        assert_abs_diff_eq!(
            Plane::from(Side::A).distance_to_chunk(abc, &na::Vector3::new(1.0, 1.0, 1.0)),
            Plane::from(Side::A).distance_to_chunk(abc, &na::Vector3::new(1.0, 1.0, -1.0)),
            epsilon = 1e-8,
        );
    }

    #[test]
    fn large_distances() {
        for offset in [10.0, -10.0] {
            let mut plane = Plane::from(MDirection::x());
            let point = MPoint::<f64>::origin();
            let mut expected_distance = 0.0;

            for _ in 0..200 {
                plane = &MIsometry::translation_along(&(na::Vector3::x() * offset)) * plane;
                expected_distance -= offset;
                assert_abs_diff_eq!(
                    plane.distance_to(&point),
                    expected_distance,
                    epsilon = 1.0e-8
                );
            }
        }
    }
}
