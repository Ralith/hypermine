use std::ops::{Mul, Neg};

use crate::{
    dodeca::{Side, Vertex},
    math::{MDirection, MIsometry, MPoint, MVector},
};

/// A hyperbolic plane
#[derive(Debug, Copy, Clone)]
pub struct Plane {
    normal: MDirection<f64>,
}

impl From<Side> for Plane {
    /// A surface overlapping with a particular dodecahedron side
    fn from(side: Side) -> Self {
        Self {
            normal: *side.normal_f64(),
        }
    }
}

impl From<na::Unit<na::Vector3<f64>>> for Plane {
    /// A plane passing through the origin
    fn from(x: na::Unit<na::Vector3<f64>>) -> Self {
        Self {
            normal: MDirection::from(x),
        }
    }
}

impl Neg for Plane {
    type Output = Self;
    fn neg(self) -> Self {
        Self {
            normal: -self.normal,
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
            normal: (self * rhs.normal).as_ref().normalized_direction(),
        }
    }
}

impl Plane {
    /// Hyperbolic normal vector identifying the plane
    pub fn normal(&self) -> &MDirection<f64> {
        &self.normal
    }

    /// Shortest distance between the plane and a point
    pub fn distance_to(&self, point: &MPoint<f64>) -> f64 {
        let mip_value = self.normal.mip(point);
        // Workaround for bug fixed in rust PR #72486
        libm::asinh(mip_value.abs()) * mip_value.signum()
    }

    /// Like `distance_to`, but using chunk coordinates for a chunk in the same node space
    pub fn distance_to_chunk(&self, chunk: Vertex, coord: &na::Vector3<f64>) -> f64 {
        let pos = (MVector::from(chunk.chunk_to_node_f64() * coord.push(1.0))).normalized_point();
        self.distance_to(&pos)
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
}
