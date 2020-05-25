use std::ops::{Mul, Neg};

use crate::{
    dodeca::{Side, Vertex},
    math::{lorentz_normalize, mip, origin},
};

/// A hyperbolic plane
#[derive(Debug, Copy, Clone)]
pub struct Plane {
    normal: na::Vector4<f64>,
}

impl From<Side> for Plane {
    /// A surface overlapping with a particular dodecahedron side
    fn from(side: Side) -> Self {
        let n = side.reflection().column(3) - origin();
        Self {
            normal: lorentz_normalize(&n),
        }
    }
}

impl From<na::Unit<na::Vector3<f64>>> for Plane {
    /// A plane passing through the origin
    fn from(x: na::Unit<na::Vector3<f64>>) -> Self {
        Self {
            normal: x.into_inner().push(0.0),
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
        self.reflection() * rhs
    }
}

impl<'a> Mul<Plane> for &'a na::Matrix4<f64> {
    type Output = Plane;
    fn mul(self, rhs: Plane) -> Plane {
        Plane {
            normal: lorentz_normalize(&(self * rhs.normal)),
        }
    }
}

impl Plane {
    /// Hyperbolic normal vector identifying the plane
    pub fn normal(&self) -> &na::Vector4<f64> {
        &self.normal
    }

    /// Shortest distance between the plane and a point
    pub fn distance_to(&self, point: &na::Vector4<f64>) -> f64 {
        let mip_value = mip(&self.normal, point);
        // Workaround for bug fixed in rust PR #72486
        mip_value.abs().asinh().copysign(mip_value)
    }

    /// Like `distance_to`, but using chunk coordinates for a chunk in the same node space
    pub fn distance_to_chunk(&self, chunk: Vertex, coord: &na::Vector3<f64>) -> f64 {
        let pos = lorentz_normalize(&(chunk.chunk_to_node() * coord.push(1.0)));
        self.distance_to(&pos)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::translate_along;
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
                    plane.distance_to(&(translate_along(&axis, distance) * origin())),
                    distance
                );
            }
        }
    }

    #[test]
    fn check_surface_flipped() {
        let root = Plane::from(Side::A);
        assert_abs_diff_eq!(
            root.distance_to_chunk(Vertex::A, &(na::Vector3::x() * 2.0)),
            root.distance_to_chunk(Vertex::J, &(na::Vector3::x() * 2.0)) * -1.0,
            epsilon = 1e-5
        );
    }

    #[test]
    fn check_surface_on_plane() {
        assert_abs_diff_eq!(
            Plane::from(Side::A).distance_to_chunk(
                Vertex::from_sides(Side::A, Side::B, Side::C).unwrap(),
                &na::Vector3::new(1.0, 0.3, 0.9), // The first 1.0 is important, the plane is the midplane of the cube in Side::A direction
            ),
            0.0,
            epsilon = 1e-8,
        );
    }

    #[test]
    fn check_elevation_consistency() {
        let abc = Vertex::from_sides(Side::A, Side::B, Side::C).unwrap();

        // A cube corner should have the same elevation seen from different cubes
        assert_abs_diff_eq!(
            Plane::from(Side::A).distance_to_chunk(abc, &na::Vector3::new(0.0, 0.0, 0.0)),
            Plane::from(Side::A).distance_to_chunk(
                Vertex::from_sides(Side::F, Side::H, Side::J).unwrap(),
                &na::Vector3::new(0.0, 0.0, 0.0),
            ),
            epsilon = 1e-8,
        );

        // The same corner should have the same distance_to_chunk when represented from the same cube at different corners
        assert_abs_diff_eq!(
            Plane::from(Side::A).distance_to_chunk(abc, &na::Vector3::new(1.0, 0.0, 0.0)),
            (Side::A * Plane::from(Side::A))
                .distance_to_chunk(abc, &na::Vector3::new(1.0, 0.0, 0.0),),
            epsilon = 1e-8,
        );

        // Corners of midplane cubes separated by the midplane should have the same distance_to_chunk with a different sign
        assert_abs_diff_eq!(
            Plane::from(Side::A).distance_to_chunk(abc, &na::Vector3::new(0.0, 0.0, 0.0)),
            -Plane::from(Side::A).distance_to_chunk(abc, &na::Vector3::new(2.0, 0.0, 0.0)),
            epsilon = 1e-8,
        );

        // Corners of midplane cubes not separated by the midplane should have the same distance_to_chunk
        assert_abs_diff_eq!(
            Plane::from(Side::A).distance_to_chunk(abc, &na::Vector3::new(0.0, 0.0, 0.0)),
            Plane::from(Side::A).distance_to_chunk(abc, &na::Vector3::new(0.0, 0.0, 2.0)),
            epsilon = 1e-8,
        );
    }
}
