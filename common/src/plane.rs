use std::ops::{Mul, Neg};

use crate::{
    dodeca::{Side, Vertex},
    math::{MIsometry, MVector},
};

/// A hyperbolic plane
#[derive(Debug, Copy, Clone)]
pub struct Plane<N: na::RealField> {
    normal: MVector<N>,
}

impl From<Side> for Plane<f64> {
    /// A surface overlapping with a particular dodecahedron side
    fn from(side: Side) -> Self {
        Self {
            normal: *side.normal_f64(),
        }
    }
}

impl<N: na::RealField + Copy> From<na::Unit<na::Vector3<N>>> for Plane<N> {
    /// A plane passing through the origin
    fn from(x: na::Unit<na::Vector3<N>>) -> Self {
        Self {
            normal: MVector::from(x),
        }
    }
}

impl<N: na::RealField + Copy> Neg for Plane<N> {
    type Output = Self;
    fn neg(self) -> Self {
        Self {
            normal: -self.normal,
        }
    }
}

impl Mul<Plane<f64>> for Side {
    type Output = Plane<f64>;
    /// Reflect a plane across the side
    fn mul(self, rhs: Plane<f64>) -> Plane<f64> {
        self.reflection_f64() * rhs
    }
}

impl<N: na::RealField + Copy> Mul<Plane<N>> for &MIsometry<N> {
    type Output = Plane<N>;
    fn mul(self, rhs: Plane<N>) -> Plane<N> {
        Plane {
            normal: (*self * rhs.normal).lorentz_normalize(),
        }
    }
}

impl<N: na::RealField + Copy> Plane<N> {
    /// Hyperbolic normal vector identifying the plane
    pub fn normal(&self) -> &MVector<N> {
        &self.normal
    }

    /// Shortest distance between the plane and a point
    pub fn distance_to(&self, point: &MVector<N>) -> N {
        let mip_value = self.normal.mip(point);
        // Workaround for bug fixed in rust PR #72486
        mip_value.abs().asinh() * mip_value.signum()
    }
}

impl Plane<f64> {
    /// Like `distance_to`, but using chunk coordinates for a chunk in the same node space
    pub fn distance_to_chunk(&self, chunk: Vertex, coord: &na::Vector3<f64>) -> f64 {
        let pos = (MVector::from(chunk.chunk_to_node_f64() * coord.push(1.0))).lorentz_normalize();
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
                    plane.distance_to(
                        &(translate_along(&(axis.into_inner() * distance)) * MVector::origin())
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
                Vertex::from_sides(Side::A, Side::B, Side::C).unwrap(),
                &na::Vector3::new(0.0, 0.7, 0.1), // The first 0.0 is important, the plane is the midplane of the cube in Side::A direction
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
            Plane::from(Side::A).distance_to_chunk(abc, &na::Vector3::new(1.0, 1.0, 1.0)),
            Plane::from(Side::A).distance_to_chunk(
                Vertex::from_sides(Side::F, Side::H, Side::J).unwrap(),
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
