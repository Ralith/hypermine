//! From "Visualizing Hyperbolic Space: Unusual Uses of 4x4 Matrices." Phillips, Gunn.
//!
//! Vector4 values are assumed to be homogeneous Klein model coordinates unless otherwise
//! stated. Note that Minkowski model coordinates are valid Klein coordinates, but not vis versa.

use std::f64;

use na::{RealField, Scalar};
use serde::{Deserialize, Serialize};

/// A point on the surface of the 3D hyperboloid in Minkowski coordinates with an implicit w
#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
#[repr(C)]
pub struct HPoint<N: Scalar>(na::Vector3<N>);

impl<N: Scalar> HPoint<N> {
    pub fn new(x: N, y: N, z: N) -> Self {
        Self(na::Vector3::new(x, y, z))
    }

    /// Construct from Minkowski coordinates
    pub fn from_homogeneous(v: na::Vector4<N>) -> Self {
        Self(v.xyz())
    }
}

impl<N: RealField> HPoint<N> {
    pub fn origin() -> Self {
        Self::new(na::zero(), na::zero(), na::zero())
    }

    /// Convert to Minkowski coordinates
    pub fn to_homogeneous(self) -> na::Vector4<N> {
        // x^2 + y^2 + z^2 - w^2 = -1
        // sqrt(x^2 + y^2 + z^2 + 1) = w
        let w = (self.0.x.powi(2) + self.0.y.powi(2) + self.0.z.powi(2) + na::one()).sqrt();
        na::Vector4::new(self.0.x, self.0.y, self.0.z, w)
    }
}

#[derive(Debug, Copy, Clone)]
pub struct HIsometry<N: RealField> {
    position: na::Vector4<N>,
    orientation: na::UnitQuaternion<N>,
}

impl<N: RealField> HIsometry<N> {
    pub fn identity() -> Self {
        Self {
            position: na::zero(),
            orientation: na::one(),
        }
    }

    pub fn to_homogeneous(&self) -> na::Matrix4<N> {
        translate(&na::zero(), &self.position) * self.orientation.to_homogeneous()
    }
}

pub fn reflect<N: RealField>(p: &na::Vector4<N>) -> na::Matrix4<N> {
    na::Matrix4::<N>::identity()
        - (*p * p.transpose() * i31::<N>()) * na::convert::<_, N>(2.0) / mip(&p, &p)
}

pub fn translate<N: RealField>(a: &na::Vector4<N>, b: &na::Vector4<N>) -> na::Matrix4<N> {
    reflect(&midpoint(a, b)) * reflect(a)
}

/// 4D reflection around a normal vector; length is not significant (so long as it's nonzero)
pub fn euclidean_reflect<N: RealField>(v: &na::Vector4<N>) -> na::Matrix4<N> {
    na::Matrix4::identity() - v * v.transpose() * (na::convert::<_, N>(2.0) / v.norm_squared())
}

pub fn midpoint<N: RealField>(a: &na::Vector4<N>, b: &na::Vector4<N>) -> na::Vector4<N> {
    a * (mip(b, b) * mip(a, b)).sqrt() + b * (mip(a, a) * mip(a, b)).sqrt()
}

pub fn distance<N: RealField>(a: &na::Vector4<N>, b: &na::Vector4<N>) -> N {
    na::convert::<_, N>(2.0) * (mip(a, b).powi(2) / (mip(a, a) * mip(b, b))).sqrt().acosh()
}

pub fn origin<N: RealField>() -> na::Vector4<N> {
    na::Vector4::new(na::zero(), na::zero(), na::zero(), na::one())
}

pub fn lorentz_normalize<N: RealField>(v: &na::Vector4<N>) -> na::Vector4<N> {
    let sf2 = mip(v, v);
    if sf2 == na::zero() {
        return origin();
    }
    let sf = sf2.abs().sqrt();
    na::Vector4::new(v.x / sf, v.y / sf, v.z / sf, v.w / sf)
}

/// Computes `a` in `(±a, ±a, ±a, sqrt(3a^2+1))`, the vertices of a cube centered on the origin
pub fn cubic_tiling_coordinate() -> f64 {
    // cosh a = cos A / sin B
    // subdivide the square into 8 congruent triangles around a vertex at the center
    // angle at center vertex is necessarily 2π/8
    // angles at the edge are 2π/4 and 2π/10
    // distance from the center of a face to the center of a neighboring edge:
    // acosh(cos(2π/8) / sin(2π/10)) = acosh(cos(π/4) / sin(π/5))
    let face_center_to_edge =
        (f64::consts::FRAC_PI_4.cos() / (f64::consts::PI / 5.0).sin()).acosh();
    // solve for edge length using property of hyperbolic lambert quadrilateral: tanh AF = cosh OA tanh OB
    let half_edge = (face_center_to_edge.cosh() * face_center_to_edge.tanh()).atanh();
    // edge length = 2 * acosh(2 * a^2 + 1)
    // cosh(edge length / 2) = 2 * a^2 + 1
    // sqrt(cosh(edge length / 2) - 1) / 2 = a
    (half_edge.cosh() - 1.0).sqrt() / 2.0
}

/// Minkowski inner product, aka <a, b>_h
fn mip<N: RealField>(a: &na::Vector4<N>, b: &na::Vector4<N>) -> N {
    a.x * b.x + a.y * b.y + a.z * b.z - a.w * b.w
}

#[rustfmt::skip]
fn i31<N: RealField>() -> na::Matrix4<N> {
    na::convert::<_, na::Matrix4<N>>(na::Matrix4::<f64>::new(
        1.0, 0.0, 0.0,  0.0,
        0.0, 1.0, 0.0,  0.0,
        0.0, 0.0, 1.0,  0.0,
        0.0, 0.0, 0.0, -1.0
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::*;

    #[test]
    #[rustfmt::skip]
    fn reflect_example() {
        assert_abs_diff_eq!(
            reflect(&na::Vector4::new(0.5, 0.0, 0.0, 1.0)),
            na::Matrix4::new(
                1.666, 0.0, 0.0, -1.333,
                0.0  , 1.0, 0.0,  0.0,
                0.0  , 0.0, 1.0,  0.0,
                1.333, 0.0, 0.0, -1.666
            ),
            epsilon = 1e-3
        );
    }

    #[test]
    #[rustfmt::skip]
    fn translate_example() {
        assert_abs_diff_eq!(
            translate(&na::Vector4::new(-0.5, -0.5, 0.0, 1.0), &na::Vector4::new(0.3, -0.7, 0.0, 1.0)),
            na::Matrix4::new(
                 1.676, 0.814, 0.0,  1.572,
                -1.369, 0.636, 0.0, -1.130,
                 0.0,   0.0,   1.0,  0.0,
                 1.919, 0.257, 0.0,  2.179,
            ),
            epsilon = 1e-3
        );
    }

    #[test]
    fn translate_identity() {
        let a = na::Vector4::new(-0.5, -0.5, 0.0, 1.0);
        let b = na::Vector4::new(0.3, -0.7, 0.0, 1.0);
        let o = na::Vector4::new(0.0, 0.0, 0.0, 1.0);
        assert_abs_diff_eq!(
            translate(&a, &b),
            translate(&o, &a) * translate(&o, &(translate(&a, &o) * b)) * translate(&a, &o),
            epsilon = 1e-5
        );
    }

    #[test]
    fn distance_example() {
        let a = na::Vector4::new(0.2, 0.0, 0.0, 1.0);
        let b = na::Vector4::new(-0.5, -0.5, 0.0, 1.0);
        assert_abs_diff_eq!(distance(&a, &b), 2.074, epsilon = 1e-3);
    }

    #[test]
    fn distance_commutative() {
        let p = HPoint::new(-1.0, -1.0, 0.0).to_homogeneous();
        let q = HPoint::new(1.0, -1.0, 0.0).to_homogeneous();
        assert_abs_diff_eq!(distance(&p, &q), distance(&q, &p));
    }

    #[test]
    fn midpoint_distance() {
        let p = HPoint::new(-1.0, -1.0, 0.0).to_homogeneous();
        let q = HPoint::new(1.0, -1.0, 0.0).to_homogeneous();
        let m = midpoint(&p, &q);
        assert_abs_diff_eq!(distance(&p, &m), distance(&m, &q), epsilon = 1e-5);
        assert_abs_diff_eq!(distance(&p, &m) * 2.0, distance(&p, &q), epsilon = 1e-5);
    }
}
