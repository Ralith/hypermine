//! From "Visualizing Hyperbolic Space: Unusual Uses of 4x4 Matrices." Phillips, Gunn.
//!
//! Vector4 values are assumed to be homogeneous Klein model coordinates unless otherwise
//! stated. Note that Minkowski model coordinates are valid Klein coordinates, but not vis versa.

use std::{f64, ops::Mul};

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
    pub fn from_homogeneous(v: &na::Vector4<N>) -> Self {
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

#[derive(Debug, Copy, Clone, Serialize, Deserialize, PartialEq)]
pub struct HIsometry3<N: RealField> {
    pub translation: na::Vector4<N>,
    pub rotation: na::UnitQuaternion<N>,
}

impl<N: RealField> HIsometry3<N> {
    pub fn identity() -> Self {
        Self {
            translation: origin(),
            rotation: na::one(),
        }
    }

    pub fn from_parts(translation: na::Vector4<N>, rotation: na::UnitQuaternion<N>) -> Self {
        debug_assert!(translation.w != N::zero());
        Self {
            translation,
            rotation,
        }
    }

    pub fn to_homogeneous(&self) -> na::Matrix4<N> {
        self.rotation.to_homogeneous() * translate(&origin(), &self.translation)
    }
}

impl<'a, 'b, N: RealField> Mul<&'b na::Vector4<N>> for &'a HIsometry3<N> {
    type Output = na::Vector4<N>;
    fn mul(self, rhs: &'b na::Vector4<N>) -> Self::Output {
        let translated = translate(&origin(), &self.translation) * rhs;
        let rotated = self.rotation * translated.xyz();
        na::Vector4::new(rotated.x, rotated.y, rotated.z, translated.w)
    }
}

impl<'a, 'b, N: RealField> Mul<&'b HIsometry3<N>> for &'a HIsometry3<N> {
    type Output = HIsometry3<N>;
    fn mul(self, rhs: &'b HIsometry3<N>) -> Self::Output {
        let x = rhs
            .rotation
            .inverse_transform_vector(&self.translation.xyz());
        let x = na::Vector4::new(x.x, x.y, x.z, self.translation.w);
        let translation = translate(&origin(), &x) * rhs.translation;

        let a = distance(&x, &translation);
        let b = distance(&translation, &origin());
        let c = distance(&origin(), &x);
        let angle_sum = loc_angle(a, b, c) + loc_angle(b, c, a) + loc_angle(c, a, b);
        let rotation = if angle_sum == N::zero() {
            na::one()
        } else {
            let defect = N::pi() - angle_sum;
            let axis = na::Unit::new_normalize(translation.xyz().cross(&x.xyz()));
            self.rotation * rhs.rotation * na::UnitQuaternion::from_axis_angle(&axis, defect)
        };
        HIsometry3 {
            translation,
            rotation,
        }
    }
}

#[cfg(test)]
impl<N: RealField> approx::AbsDiffEq for HIsometry3<N> {
    type Epsilon = N;

    fn default_epsilon() -> N {
        N::default_epsilon()
    }

    fn abs_diff_eq(&self, other: &Self, epsilon: N) -> bool {
        self.translation.abs_diff_eq(&other.translation, epsilon)
            && self.rotation.abs_diff_eq(&other.rotation, epsilon)
    }
}

/// Compute angle at the vertex opposite side `a` using the hyperbolic law of cosines
fn loc_angle<N: RealField>(a: N, b: N, c: N) -> N {
    // cosh a = cosh b cosh c - sinh b sinh c cos θ
    // θ = acos ((cosh b cosh c - cosh a) / (sinh b sinh c))
    let denom = b.sinh() * c.sinh();
    if denom == N::zero() {
        return N::zero();
    }
    ((b.cosh() * c.cosh() - a.cosh()) / denom).acos()
}

/// Point reflection around `p`
pub fn reflect<N: RealField>(p: &na::Vector4<N>) -> na::Matrix4<N> {
    na::Matrix4::<N>::identity()
        - (*p * p.transpose() * i31::<N>()) * na::convert::<_, N>(2.0) / mip(&p, &p)
}

/// Transform that translates `a` to `b`
pub fn translate<N: RealField>(a: &na::Vector4<N>, b: &na::Vector4<N>) -> na::Matrix4<N> {
    reflect(&midpoint(a, b)) * reflect(a)
}

#[rustfmt::skip]
pub fn translate_along<N: RealField>(v: &na::Unit<na::Vector3<N>>, distance: N) -> na::Matrix4<N> {
    if distance == na::zero() {
        return na::Matrix4::identity();
    }
    // g = Lorentz gamma factor
    let g = distance.cosh();
    let one = na::one::<N>();
    let gm = g - one;
    let bg = distance.sinh();
    // TODO: Make this more elegant
    na::Matrix4::new(
        one + gm * v.x * v.x,       gm * v.x * v.y,       gm * v.x * v.z, bg * v.x,
              gm * v.y * v.x, one + gm * v.y * v.y,       gm * v.y * v.z, bg * v.y,
              gm * v.z * v.x,       gm * v.z * v.y, one + gm * v.z * v.z, bg * v.z,
              bg * v.x,                   bg * v.y,             bg * v.z,        g)
}

/// 4D reflection around a normal vector; length is not significant (so long as it's nonzero)
pub fn euclidean_reflect<N: RealField>(v: &na::Vector4<N>) -> na::Matrix4<N> {
    na::Matrix4::identity() - v * v.transpose() * (na::convert::<_, N>(2.0) / v.norm_squared())
}

pub fn midpoint<N: RealField>(a: &na::Vector4<N>, b: &na::Vector4<N>) -> na::Vector4<N> {
    a * (mip(b, b) * mip(a, b)).sqrt() + b * (mip(a, a) * mip(a, b)).sqrt()
}

pub fn distance<N: RealField>(a: &na::Vector4<N>, b: &na::Vector4<N>) -> N {
    (mip(a, b).powi(2) / (mip(a, a) * mip(b, b))).sqrt().acosh()
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

pub fn renormalize_isometry<N: RealField>(m: &na::Matrix4<N>) -> na::Matrix4<N> {
    let dest = m.index((.., 3));
    let norm = dest.xyz().norm();
    let boost_length = (dest.w + norm).ln();
    let direction = na::Unit::new_unchecked(dest.xyz() / norm);
    let inverse_boost = translate_along(&direction, -boost_length);
    let rotation = renormalize_rotation_reflection(
        &(inverse_boost * m)
            .fixed_slice::<na::U3, na::U3>(0, 0)
            .clone_owned(),
    );
    translate_along(&direction, boost_length) * rotation.to_homogeneous()
}

#[rustfmt::skip]
fn renormalize_rotation_reflection<N: RealField>(m: &na::Matrix3<N>) -> na::Matrix3<N> {
    let zv = m.index((.., 2)).normalize();
    let yv = m.index((.., 1));
    let dot = zv.dot(&yv);
    let yv = na::Vector3::new(yv.x - dot * zv.x, yv.y - dot * zv.y, yv.z - dot * zv.z).normalize();
    let sign = m.determinant().signum();
    na::Matrix3::new(
        sign * (yv.y * zv.z - yv.z * zv.y), yv.x, zv.x,
	sign * (yv.z * zv.x - yv.x * zv.z), yv.y, zv.y,
	sign * (yv.x * zv.y - yv.y * zv.x), yv.z, zv.z,
    )
}

/// Whether an isometry reverses winding with respect to the norm
pub fn parity<N: RealField>(m: &na::Matrix4<N>) -> bool {
    m.fixed_slice::<na::U3, na::U3>(0, 0).determinant() < na::zero::<N>()
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
    fn translate_equivalence() {
        let a = na::Vector4::new(-0.5, -0.5, 0.0, 1.0);
        let o = na::Vector4::new(0.0, 0.0, 0.0, 1.0);
        let direction = na::Unit::new_normalize(a.xyz());
        let distance = dbg!(distance(&o, &a));
        assert_abs_diff_eq!(
            translate(&o, &a),
            translate_along(&direction, distance),
            epsilon = 1e-5
        );
    }

    #[test]
    fn translate_distance() {
        let dx = 2.3;
        let xf = translate_along(&na::Vector3::x_axis(), dx);
        assert_abs_diff_eq!(dx, distance(&origin(), &(xf * origin())));
    }

    #[test]
    fn distance_example() {
        let a = na::Vector4::new(0.2, 0.0, 0.0, 1.0);
        let b = na::Vector4::new(-0.5, -0.5, 0.0, 1.0);
        // Paper doubles distances for reasons unknown
        assert_abs_diff_eq!(distance(&a, &b), 2.074 / 2.0, epsilon = 1e-3);
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

    #[test]
    fn renormalize_translation() {
        let mat = translate(
            &na::Vector4::new(-0.5, -0.5, 0.0, 1.0),
            &na::Vector4::new(0.3, -0.7, 0.0, 1.0),
        );
        assert_abs_diff_eq!(renormalize_isometry(&mat), mat, epsilon = 1e-5);
    }

    #[test]
    #[rustfmt::skip]
    fn renormalize_reflection() {
        let mat = na::Matrix4::new(-1.0, 0.0, 0.0, 0.0,
                                   0.0, 1.0, 0.0, 0.0,
                                   0.0, 0.0, 1.0, 0.0,
                                   0.0, 0.0, 0.0, 1.0);
        assert_abs_diff_eq!(renormalize_isometry(&mat), mat, epsilon = 1e-5);
    }

    #[test]
    fn isometry_composition() {
        assert_abs_diff_eq!(
            &HIsometry3::<f64>::identity() * &HIsometry3::identity(),
            HIsometry3::identity()
        );

        let a = na::Vector4::new(0.5, 0.0, 0.0, 1.0);
        let b = na::Vector4::new(0.0, 0.5, 0.0, 1.0);
        println!(
            "{}\n{}",
            (&HIsometry3::<f64>::from_parts(a, na::one())
                * &HIsometry3::<f64>::from_parts(b, na::one()))
                .to_homogeneous(),
            translate(&origin(), &a) * translate(&origin(), &b)
        );
        assert_abs_diff_eq!(
            (&HIsometry3::<f64>::from_parts(a, na::one())
                * &HIsometry3::<f64>::from_parts(b, na::one()))
                .to_homogeneous(),
            translate(&origin(), &a) * translate(&origin(), &b),
            epsilon = 1e-3
        );
    }
}
