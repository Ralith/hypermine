use std::ops::Mul;

use na::RealField;
use serde::{Deserialize, Serialize};

use super::{distance, origin, translate};

/// A hyperbolic translation followed by a rotation
#[derive(Debug, Copy, Clone, Serialize, Deserialize, PartialEq)]
pub struct Isometry<N: RealField> {
    pub translation: na::Vector4<N>,
    pub rotation: na::UnitQuaternion<N>,
}

impl<N: RealField> Isometry<N> {
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

impl<'a, 'b, N: RealField> Mul<&'b na::Vector4<N>> for &'a Isometry<N> {
    type Output = na::Vector4<N>;
    fn mul(self, rhs: &'b na::Vector4<N>) -> Self::Output {
        let translated = translate(&origin(), &self.translation) * rhs;
        let rotated = self.rotation * translated.xyz();
        na::Vector4::new(rotated.x, rotated.y, rotated.z, translated.w)
    }
}

impl<'a, 'b, N: RealField> Mul<&'b Isometry<N>> for &'a Isometry<N> {
    type Output = Isometry<N>;
    fn mul(self, rhs: &'b Isometry<N>) -> Self::Output {
        let x = rhs
            .rotation
            .inverse_transform_vector(&self.translation.xyz());
        let x = na::Vector4::new(x.x, x.y, x.z, self.translation.w);
        let translation = translate(&origin(), &x) * rhs.translation;

        let a = distance(&x, &translation);
        let b = distance(&translation, &origin());
        let c = distance(&origin(), &x);
        let angle_sum = loc_angle(a, b, c) + loc_angle(b, c, a) + loc_angle(c, a, b);
        let rotation = if angle_sum == N::zero() || angle_sum >= N::pi() {
            self.rotation * rhs.rotation
        } else {
            let defect = N::pi() - angle_sum;
            let axis = na::Unit::new_normalize(translation.xyz().cross(&x.xyz()));
            self.rotation * rhs.rotation * na::UnitQuaternion::from_axis_angle(&axis, defect)
        };
        Isometry {
            translation,
            rotation,
        }
    }
}

#[cfg(test)]
impl<N: RealField> approx::AbsDiffEq for Isometry<N> {
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
    na::clamp(
        (b.cosh() * c.cosh() - a.cosh()) / denom,
        -N::one(),
        N::one(),
    )
    .acos()
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::*;

    #[test]
    fn simple() {
        assert_abs_diff_eq!(
            &Isometry::<f64>::identity() * &Isometry::identity(),
            Isometry::identity()
        );

        let a = na::Vector4::new(0.5, 0.0, 0.0, 1.0);
        println!(
            "{}\n{}",
            (&Isometry::<f64>::from_parts(a, na::one())).to_homogeneous(),
            translate(&origin(), &a)
        );
        assert_abs_diff_eq!(
            (&Isometry::<f64>::from_parts(a, na::one())).to_homogeneous(),
            translate(&origin(), &a),
            epsilon = 1e-5
        );
    }

    #[test]
    fn rotation_composition() {
        let q = na::UnitQuaternion::from_axis_angle(&na::Vector3::x_axis(), 1.0);
        assert_abs_diff_eq!(
            (&Isometry::<f64>::from_parts(origin(), q) * &Isometry::<f64>::from_parts(origin(), q))
                .to_homogeneous(),
            (q * q).to_homogeneous(),
            epsilon = 1e-5
        );
    }

    #[test]
    fn homogenize_distributes() {
        assert_abs_diff_eq!(
            &Isometry::<f64>::identity() * &Isometry::identity(),
            Isometry::identity()
        );

        let a = na::Vector4::new(0.5, 0.0, 0.0, 1.0);
        let b = na::Vector4::new(0.0, 0.5, 0.0, 1.0);
        println!(
            "{}\n{}",
            (&Isometry::<f64>::from_parts(a, na::one())
                * &Isometry::<f64>::from_parts(b, na::one()))
                .to_homogeneous(),
            &Isometry::<f64>::from_parts(a, na::one()).to_homogeneous()
                * &Isometry::<f64>::from_parts(b, na::one()).to_homogeneous()
        );
        assert_abs_diff_eq!(
            (&Isometry::<f64>::from_parts(a, na::one())
                * &Isometry::<f64>::from_parts(b, na::one()))
                .to_homogeneous(),
            &Isometry::<f64>::from_parts(a, na::one()).to_homogeneous()
                * &Isometry::<f64>::from_parts(b, na::one()).to_homogeneous(),
            epsilon = 1e-5
        );
    }

    #[test]
    fn translation_composition() {
        assert_abs_diff_eq!(
            &Isometry::<f64>::identity() * &Isometry::identity(),
            Isometry::identity()
        );

        let a = na::Vector4::new(0.5, 0.0, 0.0, 1.0);
        let b = na::Vector4::new(0.0, 0.5, 0.0, 1.0);
        println!(
            "{}\n{}",
            (&Isometry::<f64>::from_parts(a, na::one())
                * &Isometry::<f64>::from_parts(b, na::one()))
                .to_homogeneous(),
            translate(&origin(), &a) * translate(&origin(), &b)
        );
        assert_abs_diff_eq!(
            (&Isometry::<f64>::from_parts(a, na::one())
                * &Isometry::<f64>::from_parts(b, na::one()))
                .to_homogeneous(),
            translate(&origin(), &a) * translate(&origin(), &b),
            epsilon = 1e-3
        );
    }
}
