//! From "Visualizing Hyperbolic Space: Unusual Uses of 4x4 Matrices." Phillips, Gunn.
//!
//! Vector4 values are assumed to be homogeneous Klein model coordinates unless otherwise
//! stated. Note that Minkowski model coordinates are valid Klein coordinates, but not vis versa.

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

impl<N: RealField + Copy> HPoint<N> {
    pub fn origin() -> Self {
        Self::new(na::zero(), na::zero(), na::zero())
    }

    /// Convert to Minkowski coordinates
    pub fn to_homogeneous(self) -> na::Vector4<N> {
        // x^2 + y^2 + z^2 - w^2 = -1
        // sqrt(x^2 + y^2 + z^2 + 1) = w
        let w = (sqr(self.0.x) + sqr(self.0.y) + sqr(self.0.z) + na::one()).sqrt();
        na::Vector4::new(self.0.x, self.0.y, self.0.z, w)
    }
}

/// Point or plane reflection around point or normal `p`
pub fn reflect<N: RealField + Copy>(p: &na::Vector4<N>) -> na::Matrix4<N> {
    na::Matrix4::<N>::identity()
        - minkowski_outer_product(p, p) * na::convert::<_, N>(2.0) / mip(p, p)
}

/// Transform that translates `a` to `b` given that `a` and `b` are Lorentz normalized pointlike vectors
pub fn translate<N: RealField + Copy>(a: &na::Vector4<N>, b: &na::Vector4<N>) -> na::Matrix4<N> {
    let a_plus_b = a + b;
    na::Matrix4::<N>::identity() - minkowski_outer_product(b, a) * na::convert::<_, N>(2.0)
        + minkowski_outer_product(&a_plus_b, &a_plus_b) / (N::one() - mip(a, b))
}

/// Transform that translates the origin in the direction of the given vector with distance equal to its magnitude
pub fn translate_along<N: RealField + Copy>(v: &na::Vector3<N>) -> na::Matrix4<N> {
    let norm = v.norm();
    if norm == na::zero() {
        return na::Matrix4::identity();
    }
    // g = Lorentz gamma factor
    let g = norm.cosh();
    let bgc = norm.sinhc();
    translate(&origin(), &(v * bgc).insert_row(3, g))
}

/// 4D reflection around a normal vector; length is not significant (so long as it's nonzero)
pub fn euclidean_reflect<N: RealField + Copy>(v: &na::Vector4<N>) -> na::Matrix4<N> {
    na::Matrix4::identity() - v * v.transpose() * (na::convert::<_, N>(2.0) / v.norm_squared())
}

pub fn midpoint<N: RealField + Copy>(a: &na::Vector4<N>, b: &na::Vector4<N>) -> na::Vector4<N> {
    a * (mip(b, b) * mip(a, b)).sqrt() + b * (mip(a, a) * mip(a, b)).sqrt()
}

pub fn distance<N: RealField + Copy>(a: &na::Vector4<N>, b: &na::Vector4<N>) -> N {
    (sqr(mip(a, b)) / (mip(a, a) * mip(b, b))).sqrt().acosh()
}

pub fn origin<N: RealField + Copy>() -> na::Vector4<N> {
    na::Vector4::new(na::zero(), na::zero(), na::zero(), na::one())
}

pub fn lorentz_normalize<N: RealField + Copy>(v: &na::Vector4<N>) -> na::Vector4<N> {
    let sf2 = mip(v, v);
    if sf2 == na::zero() {
        return origin();
    }
    let sf = sf2.abs().sqrt();
    v / sf
}

pub fn renormalize_isometry<N: RealField + Copy>(m: &na::Matrix4<N>) -> na::Matrix4<N> {
    let boost = translate(&origin(), &lorentz_normalize(&m.index((.., 3)).into()));
    let inverse_boost = mtranspose(&boost);
    let rotation = renormalize_rotation_reflection(
        &(inverse_boost * m).fixed_view::<3, 3>(0, 0).clone_owned(),
    );
    boost * rotation.to_homogeneous()
}

#[rustfmt::skip]
fn renormalize_rotation_reflection<N: RealField + Copy>(m: &na::Matrix3<N>) -> na::Matrix3<N> {
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
pub fn parity<N: RealField + Copy>(m: &na::Matrix4<N>) -> bool {
    m.fixed_view::<3, 3>(0, 0).determinant() < na::zero::<N>()
}

/// Minkowski inner product, aka <a, b>_h
pub fn mip<N: RealField + Copy>(a: &na::Vector4<N>, b: &na::Vector4<N>) -> N {
    a.x * b.x + a.y * b.y + a.z * b.z - a.w * b.w
}

#[inline]
pub fn sqr<N: RealField + Copy>(x: N) -> N {
    x * x
}

/// Minkowski transpose. Inverse for hyperbolic isometries
#[rustfmt::skip]
pub fn mtranspose<N: RealField + Copy>(m: &na::Matrix4<N>) -> na::Matrix4<N> {
    na::Matrix4::new(
         m.m11,  m.m21,  m.m31, -m.m41,
         m.m12,  m.m22,  m.m32, -m.m42,
         m.m13,  m.m23,  m.m33, -m.m43,
        -m.m14, -m.m24, -m.m34,  m.m44,
    )
}

/// Updates `subject` by moving it along the line determined by `projection_direction` so that
/// its dot product with `normal` is `distance`. This effectively projects vectors onto the plane
/// `distance` units away from the origin with normal `normal`. The projection is non-orthogonal in
/// general, only orthogonal when `normal` is equal to `projection_direction`.
///
/// Precondition: For this to be possible, `projection_direction` cannot be orthogonal to `normal`.
pub fn project_to_plane<N: RealField + Copy>(
    subject: &mut na::Vector3<N>,
    normal: &na::UnitVector3<N>,
    projection_direction: &na::UnitVector3<N>,
    distance: N,
) {
    *subject += projection_direction.as_ref()
        * ((distance - subject.dot(normal)) / projection_direction.dot(normal));
}

/// Returns the UnitQuaternion that rotates the `from` vector to the `to` vector, or `None` if
/// `from` and `to` face opposite directions such that their sum has norm less than `epsilon`.
/// This version is more numerically stable than nalgebra's equivalent function.
pub fn rotation_between_axis<N: RealField + Copy>(
    from: &na::UnitVector3<N>,
    to: &na::UnitVector3<N>,
    epsilon: N,
) -> Option<na::UnitQuaternion<N>> {
    let angle_bisector = na::UnitVector3::try_new(from.into_inner() + to.into_inner(), epsilon)?;
    Some(na::UnitQuaternion::new_unchecked(
        na::Quaternion::from_parts(from.dot(&angle_bisector), from.cross(&angle_bisector)),
    ))
}

/// Converts from t-u-v coordinates to x-y-z coordinates. t-u-v coordinates are a permuted version of x-y-z coordinates.
/// `t_axis` determines which of the three x-y-z coordinates corresponds to the t-coordinate. This function works with
/// any indexable entity with at least three entries. Any entry after the third entry is ignored.
pub fn tuv_to_xyz<T: std::ops::IndexMut<usize, Output = N>, N: Copy>(t_axis: usize, tuv: T) -> T {
    let mut result = tuv;
    (
        result[t_axis],
        result[(t_axis + 1) % 3],
        result[(t_axis + 2) % 3],
    ) = (result[0], result[1], result[2]);
    result
}

fn minkowski_outer_product<N: RealField + Copy>(
    a: &na::Vector4<N>,
    b: &na::Vector4<N>,
) -> na::Matrix4<N> {
    *a * na::RowVector4::new(b.x, b.y, b.z, -b.w)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::*;

    #[test]
    #[rustfmt::skip]
    fn reflect_example() {
        assert_abs_diff_eq!(
            reflect(&lorentz_normalize(&na::Vector4::new(0.5, 0.0, 0.0, 1.0))),
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
            translate(
                &lorentz_normalize(&na::Vector4::new(-0.5, -0.5, 0.0, 1.0)),
                &lorentz_normalize(&na::Vector4::new(0.3, -0.7, 0.0, 1.0))
            ),
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
        let a = lorentz_normalize(&na::Vector4::new(-0.5, -0.5, 0.0, 1.0));
        let b = lorentz_normalize(&na::Vector4::new(0.3, -0.7, 0.0, 1.0));
        let o = na::Vector4::new(0.0, 0.0, 0.0, 1.0);
        assert_abs_diff_eq!(
            translate(&a, &b),
            translate(&o, &a) * translate(&o, &(translate(&a, &o) * b)) * translate(&a, &o),
            epsilon = 1e-5
        );
    }

    #[test]
    fn translate_equivalence() {
        let a = lorentz_normalize(&na::Vector4::new(-0.5, -0.5, 0.0, 1.0));
        let o = na::Vector4::new(0.0, 0.0, 0.0, 1.0);
        let direction = a.xyz().normalize();
        let distance = dbg!(distance(&o, &a));
        assert_abs_diff_eq!(
            translate(&o, &a),
            translate_along(&(direction * distance)),
            epsilon = 1e-5
        );
    }

    #[test]
    fn translate_distance() {
        let dx = 2.3;
        let xf = translate_along(&(na::Vector3::x() * dx));
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
            &lorentz_normalize(&na::Vector4::new(-0.5, -0.5, 0.0, 1.0)),
            &lorentz_normalize(&na::Vector4::new(0.3, -0.7, 0.0, 1.0)),
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
    #[rustfmt::skip]
    fn renormalize_normalizes_matrix() {
        // Matrix chosen with random entries between -1 and 1
        let error = na::Matrix4::new(
            -0.77, -0.21,  0.57, -0.59,
             0.49, -0.68,  0.36,  0.68,
            -0.75, -0.54, -0.13, -0.59,
            -0.57, -0.80,  0.00, -0.53);

        // translation with some error
        let mat = translate(
            &lorentz_normalize(&na::Vector4::new(-0.5, -0.5, 0.0, 1.0)),
            &lorentz_normalize(&na::Vector4::new(0.3, -0.7, 0.0, 1.0)),
        ) + error * 0.05;

        let normalized_mat = renormalize_isometry(&mat);

        // Check that the matrix is actually normalized
        assert_abs_diff_eq!(
            mtranspose(&normalized_mat) * normalized_mat,
            na::Matrix4::identity(),
            epsilon = 1e-5
        );
    }

    #[test]
    fn project_to_plane_example() {
        let distance = 4.0;
        let projection_direction: na::UnitVector3<f32> =
            na::UnitVector3::new_normalize(na::Vector3::new(3.0, -2.0, 7.0));
        let normal: na::UnitVector3<f32> =
            na::UnitVector3::new_normalize(na::Vector3::new(3.0, -2.0, 7.0));
        let mut subject = na::Vector3::new(-6.0, -3.0, 4.0);
        project_to_plane(&mut subject, &normal, &projection_direction, distance);
        assert_abs_diff_eq!(normal.dot(&subject), distance, epsilon = 1.0e-5);
    }

    #[test]
    fn rotation_between_axis_example() {
        let from = na::UnitVector3::new_normalize(na::Vector3::new(1.0, 1.0, 3.0));
        let to = na::UnitVector3::new_normalize(na::Vector3::new(2.0, 3.0, 2.0));
        let expected = na::UnitQuaternion::rotation_between_axis(&from, &to).unwrap();
        let actual = rotation_between_axis(&from, &to, 1e-5).unwrap();
        assert_abs_diff_eq!(expected, actual, epsilon = 1.0e-5);
    }

    #[test]
    fn tuv_to_xyz_example() {
        assert_eq!(tuv_to_xyz(0, [2, 4, 6]), [2, 4, 6]);
        assert_eq!(tuv_to_xyz(1, [2, 4, 6]), [6, 2, 4]);
        assert_eq!(tuv_to_xyz(2, [2, 4, 6]), [4, 6, 2]);

        assert_eq!(tuv_to_xyz(1, [2, 4, 6, 8]), [6, 2, 4, 8]);
    }
}
