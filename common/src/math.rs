//! From "Visualizing Hyperbolic Space: Unusual Uses of 4x4 Matrices." Phillips, Gunn.
//!
//! Vector4 values are assumed to be homogeneous Klein model coordinates unless otherwise
//! stated. Note that Minkowski model coordinates are valid Klein coordinates, but not vis versa.

// all the inline functions are basically just wrappers around corresponding nalgebra functions

use na::{RealField, Scalar};
use serde::{Deserialize, Serialize};
use std::ops::*;

#[derive(Debug, Copy, Clone, Serialize, Deserialize,PartialEq)]
#[repr(C)]
pub struct MVector<N: Scalar>(na::Vector4<N>);

#[derive(Debug, Copy, Clone, Serialize, Deserialize,PartialEq)]
#[repr(C)]
pub struct MIsometry<N: Scalar>(na::Matrix4<N>);

#[cfg(test)]
impl<N: RealField> approx::AbsDiffEq<MIsometry<N>> for MIsometry<N>
{
    type Epsilon = N;
    #[inline]
    fn default_epsilon() -> Self::Epsilon
    {
        na::Matrix4::<N>::default_epsilon()
    }
    #[inline]
    fn abs_diff_eq(&self, other: &Self, epsilon: Self::Epsilon) -> bool
    {
        self.abs_diff_eq(other, epsilon)
    }
}

#[cfg(test)]
impl<N: RealField> approx::AbsDiffEq<MVector<N>> for MVector<N>
{
    type Epsilon = N;
    #[inline]
    fn default_epsilon() -> Self::Epsilon
    {
        na::Vector4::<N>::default_epsilon()
    }
    #[inline]
    fn abs_diff_eq(&self, other: &Self, epsilon: Self::Epsilon) -> bool
    {
        self.abs_diff_eq(other, epsilon)
    }
}

impl<N: RealField> From<na::Unit<na::Vector3<N>>> for MVector<N>
{
    fn from(value: na::Unit<na::Vector3<N>>) -> Self
    {
        Self(value.into_inner().push(na::zero()))
    }
}

impl<N: Scalar> From<na::Vector4<N>> for MVector<N>
{
    fn from(value: na::Vector4<N>) -> Self
    {
        Self(value)
    }
}

impl<N: Scalar> Deref for MVector<N>
{
    type Target = na::coordinates::XYZW<N>;
    #[inline]
    fn deref(&self) -> &Self::Target
    {
        self.0.deref()
    }
}

impl<N: Scalar> DerefMut for MVector<N> {
    #[inline]
    fn deref_mut(&mut self) -> &mut Self::Target
    {
        self.0.deref_mut()
    }
}

impl<N: Scalar> Deref for MIsometry<N> {
    type Target = na::coordinates::M4x4<N>;
    #[inline]
    fn deref(&self) -> &Self::Target
    {
        self.0.deref()
    }
}

impl<N: Scalar> Into<na::Matrix4<N>> for MIsometry<N>
{
    fn into(self) -> na::Matrix4<N>
    {
        self.0
    }
}


impl MIsometry<f32>
{
    pub fn to_f64(self) -> MIsometry<f64>
    {
        MIsometry(self.0.cast::<f64>())
    }
}

impl MIsometry<f64>
{
    pub fn to_f32(self) -> MIsometry<f32>
    {
        MIsometry(self.0.cast::<f32>())
    }
}

impl MVector<f32>
{
    pub fn to_f64(self) -> MVector<f64>
    {
        MVector(self.0.cast::<f64>())
    }
}

impl MVector<f64>
{
    pub fn to_f32(self) -> MVector<f32>
    {
        MVector(self.0.cast::<f32>())
    }
}

impl<N: RealField + Copy> MIsometry<N>
{
    #[inline]
    pub fn row(self, i: usize) -> na::Vector4<N>
    {
        self.row(i)
    }
    #[inline]
    pub fn identity() -> Self
    {
        Self(na::Matrix4::identity())
    }
    #[inline]
    pub fn map<N2: Scalar, F: FnMut(N) -> N2>(&self, mut f: F) -> MIsometry<N2>
    {
        MIsometry(self.0.map(f))
    }
    #[inline]
    pub fn from_columns_unchecked(columns: &[MVector<N>;4]) -> Self
    {
        Self(na::Matrix4::from_columns(&(*columns).map(|x| x.0)))
    }
    /// Minkowski transpose. Inverse for hyperbolic isometries
    #[rustfmt::skip]
    pub fn mtranspose(self) -> Self {
        MIsometry(
            na::Matrix4::new(
                self.0.m11,  self.0.m21,  self.0.m31, -self.0.m41,
                self.0.m12,  self.0.m22,  self.0.m32, -self.0.m42,
                self.0.m13,  self.0.m23,  self.0.m33, -self.0.m43,
                -self.0.m14, -self.0.m24, -self.0.m34,  self.0.m44,
            )
        )
    }
    /// Whether an isometry reverses winding with respect to the norm
    pub fn parity(self) -> bool {
        self.0.fixed_view::<3, 3>(0, 0).determinant() < na::zero::<N>()
    }
    pub fn renormalize_isometry(self) -> MIsometry<N>
    {
        let boost = translate(&MVector::origin(), &MVector(self.0.index((.., 3)).into()).lorentz_normalize());
        let inverse_boost = boost.mtranspose();
        let rotation = renormalize_rotation_reflection(
            &((inverse_boost * self).0).fixed_view::<3, 3>(0, 0).clone_owned(),
        );
        MIsometry(boost.0 * rotation.to_homogeneous())
    }
    /*pub fn cast<N2: Scalar>(self) -> MIsometry<N2> where N: simba::scalar::SubsetOf<N2>
    {
        Self(self.0.cast::<N2>())
    }*/
}

impl<N: RealField + Copy> MVector<N> {
    pub fn lorentz_normalize(self: &MVector<N>) -> Self {
        let sf2 = self.mip(self);
        if sf2 == na::zero() {
            return MVector::origin();
        }
        let sf = sf2.abs().sqrt();
        *self / sf
    }
    /// Point or plane reflection around point or normal `p`
    pub fn reflect(self) -> MIsometry<N> {
        MIsometry(na::Matrix4::<N>::identity()
            - self.minkowski_outer_product(&self) * na::convert::<_, N>(2.0) / self.mip(&self))
    }
    /// Minkowski inner product, aka <a, b>_h
    pub fn mip(self, other: &Self) -> N
    {
        self.0.x * other.0.x + self.0.y * other.0.y + self.0.z * other.0.z - self.0.w * other.0.w
    }
    pub fn minkowski_outer_product(self, other: &Self) -> na::Matrix4<N>
    {
        ((self).0) * na::RowVector4::new(other.0.x, other.0.y, other.0.z, -other.0.w)
    }
    pub fn from_gans(gans: &na::Vector3<N>) -> Self
    {
        // x^2 + y^2 + z^2 - w^2 = -1
        // sqrt(x^2 + y^2 + z^2 + 1) = w
        let w = (sqr(gans.x) + sqr(gans.y) + sqr(gans.z) + na::one()).sqrt();
        MVector(na::Vector4::new(gans.x, gans.y, gans.z, w))
    }
    #[inline]
    pub fn zero() -> Self
    {
        Self(na::zero())
    }
    pub fn origin() -> Self
    {
        Self(na::Vector4::new(na::zero(),na::zero(),na::zero(),na::one()))
    }
    #[inline]
    pub fn normalize(self) -> Self
    {
        Self(self.0.normalize())
    }
    #[inline]
    pub fn x() -> Self
    {
        Self(na::Vector4::x())
    }
    #[inline]
    pub fn y() -> Self
    {
        Self(na::Vector4::y())
    }
    #[inline]
    pub fn z() -> Self
    {
        Self(na::Vector4::z())
    }
    #[inline]
    pub fn w() -> Self
    {
        Self(na::Vector4::w())
    }
    #[inline]
    pub fn new(x: N, y: N, z: N, w: N) -> Self
    {
        MVector(na::Vector4::new(x,y,z,w))
    }
    #[inline]
    pub fn xyz(self) -> na::Vector3<N>
    {
        self.0.xyz()
    }
}

impl<N: RealField> Mul<MIsometry<N>> for MIsometry<N>
{
    type Output = MIsometry<N>;
    #[inline]
    fn mul(self, rhs: MIsometry<N>) -> Self::Output
    {
        MIsometry(self.0 * rhs.0)
    }
}

impl<N: RealField> Mul<N> for MVector<N>
{
    type Output = MVector<N>;
    #[inline]
    fn mul(self, rhs: N) -> Self::Output
    {
        MVector(self.0 * rhs)
    }
}

impl<N: RealField> Div<N> for MVector<N>
{
    type Output = MVector<N>;
    #[inline]
    fn div(self, rhs: N) -> Self::Output
    {
        MVector(self.0 / rhs)
    }
}

impl<N: RealField> Add for MVector<N>
{
    type Output = Self;
    #[inline]
    fn add(self, other: Self) -> Self
    {
        Self(self.0 + other.0)
    }
}

impl<N: RealField> Sub for MVector<N>
{
    type Output = Self;
    #[inline]
    fn sub(self, other: Self) -> Self
    {
        Self(self.0 - other.0)
    }
}

impl<N: RealField> Neg for MVector<N>
{
    type Output = Self;
    #[inline]
    fn neg(self) -> Self
    {
        Self(-self.0)
    }
}

impl<N: RealField> Mul<MVector<N>> for MIsometry<N>
{
    type Output = MVector<N>;
    #[inline]
    fn mul(self, rhs: MVector<N>) -> Self::Output
    {
        MVector(self.0 * rhs.0)
    }
}

impl<N: Scalar> MulAssign<N> for MVector<N>
{
    #[inline]
    fn mul_assign(&mut self, rhs: N)
    {
        *self *= rhs;
    }
}

impl<N: Scalar> std::ops::AddAssign for MVector<N>
{
    #[inline]
    fn add_assign(&mut self, other: Self)
    {
        *self += other;
    }
}


impl<N: Scalar> MulAssign<N> for MIsometry<N>
{
    #[inline]
    fn mul_assign(&mut self, rhs: N)
    {
        *self *= rhs;
    }
}

impl<N: Scalar> Index<usize> for MVector<N>
{
    type Output = N;
    #[inline]
    fn index(&self, i: usize) -> &Self::Output
    {
        &self.0[i]
    }
}

impl<N: Scalar> IndexMut<usize> for MVector<N>
{
    #[inline]
    fn index_mut(&mut self, i: usize) -> &mut N
    {
        &mut self.0[i]
    }
}

impl<N: Scalar> Index<(usize,usize)> for MIsometry<N>
{
    type Output = N;
    #[inline]
    fn index(&self, ij: (usize, usize)) -> &Self::Output
    {
        &self.0[ij]
    }
}

/// Transform that translates `a` to `b` given that `a` and `b` are Lorentz normalized pointlike vectors
pub fn translate<N: RealField + Copy>(a: &MVector<N>, b: &MVector<N>) -> MIsometry<N> {
    let a_plus_b = *a + *b;
    MIsometry(na::Matrix4::<N>::identity() - (b.minkowski_outer_product(a) * na::convert::<_, N>(2.0))
        + a_plus_b.minkowski_outer_product(&a_plus_b) / (N::one() - a.mip(b)))
}

/// Transform that translates the origin in the direction of the given vector with distance equal to its magnitude
pub fn translate_along<N: RealField + Copy>(v: &na::Vector3<N>) -> MIsometry<N> {
    let norm = v.norm();
    if norm == na::zero() {
        return MIsometry::identity();
    }
    // g = Lorentz gamma factor
    let g = norm.cosh();
    let bgc = norm.sinhc();
    translate(&MVector::origin(), &MVector(((v * bgc)).insert_row(3, g)))
}

/// 4D reflection around a normal vector; length is not significant (so long as it's nonzero)
pub fn euclidean_reflect<N: RealField + Copy>(v: &na::Vector4<N>) -> na::Matrix4<N> {
    na::Matrix4::identity() - v * v.transpose() * (na::convert::<_, N>(2.0) / v.norm_squared())
}

pub fn midpoint<N: RealField + Copy>(a: &MVector<N>, b: &MVector<N>) -> MVector<N> {
    *a * (b.mip(b) * a.mip(b)).sqrt() + *b * (a.mip(a) * a.mip(b)).sqrt()
}

pub fn distance<N: RealField + Copy>(a: &MVector<N>, b: &MVector<N>) -> N {
    (sqr(a.mip(b)) / (a.mip(a) * b.mip(b))).sqrt().acosh()
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

#[inline]
pub fn sqr<N: RealField + Copy>(x: N) -> N {
    x * x
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
    a: &MVector<N>,
    b: &MVector<N>,
) -> na::Matrix4<N> {
    ((*a).0) * na::RowVector4::new(b.0.x, b.0.y, b.0.z, -b.0.w)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::*;


    #[test]
    #[rustfmt::skip]
    fn reflect_example() {
        assert_abs_diff_eq!(
            MVector::new(0.5, 0.0, 0.0, 1.0).lorentz_normalize().reflect(),
            MIsometry(
                na::Matrix4::new(
                    1.666, 0.0, 0.0, -1.333,
                    0.0  , 1.0, 0.0,  0.0,
                    0.0  , 0.0, 1.0,  0.0,
                    1.333, 0.0, 0.0, -1.666
                )
            ),
            epsilon = 1e-3
        );
    }

    #[test]
    #[rustfmt::skip]
    fn translate_example() {
        assert_abs_diff_eq!(
            translate(
                &MVector::new(-0.5, -0.5, 0.0, 1.0).lorentz_normalize(),
                &MVector::new(0.3, -0.7, 0.0, 1.0).lorentz_normalize()
            ),
            MIsometry(
                na::Matrix4::new(
                    1.676, 0.814, 0.0,  1.572,
                    -1.369, 0.636, 0.0, -1.130,
                    0.0,   0.0,   1.0,  0.0,
                    1.919, 0.257, 0.0,  2.179,
                )
            ),
            epsilon = 1e-3
        );
    }

    #[test]
    fn translate_identity() {
        let a = MVector::new(-0.5, -0.5, 0.0, 1.0).lorentz_normalize();
        let b = MVector::new(0.3, -0.7, 0.0, 1.0).lorentz_normalize();
        let o = MVector::new(0.0, 0.0, 0.0, 1.0);
        assert_abs_diff_eq!(
            translate(&a, &b),
            translate(&o, &a) * translate(&o, &(translate(&a, &o) * b)) * translate(&a, &o),
            epsilon = 1e-5
        );
    }

    #[test]
    fn translate_equivalence() {
        let a = MVector::new(-0.5, -0.5, 0.0, 1.0).lorentz_normalize();
        let o = MVector::new(0.0, 0.0, 0.0, 1.0);
        let direction = a.0.xyz().normalize();
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
        assert_abs_diff_eq!(dx, distance(&MVector::origin(), &(xf * MVector::origin())));
    }

    #[test]
    fn distance_example() {
        let a = MVector::new(0.2, 0.0, 0.0, 1.0);
        let b = MVector::new(-0.5, -0.5, 0.0, 1.0);
        // Paper doubles distances for reasons unknown
        assert_abs_diff_eq!(distance(&a, &b), 2.074 / 2.0, epsilon = 1e-3);
    }

    #[test]
    fn distance_commutative() {
        let p = MVector::from_gans(&na::Vector3::new(-1.0, -1.0, 0.0));
        let q = MVector::from_gans(&na::Vector3::new(1.0, -1.0, 0.0));
        assert_abs_diff_eq!(distance(&p, &q), distance(&q, &p));
    }

    #[test]
    fn midpoint_distance() {
        let p = MVector::from_gans(&na::Vector3::new(-1.0, -1.0, 0.0));
        let q = MVector::from_gans(&na::Vector3::new(1.0, -1.0, 0.0));
        let m = midpoint(&p, &q);
        assert_abs_diff_eq!(distance(&p, &m), distance(&m, &q), epsilon = 1e-5);
        assert_abs_diff_eq!(distance(&p, &m) * 2.0, distance(&p, &q), epsilon = 1e-5);
    }

    #[test]
    fn renormalize_translation() {
        let mat = translate(
            &MVector::new(-0.5, -0.5, 0.0, 1.0).lorentz_normalize(),
            &MVector::new(0.3, -0.7, 0.0, 1.0).lorentz_normalize(),
        );
        assert_abs_diff_eq!(mat.renormalize_isometry(), mat, epsilon = 1e-5);
    }

    #[test]
    #[rustfmt::skip]
    fn renormalize_reflection() {
        let mat = MIsometry(na::Matrix4::new(-1.0, 0.0, 0.0, 0.0,
                                   0.0, 1.0, 0.0, 0.0,
                                   0.0, 0.0, 1.0, 0.0,
                                   0.0, 0.0, 0.0, 1.0));
        assert_abs_diff_eq!(mat.renormalize_isometry(), mat, epsilon = 1e-5);
    }

    #[test]
    #[rustfmt::skip]
    fn renormalize_normalizes_matrix() {
        // Matrix chosen with random entries between -1 and 1
        let error = MIsometry(na::Matrix4::new(
            -0.77, -0.21,  0.57, -0.59,
             0.49, -0.68,  0.36,  0.68,
            -0.75, -0.54, -0.13, -0.59,
            -0.57, -0.80,  0.00, -0.53));

        // translation with some error
        let mat = MIsometry(translate(
            &MVector::new(-0.5, -0.5, 0.0, 1.0),
            &MVector::new(0.3, -0.7, 0.0, 1.0),
        ).0 + error.0 * 0.05);

        let normalized_mat = mat.renormalize_isometry();

        // Check that the matrix is actually normalized
        assert_abs_diff_eq!(
            normalized_mat.mtranspose() * normalized_mat,
            MIsometry(na::Matrix4::identity()),
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
