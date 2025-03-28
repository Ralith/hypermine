//! This module defines the a vector and matrix type for use in Minkowski space,
//! allowing natural operations to be performed in hyperbolic space. To better
//! understand the math involved, it is recommended to read "Visualizing
//! Hyperbolic Space: Unusual Uses of 4x4 Matrices." Phillips, Gunn.
//!
//! This module also defines a few other free helper functions that do not
//! directly relate to hyperbolic space.

use na::{RealField, Scalar};
use serde::{Deserialize, Serialize};
use simba::scalar::SupersetOf;

/// A stack-allocated 4-dimensional column-vector in Minkowski space. Such
/// vectors are useful for computations in the hyperboloid model of hyperbolic
/// space. Note that the last coordinate, not the first coordinate, is treated
/// as the special "time" coordinate.
///
/// This vector type is versatile, being able to represent multiple things in
/// Hyperbolic space. What it can represent is generally determined by the
/// Minkowski inner product between the vector and itself.
/// - If it's negative, it represents a point in hyperbolic space. The origin is
///   represented with the unit w-vector.
/// - If it's zero, it represents an _ideal_ point in hyperbolic space. Such a
///   point can be associated with horospheres
/// - If it's positive, it represents an _ultraideal_ point in hyperbolic space.
///   Such points can be treated as oriented planes.
///
/// If the absolute value of this Minkowski inner product is 1, it is
/// normalized, and equations involving such a vector tend to be simpler, much
/// like with unit vectors. Two types, `MPoint` and `MDirection`, are available
/// to facilitate the use of such vectors (for Minkowski inner product -1 and 1,
/// respectively).
///
/// Note that the simplest way to represent directions/velocities/normals at a
/// point in hyperbolic space is with a vector whose Minkowski inner product
/// with that point is 0. Such a vector will be tangent to the hyperboloid model
/// at that associated point, so it can naturally represent movement along the
/// hyperboloid in that direction.
///
/// As a general rule, when working with such vectors, it is highly recommended
/// to avoid dot products and related operations such as vector magnitude, as
/// these operations are meaningless in Minkowski space and are not preserved by
/// isometries.
#[derive(Debug, Copy, Clone, Serialize, Deserialize, PartialEq)]
#[repr(transparent)]
pub struct MVector<N: Scalar>(na::Vector4<N>);

impl<N: RealField + Copy> MVector<N> {
    /// Normalizes the vector so that the Minkowski inner product between the
    /// vector and itself is -1. It should be called on vectors with a negative
    /// self-mip, generally representing points.
    ///
    /// Note that this function is numerically unstable for vectors representing
    /// points far from the origin, so it is recommended to avoid this function
    /// for such vectors.
    pub fn normalized_point(&self) -> MPoint<N> {
        let scale_factor_squared = -self.mip(self);
        if scale_factor_squared <= na::zero() {
            debug_assert!(
                false,
                "Tried to normalize a non-point-like vector as a point."
            );
            return MPoint::origin();
        }
        let scale_factor = scale_factor_squared.sqrt();
        MPoint(*self / scale_factor)
    }

    /// Normalizes the vector so that the Minkowski inner product between the
    /// vector and itself is 1. It should be called on vectors with a positive
    /// self-mip, generally representing directions.
    ///
    /// Note that this function is numerically unstable for vectors representing
    /// directions from points far from the origin, so it is recommended to
    /// avoid this function for such vectors.
    pub fn normalized_direction(&self) -> MDirection<N> {
        let scale_factor_squared = self.mip(self);
        if scale_factor_squared <= na::zero() {
            debug_assert!(
                false,
                "Tried to normalize a non-direction-like vector as a direction."
            );
            return MDirection::x();
        }
        let scale_factor = scale_factor_squared.sqrt();
        MDirection(*self / scale_factor)
    }

    /// Minkowski inner product, aka `<a, b>_h`. This is much like the dot
    /// product, but the product of the w-components is negated. This is the
    /// main operation that distinguishes Minkowski space from Euclidean
    /// 4-space.
    pub fn mip(&self, other: &impl AsRef<MVector<N>>) -> N {
        let other = other.as_ref();
        self.x * other.x + self.y * other.y + self.z * other.z - self.w * other.w
    }

    /// The Minkowski-space equivalent of the outer product of two vectors. This
    /// produces a rank-one matrix that is a useful intermediate result when
    /// computing other matrices, such as reflection or translation matrices.
    fn minkowski_outer_product(self, other: &Self) -> na::Matrix4<N> {
        self.0 * na::RowVector4::new(other.x, other.y, other.z, -other.w)
    }

    /// Cast the components of `self` to another type.
    #[inline]
    pub fn cast<N2: RealField + Copy + SupersetOf<N>>(self) -> MVector<N2> {
        MVector(self.0.cast())
    }

    /// The column vector with components `[0, 0, 0, 0]`.
    #[inline]
    pub fn zero() -> Self {
        Self(na::zero())
    }

    /// The vector representing the origin in hyperbolic space. Alias for `MVector::w()`.
    #[inline]
    pub fn origin() -> Self {
        Self::w()
    }

    /// The column vector with components `[1, 0, 0, 0]`.
    #[inline]
    pub fn x() -> Self {
        Self(na::Vector4::x())
    }

    /// The column vector with components `[0, 1, 0, 0]`.
    #[inline]
    pub fn y() -> Self {
        Self(na::Vector4::y())
    }

    /// The column vector with components `[0, 0, 1, 0]`.
    #[inline]
    pub fn z() -> Self {
        Self(na::Vector4::z())
    }

    /// The column vector with components `[0, 0, 0, 1]`.
    #[inline]
    pub fn w() -> Self {
        Self(na::Vector4::w())
    }

    /// Creates an `MVector` with the given components.
    #[inline]
    pub fn new(x: N, y: N, z: N, w: N) -> Self {
        MVector(na::Vector4::new(x, y, z, w))
    }

    /// The first three coordinates of the vector. When working with an
    /// `MVector` representing a velocity/direction from the origin, the
    /// w-coordinate should always be 0, so using this function to extract a 3D
    /// vector can help make that assumption more explicit.
    #[inline]
    pub fn xyz(self) -> na::Vector3<N> {
        self.0.xyz()
    }
}

impl<N: Scalar> std::ops::Index<usize> for MVector<N> {
    type Output = N;
    #[inline]
    fn index(&self, i: usize) -> &Self::Output {
        &self.0[i]
    }
}

impl<N: Scalar> std::ops::IndexMut<usize> for MVector<N> {
    #[inline]
    fn index_mut(&mut self, i: usize) -> &mut Self::Output {
        &mut self.0[i]
    }
}

impl<N: Scalar> From<na::Vector4<N>> for MVector<N> {
    /// Reinterprets the input as a vector in Minkowski space.
    fn from(value: na::Vector4<N>) -> Self {
        Self(value)
    }
}

impl<N: Scalar> From<MVector<N>> for na::Vector4<N> {
    /// Unwraps the underlying vector. This effectively reinterprets the vector
    /// as a vector in Euclidean 4-space, or, if interpreted as homogeneous
    /// coordinates, a point within the 3D Beltrami-Klein model (as long as it's
    /// inside the unit ball).
    fn from(value: MVector<N>) -> na::Vector4<N> {
        value.0
    }
}

impl<N: Scalar> std::ops::Deref for MVector<N> {
    type Target = na::coordinates::XYZW<N>;
    #[inline]
    fn deref(&self) -> &Self::Target {
        self.0.deref()
    }
}

impl<N: Scalar> std::ops::DerefMut for MVector<N> {
    #[inline]
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.0.deref_mut()
    }
}

impl<N: Scalar> AsRef<MVector<N>> for MVector<N> {
    #[inline]
    fn as_ref(&self) -> &MVector<N> {
        self
    }
}

impl<N: RealField> std::ops::Add<MVector<N>> for MVector<N> {
    type Output = MVector<N>;
    #[inline]
    fn add(self, other: MVector<N>) -> Self::Output {
        MVector(self.0 + other.0)
    }
}

impl<N: RealField> std::ops::Add<&MVector<N>> for MVector<N> {
    type Output = MVector<N>;
    #[inline]
    fn add(self, other: &MVector<N>) -> Self::Output {
        MVector(self.0 + &other.0)
    }
}

impl<N: RealField> std::ops::Add<MVector<N>> for &MVector<N> {
    type Output = MVector<N>;
    #[inline]
    fn add(self, other: MVector<N>) -> Self::Output {
        MVector(&self.0 + other.0)
    }
}

impl<N: RealField> std::ops::Add<&MVector<N>> for &MVector<N> {
    type Output = MVector<N>;
    #[inline]
    fn add(self, other: &MVector<N>) -> Self::Output {
        MVector(&self.0 + &other.0)
    }
}

impl<N: RealField> std::ops::AddAssign<MVector<N>> for MVector<N> {
    #[inline]
    fn add_assign(&mut self, other: MVector<N>) {
        self.0 += other.0;
    }
}

impl<N: RealField> std::ops::AddAssign<&MVector<N>> for MVector<N> {
    #[inline]
    fn add_assign(&mut self, other: &MVector<N>) {
        self.0 += &other.0;
    }
}

impl<N: RealField> std::ops::Sub<MVector<N>> for MVector<N> {
    type Output = MVector<N>;
    #[inline]
    fn sub(self, other: MVector<N>) -> Self::Output {
        MVector(self.0 - other.0)
    }
}

impl<N: RealField> std::ops::Sub<&MVector<N>> for MVector<N> {
    type Output = MVector<N>;
    #[inline]
    fn sub(self, other: &MVector<N>) -> Self::Output {
        MVector(self.0 - &other.0)
    }
}

impl<N: RealField> std::ops::Sub<MVector<N>> for &MVector<N> {
    type Output = MVector<N>;
    #[inline]
    fn sub(self, other: MVector<N>) -> Self::Output {
        MVector(&self.0 - other.0)
    }
}

impl<N: RealField> std::ops::Sub<&MVector<N>> for &MVector<N> {
    type Output = MVector<N>;
    #[inline]
    fn sub(self, other: &MVector<N>) -> Self::Output {
        MVector(&self.0 - &other.0)
    }
}

impl<N: RealField> std::ops::SubAssign<MVector<N>> for MVector<N> {
    #[inline]
    fn sub_assign(&mut self, other: MVector<N>) {
        self.0 -= other.0;
    }
}

impl<N: RealField> std::ops::SubAssign<&MVector<N>> for MVector<N> {
    #[inline]
    fn sub_assign(&mut self, other: &MVector<N>) {
        self.0 -= &other.0;
    }
}

impl<N: RealField> std::ops::Neg for MVector<N> {
    type Output = MVector<N>;
    #[inline]
    fn neg(self) -> Self::Output {
        MVector(-self.0)
    }
}

impl<N: RealField> std::ops::Neg for &MVector<N> {
    type Output = MVector<N>;
    #[inline]
    fn neg(self) -> Self::Output {
        MVector(-&self.0)
    }
}

impl<N: RealField> std::ops::Mul<N> for MVector<N> {
    type Output = MVector<N>;
    #[inline]
    fn mul(self, rhs: N) -> Self::Output {
        MVector(self.0 * rhs)
    }
}

impl<N: RealField> std::ops::Mul<N> for &MVector<N> {
    type Output = MVector<N>;
    #[inline]
    fn mul(self, rhs: N) -> Self::Output {
        MVector(&self.0 * rhs)
    }
}

impl<N: RealField> std::ops::MulAssign<N> for MVector<N> {
    #[inline]
    fn mul_assign(&mut self, rhs: N) {
        self.0 *= rhs;
    }
}

impl<N: RealField> std::ops::Div<N> for MVector<N> {
    type Output = MVector<N>;
    #[inline]
    fn div(self, rhs: N) -> Self::Output {
        MVector(self.0 / rhs)
    }
}

impl<N: RealField> std::ops::Div<N> for &MVector<N> {
    type Output = MVector<N>;
    #[inline]
    fn div(self, rhs: N) -> Self::Output {
        MVector(&self.0 / rhs)
    }
}

impl<N: RealField> std::ops::DivAssign<N> for MVector<N> {
    #[inline]
    fn div_assign(&mut self, rhs: N) {
        self.0 /= rhs;
    }
}

/// An `MVector` with the constraint that the Minkowski inner product between
/// the vector and itself is -1. Such a vector can be used to represent a point
/// in hyperbolic space.
#[derive(Debug, Copy, Clone, Serialize, Deserialize, PartialEq)]
#[repr(transparent)]
pub struct MPoint<N: Scalar>(MVector<N>);

impl<N: RealField + Copy> MPoint<N> {
    /// Returns the midpoint between this vector and the given vector.
    pub fn midpoint(&self, other: &Self) -> MPoint<N> {
        // The midpoint in the hyperboloid model is simply the midpoint in the
        // underlying Euclidean 4-space normalized to land on the hyperboloid.
        (self.as_ref() + other.as_ref()).normalized_point()
    }

    /// Returns the distance between the this vector and the given vector.
    pub fn distance(&self, other: &Self) -> N {
        // The absolute value of the mip between two normalized point-like is
        // the cosh of their distance in hyperbolic space. This is analogous to
        // the fact that the dot product between two unit vectors is the cos of
        // their angle (or distance in spherical geometry).
        (-self.mip(other)).acosh()
    }

    /// Minkowski inner product, aka `<a, b>_h`. This is much like the dot
    /// product, but the product of the w-components is negated. This is the
    /// main operation that distinguishes Minkowski space from Euclidean
    /// 4-space.
    #[inline]
    pub fn mip(&self, other: &impl AsRef<MVector<N>>) -> N {
        self.as_ref().mip(other)
    }

    /// The vector representing the origin in hyperbolic space. Alias for `MVector::w()`.
    #[inline]
    pub fn origin() -> Self {
        Self::w()
    }

    /// The column vector with components `[0, 0, 0, 1]`.
    #[inline]
    pub fn w() -> Self {
        Self(MVector::w())
    }

    /// Creates an `MPoint` with the given components. It is the caller's
    /// responsibility to ensure that the `MPoint` invariant holds.
    #[inline]
    pub fn new_unchecked(x: N, y: N, z: N, w: N) -> Self {
        Self(MVector::new(x, y, z, w))
    }

    /// Cast the components of `self` to another type.
    #[inline]
    pub fn cast<N2: RealField + Copy + SupersetOf<N>>(self) -> MPoint<N2> {
        MPoint(self.0.cast())
    }
}

impl<N: Scalar> From<MPoint<N>> for MVector<N> {
    /// Removes the constraint that makes the argument an `MPoint`
    fn from(value: MPoint<N>) -> MVector<N> {
        value.0
    }
}

impl<N: Scalar> From<MPoint<N>> for na::Vector4<N> {
    /// Unwraps the underlying vector. This effectively reinterprets the vector
    /// as a vector in Euclidean 4-space, or, if interpreted as homogeneous
    /// coordinates, a point within the 3D Beltrami-Klein model (as long as it's
    /// inside the unit ball).
    fn from(value: MPoint<N>) -> na::Vector4<N> {
        value.0.0
    }
}

impl<N: Scalar> std::ops::Deref for MPoint<N> {
    type Target = na::coordinates::XYZW<N>;
    #[inline]
    fn deref(&self) -> &Self::Target {
        self.0.deref()
    }
}

impl<N: Scalar> AsRef<MVector<N>> for MPoint<N> {
    /// Unwraps the `MPoint` into its underlying `MVector`
    #[inline]
    fn as_ref(&self) -> &MVector<N> {
        &self.0
    }
}

/// An `MVector` with the constraint that the Minkowski inner product between
/// the vector and itself is 1. Such a vector can be used to represent a
/// direction in hyperbolic space.
#[derive(Debug, Copy, Clone, Serialize, Deserialize, PartialEq)]
#[repr(transparent)]
pub struct MDirection<N: Scalar>(MVector<N>);

impl<N: RealField + Copy> MDirection<N> {
    /// The column vector with components `[1, 0, 0, 0]`.
    #[inline]
    pub fn x() -> Self {
        Self(MVector::x())
    }

    /// The column vector with components `[0, 1, 0, 0]`.
    #[inline]
    pub fn y() -> Self {
        Self(MVector::y())
    }

    /// The column vector with components `[0, 0, 1, 0]`.
    #[inline]
    pub fn z() -> Self {
        Self(MVector::z())
    }

    /// Minkowski inner product, aka `<a, b>_h`. This is much like the dot
    /// product, but the product of the w-components is negated. This is the
    /// main operation that distinguishes Minkowski space from Euclidean
    /// 4-space.
    #[inline]
    pub fn mip(&self, other: &impl AsRef<MVector<N>>) -> N {
        self.as_ref().mip(other)
    }

    /// Creates an `MDirection with the given components. It is the caller's
    /// responsibility to ensure that the `MDirection` invariant holds.
    #[inline]
    pub fn new_unchecked(x: N, y: N, z: N, w: N) -> Self {
        Self(MVector::new(x, y, z, w))
    }

    /// Cast the components of `self` to another type.
    #[inline]
    pub fn cast<N2: RealField + Copy + SupersetOf<N>>(self) -> MDirection<N2> {
        MDirection(self.0.cast())
    }
}

impl<N: Scalar> From<MDirection<N>> for na::Vector4<N> {
    /// Unwraps the underlying vector. This effectively reinterprets the vector
    /// as a vector in Euclidean 4-space, or, if interpreted as homogeneous
    /// coordinates, a point within the 3D Beltrami-Klein model (as long as it's
    /// inside the unit ball).
    fn from(value: MDirection<N>) -> na::Vector4<N> {
        value.0.0
    }
}

impl<N: Scalar> From<MDirection<N>> for MVector<N> {
    /// Removes the constraint that makes the argument an `MDirection`
    fn from(value: MDirection<N>) -> MVector<N> {
        value.0
    }
}

impl<N: RealField + Copy> From<na::UnitVector3<N>> for MDirection<N> {
    /// Reinterprets the input as a vector in Minkowski space.
    fn from(value: na::UnitVector3<N>) -> Self {
        MDirection(MVector(value.to_homogeneous()))
    }
}

impl<N: Scalar> std::ops::Deref for MDirection<N> {
    type Target = na::coordinates::XYZW<N>;
    #[inline]
    fn deref(&self) -> &Self::Target {
        self.0.deref()
    }
}

impl<N: Scalar> AsRef<MVector<N>> for MDirection<N> {
    /// Unwraps the `MPoint` into its underlying `MVector`
    #[inline]
    fn as_ref(&self) -> &MVector<N> {
        &self.0
    }
}

impl<N: RealField> std::ops::Neg for MDirection<N> {
    type Output = MDirection<N>;
    #[inline]
    fn neg(self) -> Self::Output {
        MDirection(-self.0)
    }
}

impl<N: RealField> std::ops::Neg for &MDirection<N> {
    type Output = MDirection<N>;
    #[inline]
    fn neg(self) -> Self::Output {
        MDirection(-&self.0)
    }
}

/// A stack-allocated, column-major, 4x4 square matrix in Minkowski space that
/// preserves the Minkowski inner product. Such matrices are useful for
/// computations in the hyperboloid model of hyperbolic space. Note that the
/// last coordinate, not the first coordinate, is treated as the special "time"
/// coordinate.
///
/// To ensure that this matrix indeed represents an isometry in Minkowski space,
/// a few invariants are preserved:
/// - The Minkowski inner product between any two distinct columns is 0.
/// - The Minkowski inner product of a column with itself is 1 for the first
///   three columns, and -1 for the last column.
#[derive(Debug, Copy, Clone, Serialize, Deserialize, PartialEq)]
#[repr(transparent)]
pub struct MIsometry<N: Scalar>(na::Matrix4<N>);

impl<N: RealField + Copy> MIsometry<N> {
    /// Returns a view containing the i-th row of this matrix.
    #[inline]
    pub fn row(&self, i: usize) -> na::MatrixView1x4<'_, N, na::U1, na::U4> {
        self.0.row(i)
    }

    /// Creates an identity matrix.
    #[inline]
    pub fn identity() -> Self {
        Self(na::Matrix4::identity())
    }

    /// The reflection about the hyperbolic plane represented by the given
    /// vector.
    pub fn reflection(normal: &MDirection<N>) -> Self {
        // The formula below is the equivalent of the formula for the
        // Householder matrix, but the minkowski outer product instead of the
        // standard outer product to ensure that the reflection is done in
        // Minkowski space. The resulting formula is
        // `I - 2vv*`
        Self(
            na::Matrix4::<N>::identity()
                - normal.as_ref().minkowski_outer_product(normal.as_ref())
                    * na::convert::<_, N>(2.0),
        )
    }

    /// The matrix that translates `a` to `b`.
    pub fn translation(a: &MPoint<N>, b: &MPoint<N>) -> MIsometry<N> {
        // A translation in hyperbolic space can be split into two
        // point-reflections (reflections about a point, where the midpoint of
        // the start and end points is that point)
        // - A reflection about the point `a`
        // - A reflection about the midpoint between `a` and `b`
        // One can convince oneself of this by seeing where `a` goes and noting
        // that no points will leave the line between `a` and `b`.

        // The following notes below will use "*" as the adjoint operator,
        // working in Minkowski space. All multiplication will be implied
        // multiplication to avoid ambiguity with this operator. The matrix for
        // a point-reflection can be derived in a similar manner to a
        // Householder matrix and ends up being the same with a negated term:
        // `-I - 2vv*`
        // The midpoint of `a` and `b` is `a+b` normalized:
        // `(a+b) / sqrt(-(a+b)*(a+b))`
        // which simplifies to
        // `(a+b) / sqrt(-(a*a + a*b + b*a + b*b))`
        // `(a+b) / sqrt(-(a*a + 2a*b + b*b))`
        // `(a+b) / sqrt(-(-1 + 2a*b + -1))`
        // `(a+b) / sqrt(2-2a*b)`

        // Therefore, the derivation of the translation formula is as follows:
        // `reflect_about((a+b) / -(a*b)) * reflect_about(a)`
        // `(-I - 2((a+b) / sqrt(2-2a*b))((a+b) / sqrt(2-2a*b))*) (-I - 2aa*)`
        // `(-I - 2(a+b)(a+b)*/(2-2a*b)) (-I - 2aa*)`
        // `(-I - (a+b)(a+b)*/(1-a*b)) (-I - 2aa*)`
        // `I + (a+b)(a+b)*/(1-a*b) + 2aa* + 2(a+b)(a+b)*aa*/(1-a*b)`
        // Using `(1-a*b) = (-a*a-a*b) = -a*(a+b) = -(a+b)*a`
        // `I + (a+b)(a+b)*/(1-a*b) + 2aa* - 2(a+b)((a+b)*a)a*/((a+b)*a)`
        // `I + (a+b)(a+b)*/(1-a*b) + 2aa* - 2(a+b)a*`
        // `I + (a+b)(a+b)*/(1-a*b) + 2aa* - 2aa* - 2ba*`
        // `I - 2ba* + (a+b)(a+b)*/(1-a*b)`
        let a_plus_b = a.as_ref() + b.as_ref();
        Self(
            na::Matrix4::<N>::identity()
                - b.as_ref().minkowski_outer_product(a.as_ref()) * na::convert::<_, N>(2.0)
                + a_plus_b.minkowski_outer_product(&a_plus_b) / (N::one() - a.mip(b)),
        )
    }

    /// The matrix that translates the origin in the direction of the given
    /// vector with distance equal to its magnitude
    pub fn translation_along(v: &na::Vector3<N>) -> MIsometry<N> {
        // Translating x units along the x-axis takes the origin to `[sinh(x), 0, 0, cosh(x)]`.
        // This is analogous to a rotation of `[0, 0, 1, 1]` along the xw-plane being `[sin(theta), 0, 0, cos(theta)]`.

        // To find a general translation given this principle, we know that the
        // origin moves to a location fitting the following constraints:
        // - The first three coordinates must be in the same direction as `v`
        // - The magnitude of the vector representing the first three coordinates is `sinh(||v||)`
        // - The fourth component is `cosh(||v||)`

        // Once we know where the origin goes, we use the `MIsometry::translation` function.
        let norm = v.norm();
        if norm == na::zero() {
            return MIsometry::identity();
        }
        // `sinhc(x)` simply means `sinh(x)/x` but defined when `x` is 0. Using sinhc combines
        // the normalization of `v` with its multiplication by `sinh(||v||)`.
        MIsometry::translation(
            &MPoint::origin(),
            &MPoint(MVector((v * norm.sinhc()).insert_row(3, norm.cosh()))),
        )
    }

    /// Creates an `MIsometry` with the given columns. It is the caller's
    /// responsibility to ensure that the resulting matrix is a valid isometry
    /// by ensuring that columns are mutually orthogonal.
    #[inline]
    pub fn from_columns_unchecked(
        direction_columns: &[MDirection<N>; 3],
        point_column: MPoint<N>,
    ) -> Self {
        Self(na::Matrix4::from_columns(&[
            direction_columns[0].0.0,
            direction_columns[1].0.0,
            direction_columns[2].0.0,
            point_column.0.0,
        ]))
    }

    /// Creates an `MIsometry` with its elements filled with the components
    /// provided by a slice in column-major order. It is the caller's
    /// responsibility to ensure that the resulting matrix is a valid isometry.
    #[inline]
    pub fn from_column_slice_unchecked(data: &[N]) -> Self {
        Self(na::Matrix4::from_column_slice(data))
    }

    /// Inverts the matrix. Note that this is an efficient operation because the
    /// matrix is an isometry in Minkowski space. The operation actually
    /// performed resembles a matrix transpose, but with some terms negated.
    ///
    /// Mathematically, this operation performed is the Hermitian adjoint, where
    /// the inner product used is the Minkowski inner product.
    #[rustfmt::skip]
    pub fn inverse(&self) -> Self {
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
    pub fn parity(&self) -> bool {
        self.0.fixed_view::<3, 3>(0, 0).determinant() < na::zero::<N>()
    }

    /// Corrects for any drift that may have occurred in the matrix entries due
    /// to rounding that would violate the isometry constraints of the matrix.
    /// If many operations are performed on a single matrix, it is represented
    /// to call this function to correct for this drift.
    ///
    /// Note that this function is numerically unstable for transformations that
    /// have a large translation component, so it is recommended to avoid this
    /// function for such matrices.
    pub fn renormalized(&self) -> MIsometry<N> {
        // There are multiple ways this matrix can be renormalized. This
        // approach splits the translation and orientation components of the
        // hyperbolic isometry, renormalized them both, and recombines them.

        // Since the last column of the matrix is where the origin gets
        // translated, we extract the normalized translation component by
        // recreating a hyperbolic translation matrix using that column.
        let normalized_translation_component = MIsometry::translation(
            &MPoint::origin(),
            &MVector(self.0.column(3).into()).normalized_point(),
        );

        // Once we have the translation component, we use that component's
        // inverse to remove the translation from the original matrix to extract
        // the orientation component.
        let orientation_component = normalized_translation_component.inverse() * self;

        // Then, we use the QR decomposition to convert the orientation
        // component into an orthogonal matrix, which renormalizes it.
        let normalized_orientation_component = MIsometry(
            na::QR::new(
                (orientation_component.0)
                    .fixed_view::<3, 3>(0, 0)
                    .clone_owned(),
            )
            .q()
            .to_homogeneous(),
        );

        // Finally, we recombine the newly-renormalized translation and
        // orientation components.
        normalized_translation_component * normalized_orientation_component
    }

    /// Cast the components of `self` to another type.
    #[inline]
    pub fn cast<N2: RealField + Copy + SupersetOf<N>>(self) -> MIsometry<N2> {
        MIsometry(self.0.cast())
    }
}

impl<N: Scalar> std::ops::Index<(usize, usize)> for MIsometry<N> {
    type Output = N;
    #[inline]
    fn index(&self, ij: (usize, usize)) -> &Self::Output {
        &self.0[ij]
    }
}

impl<N: RealField + Copy> From<na::UnitQuaternion<N>> for MIsometry<N> {
    /// Converts a quaternion into the matrix for the rotation it represents.
    fn from(value: na::UnitQuaternion<N>) -> Self {
        MIsometry(value.to_homogeneous())
    }
}

impl<N: RealField + Copy> From<na::Rotation3<N>> for MIsometry<N> {
    /// Converts a rotation into the matrix representing that rotation.
    fn from(value: na::Rotation3<N>) -> Self {
        MIsometry(value.to_homogeneous())
    }
}

impl<N: Scalar> From<MIsometry<N>> for na::Matrix4<N> {
    /// Unwraps the underlying matrix. This effectively reinterprets the matrix
    /// as a matrix in Euclidean 4-space, or, if interpreted as homogeneous
    /// coordinates, a transformation within the 3D Beltrami-Klein model.
    fn from(value: MIsometry<N>) -> na::Matrix4<N> {
        value.0
    }
}

impl<N: Scalar> std::ops::Deref for MIsometry<N> {
    type Target = na::coordinates::M4x4<N>;
    #[inline]
    fn deref(&self) -> &Self::Target {
        self.0.deref()
    }
}

impl<N: RealField> AsRef<[[N; 4]; 4]> for MIsometry<N> {
    #[inline]
    fn as_ref(&self) -> &[[N; 4]; 4] {
        self.0.as_ref()
    }
}

impl<N: RealField> std::ops::Mul<MIsometry<N>> for MIsometry<N> {
    type Output = MIsometry<N>;
    #[inline]
    fn mul(self, rhs: MIsometry<N>) -> Self::Output {
        MIsometry(self.0 * rhs.0)
    }
}

impl<N: RealField> std::ops::Mul<&MIsometry<N>> for MIsometry<N> {
    type Output = MIsometry<N>;
    #[inline]
    fn mul(self, rhs: &MIsometry<N>) -> Self::Output {
        MIsometry(self.0 * &rhs.0)
    }
}

impl<N: RealField> std::ops::Mul<MIsometry<N>> for &MIsometry<N> {
    type Output = MIsometry<N>;
    #[inline]
    fn mul(self, rhs: MIsometry<N>) -> Self::Output {
        MIsometry(&self.0 * rhs.0)
    }
}

impl<N: RealField> std::ops::Mul<&MIsometry<N>> for &MIsometry<N> {
    type Output = MIsometry<N>;
    #[inline]
    fn mul(self, rhs: &MIsometry<N>) -> Self::Output {
        MIsometry(&self.0 * &rhs.0)
    }
}

impl<N: RealField> std::ops::MulAssign<MIsometry<N>> for MIsometry<N> {
    #[inline]
    fn mul_assign(&mut self, rhs: MIsometry<N>) {
        self.0 *= rhs.0;
    }
}

impl<N: RealField> std::ops::MulAssign<&MIsometry<N>> for MIsometry<N> {
    #[inline]
    fn mul_assign(&mut self, rhs: &MIsometry<N>) {
        self.0 *= &rhs.0;
    }
}

impl<N: RealField> std::ops::Mul<MVector<N>> for MIsometry<N> {
    type Output = MVector<N>;
    #[inline]
    fn mul(self, rhs: MVector<N>) -> Self::Output {
        MVector(self.0 * rhs.0)
    }
}

impl<N: RealField> std::ops::Mul<&MVector<N>> for MIsometry<N> {
    type Output = MVector<N>;
    #[inline]
    fn mul(self, rhs: &MVector<N>) -> Self::Output {
        MVector(self.0 * &rhs.0)
    }
}

impl<N: RealField> std::ops::Mul<MVector<N>> for &MIsometry<N> {
    type Output = MVector<N>;
    #[inline]
    fn mul(self, rhs: MVector<N>) -> Self::Output {
        MVector(&self.0 * rhs.0)
    }
}

impl<N: RealField> std::ops::Mul<&MVector<N>> for &MIsometry<N> {
    type Output = MVector<N>;
    #[inline]
    fn mul(self, rhs: &MVector<N>) -> Self::Output {
        MVector(&self.0 * &rhs.0)
    }
}

impl<N: RealField> std::ops::Mul<MPoint<N>> for MIsometry<N> {
    type Output = MPoint<N>;
    #[inline]
    fn mul(self, rhs: MPoint<N>) -> Self::Output {
        MPoint(self * rhs.0)
    }
}

impl<N: RealField> std::ops::Mul<&MPoint<N>> for MIsometry<N> {
    type Output = MPoint<N>;
    #[inline]
    fn mul(self, rhs: &MPoint<N>) -> Self::Output {
        MPoint(self * &rhs.0)
    }
}

impl<N: RealField> std::ops::Mul<MPoint<N>> for &MIsometry<N> {
    type Output = MPoint<N>;
    #[inline]
    fn mul(self, rhs: MPoint<N>) -> Self::Output {
        MPoint(self * rhs.0)
    }
}

impl<N: RealField> std::ops::Mul<&MPoint<N>> for &MIsometry<N> {
    type Output = MPoint<N>;
    #[inline]
    fn mul(self, rhs: &MPoint<N>) -> Self::Output {
        MPoint(self * &rhs.0)
    }
}

impl<N: RealField> std::ops::Mul<MDirection<N>> for MIsometry<N> {
    type Output = MDirection<N>;
    #[inline]
    fn mul(self, rhs: MDirection<N>) -> Self::Output {
        MDirection(self * rhs.0)
    }
}

impl<N: RealField> std::ops::Mul<&MDirection<N>> for MIsometry<N> {
    type Output = MDirection<N>;
    #[inline]
    fn mul(self, rhs: &MDirection<N>) -> Self::Output {
        MDirection(self * &rhs.0)
    }
}

impl<N: RealField> std::ops::Mul<MDirection<N>> for &MIsometry<N> {
    type Output = MDirection<N>;
    #[inline]
    fn mul(self, rhs: MDirection<N>) -> Self::Output {
        MDirection(self * rhs.0)
    }
}

impl<N: RealField> std::ops::Mul<&MDirection<N>> for &MIsometry<N> {
    type Output = MDirection<N>;
    #[inline]
    fn mul(self, rhs: &MDirection<N>) -> Self::Output {
        MDirection(self * &rhs.0)
    }
}

/// Multiplies the argument by itself.
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

pub trait PermuteXYZ {
    /// Converts from t-u-v coordinates to x-y-z coordinates. t-u-v coordinates
    /// are a permuted version of x-y-z coordinates. `t_axis` determines which
    /// of the three x-y-z coordinates corresponds to the t-coordinate. This
    /// function works with any indexable entity with at least three entries.
    /// Any entry after the third entry is ignored. As an extra guarantee, this
    /// function only performs even permutations.
    ///
    /// Examples:
    /// ```
    /// # use common::math::PermuteXYZ;
    /// assert_eq!([2, 4, 6].tuv_to_xyz(0), [2, 4, 6]);
    /// assert_eq!([2, 4, 6].tuv_to_xyz(1), [6, 2, 4]);
    /// assert_eq!([2, 4, 6].tuv_to_xyz(2), [4, 6, 2]);
    /// assert_eq!([2, 4, 6, 8].tuv_to_xyz(1), [6, 2, 4, 8]);
    /// ```
    fn tuv_to_xyz(self, t_axis: usize) -> Self;
}

impl<T: std::ops::IndexMut<usize, Output = N>, N: Copy> PermuteXYZ for T {
    fn tuv_to_xyz(mut self, t_axis: usize) -> Self {
        (self[t_axis], self[(t_axis + 1) % 3], self[(t_axis + 2) % 3]) =
            (self[0], self[1], self[2]);
        self
    }
}

impl<N: Scalar + Copy> PermuteXYZ for MPoint<N> {
    fn tuv_to_xyz(self, t_axis: usize) -> Self {
        MPoint(self.0.tuv_to_xyz(t_axis))
    }
}

impl<N: Scalar + Copy> PermuteXYZ for MDirection<N> {
    fn tuv_to_xyz(self, t_axis: usize) -> Self {
        MDirection(self.0.tuv_to_xyz(t_axis))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::*;

    impl<N: RealField> AbsDiffEq<MIsometry<N>> for MIsometry<N> {
        type Epsilon = N;

        #[inline]
        fn default_epsilon() -> Self::Epsilon {
            na::Matrix4::<N>::default_epsilon()
        }

        #[inline]
        fn abs_diff_eq(&self, other: &Self, epsilon: Self::Epsilon) -> bool {
            self.0.abs_diff_eq(&other.0, epsilon)
        }
    }

    impl<N: RealField> AbsDiffEq<MVector<N>> for MVector<N> {
        type Epsilon = N;

        #[inline]
        fn default_epsilon() -> Self::Epsilon {
            na::Vector4::<N>::default_epsilon()
        }

        #[inline]
        fn abs_diff_eq(&self, other: &Self, epsilon: Self::Epsilon) -> bool {
            self.0.abs_diff_eq(&other.0, epsilon)
        }
    }

    #[test]
    #[rustfmt::skip]
    fn reflect_example() {
        assert_abs_diff_eq!(
            MIsometry::reflection(&MVector::new(1.0, 0.0, 0.0, 0.5).normalized_direction()),
            MIsometry(
                na::Matrix4::new(
                    -1.666, 0.0, 0.0, 1.333,
                     0.0  , 1.0, 0.0, 0.0,
                     0.0  , 0.0, 1.0, 0.0,
                    -1.333, 0.0, 0.0, 1.666
                )
            ),
            epsilon = 1e-3
        );
    }

    #[test]
    #[rustfmt::skip]
    fn translate_example() {
        assert_abs_diff_eq!(
            MIsometry::translation(
                &MVector::new(-0.5, -0.5, 0.0, 1.0).normalized_point(),
                &MVector::new(0.3, -0.7, 0.0, 1.0).normalized_point()
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
        let a = MVector::new(-0.5, -0.5, 0.0, 1.0).normalized_point();
        let b = MVector::new(0.3, -0.7, 0.0, 1.0).normalized_point();
        let o = MVector::new(0.0, 0.0, 0.0, 1.0).normalized_point();
        assert_abs_diff_eq!(
            MIsometry::translation(&a, &b),
            MIsometry::translation(&o, &a)
                * MIsometry::translation(&o, &(MIsometry::translation(&a, &o) * b))
                * MIsometry::translation(&a, &o),
            epsilon = 1e-5
        );
    }

    #[test]
    fn translate_equivalence() {
        let a = MVector::new(-0.5, -0.5, 0.0, 1.0).normalized_point();
        let o = MVector::new(0.0, 0.0, 0.0, 1.0).normalized_point();
        let direction = a.0.xyz().normalize();
        let distance = dbg!(o.distance(&a));
        assert_abs_diff_eq!(
            MIsometry::translation(&o, &a),
            MIsometry::translation_along(&(direction * distance)),
            epsilon = 1e-5
        );
    }

    #[test]
    fn translate_distance() {
        let dx = 2.3;
        let xf = MIsometry::translation_along(&(na::Vector3::x() * dx));
        assert_abs_diff_eq!(dx, MPoint::origin().distance(&(xf * MPoint::origin())));
    }

    #[test]
    fn distance_example() {
        let a = MVector::new(0.2, 0.0, 0.0, 1.0).normalized_point();
        let b = MVector::new(-0.5, -0.5, 0.0, 1.0).normalized_point();
        // Paper doubles distances for reasons unknown
        assert_abs_diff_eq!(a.distance(&b), 2.074 / 2.0, epsilon = 1e-3);
    }

    #[test]
    fn distance_commutative() {
        let p = MPoint::new_unchecked(-1.0, -1.0, 0.0, 3.0f32.sqrt());
        let q = MPoint::new_unchecked(1.0, -1.0, 0.0, 3.0f32.sqrt());
        assert_abs_diff_eq!(p.distance(&q), q.distance(&p));
    }

    #[test]
    fn midpoint_distance() {
        let p = MPoint::new_unchecked(-1.0, -1.0, 0.0, 3.0f32.sqrt());
        let q = MPoint::new_unchecked(1.0, -1.0, 0.0, 3.0f32.sqrt());
        let m = p.midpoint(&q);
        assert_abs_diff_eq!(p.distance(&m), m.distance(&q), epsilon = 1e-5);
        assert_abs_diff_eq!(p.distance(&m) * 2.0, p.distance(&q), epsilon = 1e-5);
    }

    #[test]
    fn renormalize_translation() {
        let mat = MIsometry::translation(
            &MVector::new(-0.5, -0.5, 0.0, 1.0).normalized_point(),
            &MVector::new(0.3, -0.7, 0.0, 1.0).normalized_point(),
        );
        assert_abs_diff_eq!(mat.renormalized(), mat, epsilon = 1e-5);
    }

    #[test]
    #[rustfmt::skip]
    fn renormalize_reflection() {
        let mat = MIsometry(na::Matrix4::new(
            -1.0, 0.0, 0.0, 0.0,
             0.0, 1.0, 0.0, 0.0,
             0.0, 0.0, 1.0, 0.0,
             0.0, 0.0, 0.0, 1.0));
        assert_abs_diff_eq!(mat.renormalized(), mat, epsilon = 1e-5);
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
        let mat = MIsometry(MIsometry::translation(
            &MVector::new(-0.5, -0.5, 0.0, 1.0).normalized_point(),
            &MVector::new(0.3, -0.7, 0.0, 1.0).normalized_point(),
        ).0 + error.0 * 0.05);

        let normalized_mat = mat.renormalized();

        // Check that the matrix is actually normalized
        assert_abs_diff_eq!(
            normalized_mat.inverse() * normalized_mat,
            MIsometry::identity(),
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
}
