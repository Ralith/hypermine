//! Tools for processing the geometry of a right dodecahedron

use serde::{Deserialize, Serialize};

use crate::math::{MDirection, MIsometry, MPoint};
use crate::voxel_math::ChunkAxisPermutation;

/// Sides of a right dodecahedron
///
/// These sides are arranged based on the following adjacency graph, although it
/// is recommended not to hardcode side names in other code:
/// ```nocode
///          A
/// (D-) I-E-B-C-D (-I)
///  (K-) L-G-F-H-K (-L)
///           J
/// ```
/// The above adjacency graph can be read as a map of a globe, where side A is
/// at the north pole, and side J is at the south pole.
#[derive(Debug, Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Serialize, Deserialize)]
pub enum Side {
    A,
    B,
    C,
    D,
    E,
    F,
    G,
    H,
    I,
    J,
    K,
    L,
}

// TODO: Remove in favor of using Side::VALUES.len() directly when https://github.com/rust-lang/rust-analyzer/issues/21478 is resolved
pub const SIDE_COUNT: usize = Side::VALUES.len();

impl Side {
    pub const VALUES: [Self; 12] = [
        Self::A,
        Self::B,
        Self::C,
        Self::D,
        Self::E,
        Self::F,
        Self::G,
        Self::H,
        Self::I,
        Self::J,
        Self::K,
        Self::L,
    ];

    pub fn iter() -> impl ExactSizeIterator<Item = Self> {
        Self::VALUES.iter().copied()
    }

    /// Whether `self` and `other` share an edge
    ///
    /// `false` when `self == other`.
    #[inline]
    pub fn adjacent_to(self, other: Side) -> bool {
        data::ADJACENT[self as usize][other as usize]
    }

    /// Outward normal vector of this side
    #[inline]
    pub fn normal(self) -> &'static MDirection<f32> {
        &data::SIDE_NORMALS_F32[self as usize]
    }

    /// Outward normal vector of this side
    #[inline]
    pub fn normal_f64(self) -> &'static MDirection<f64> {
        &data::SIDE_NORMALS_F64[self as usize]
    }

    /// Reflection across this side. Using this matrix is the standard way to
    /// switch between the coordinate systems between adjacent nodes.
    #[inline]
    pub fn reflection(self) -> &'static MIsometry<f32> {
        &data::REFLECTIONS_F32[self as usize]
    }

    /// Reflection across this side. Using this matrix is the standard way to
    /// switch between the coordinate systems between adjacent nodes.
    #[inline]
    pub fn reflection_f64(self) -> &'static MIsometry<f64> {
        &data::REFLECTIONS_F64[self as usize]
    }

    /// Whether `p` is opposite the dodecahedron across the plane containing `self`
    #[inline]
    pub fn is_facing(self, p: &MPoint<f32>) -> bool {
        let r = self.reflection().row(3).clone_owned();
        (r * na::Vector4::from(*p)).x < p.w
    }
}

/// Vertices of a right dodecahedron
///
/// In Hypermine, each dodecahedral node consists of 20 chunks, one for each
/// vertex, shaped like an irregular cube. Each chunk can also be thought of as
/// an eighth of a cube in the dual cubic tiling.
///
/// Each chunk can be given its own coordinate system, where its right-angled
/// corner (or, in other words, the actual vertex of the dodecahedron) is at the
/// origin, and the x, y, and z axes each run alongside an edge of the chunk
/// orthogoal to the respective "canonical" side (See `canonical_sides`).
///
/// Since a vertex can be identified with a chunk, `Vertex` contains methods
/// such as `node_to_dual` and `dual_to_node` that return matrices to allow one
/// to freely swap between the coordinate system for the node and the coordinate
/// system for the chunk.
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash, Serialize, Deserialize)]
pub enum Vertex {
    A,
    B,
    C,
    D,
    E,
    F,
    G,
    H,
    I,
    J,
    K,
    L,
    M,
    N,
    O,
    P,
    Q,
    R,
    S,
    T,
}

// TODO: Remove in favor of using Vertex::VALUES.len() directly when https://github.com/rust-lang/rust-analyzer/issues/21478 is resolved
pub const VERTEX_COUNT: usize = Vertex::VALUES.len();

impl Vertex {
    pub const VALUES: [Self; 20] = [
        Self::A,
        Self::B,
        Self::C,
        Self::D,
        Self::E,
        Self::F,
        Self::G,
        Self::H,
        Self::I,
        Self::J,
        Self::K,
        Self::L,
        Self::M,
        Self::N,
        Self::O,
        Self::P,
        Self::Q,
        Self::R,
        Self::S,
        Self::T,
    ];

    pub fn iter() -> impl ExactSizeIterator<Item = Self> {
        Self::VALUES.iter().copied()
    }

    /// Vertex shared by three sides, if any
    #[inline]
    pub fn from_sides(sides: [Side; 3]) -> Option<Self> {
        data::SIDES_TO_VERTEX[sides[0] as usize][sides[1] as usize][sides[2] as usize]
    }

    /// Sides incident to this vertex, in canonical order.
    ///
    /// This canonical order determines the X, Y, and Z axes of the chunk
    /// corresponding to the vertex.
    #[inline]
    pub fn canonical_sides(self) -> [Side; 3] {
        data::VERTEX_CANONICAL_SIDES[self as usize]
    }

    /// Vertices adjacent to this vertex in canonical order.
    ///
    /// The canonical order of adjacent vertices is based on the canonical order
    /// of sides incident to the vertex, as each of the three adjacent vertices
    /// corresponds to one of the three sides. As for which side, when two
    /// vertices are adjacent, they share two out of three sides of the
    /// dodecahedron. The side they do _not_ share is the side they correspond
    /// to.
    ///
    /// Put another way, anything leaving a chunk in the negative-X direction
    /// will end up crossing `canonical_sides()[0]`, while anything leaving a
    /// chunk in the positive-X direction will end up arriving at
    /// `adjacent_vertices()[0]`.
    #[inline]
    pub fn adjacent_vertices(self) -> [Vertex; 3] {
        data::ADJACENT_VERTICES[self as usize]
    }

    /// Chunk axes permutations for vertices adjacent to this vertex in
    /// canonical order.
    ///
    /// The chunks of two adjacent vertices meet at a plane. When swiching
    /// reference frames from one vertex to another, it is necessary to reflect
    /// about this plane and then apply the permutation returned by this
    /// function.
    #[inline]
    pub fn chunk_axis_permutations(self) -> &'static [ChunkAxisPermutation; 3] {
        &data::CHUNK_AXIS_PERMUTATIONS[self as usize]
    }

    /// For each vertex of the cube dual to this dodecahedral vertex, provides an iterator of at
    /// most 3 steps to reach the corresponding graph node, and binary coordinates of the vertex in
    /// question with respect to the origin vertex of the cube.
    pub fn dual_vertices(
        self,
    ) -> impl ExactSizeIterator<Item = ([bool; 3], impl ExactSizeIterator<Item = Side>)> {
        let [a, b, c] = self.canonical_sides();
        let verts = [
            ([Side::A; 3], 0, [false, false, false]),
            ([c, Side::A, Side::A], 1, [false, false, true]),
            ([b, Side::A, Side::A], 1, [false, true, false]),
            ([b, c, Side::A], 2, [false, true, true]),
            ([a, Side::A, Side::A], 1, [true, false, false]),
            ([a, c, Side::A], 2, [true, false, true]),
            ([a, b, Side::A], 2, [true, true, false]),
            ([a, b, c], 3, [true, true, true]),
        ];
        (0..8).map(move |i| {
            let (sides, len, coords) = verts[i];
            (coords, (0..len).map(move |i| sides[i]))
        })
    }

    /// Transform from euclidean chunk coordinates to hyperbolic node space
    pub fn chunk_to_node(self) -> na::Matrix4<f32> {
        na::Matrix4::from(*self.dual_to_node())
            * na::Matrix4::new_scaling(1.0 / Self::dual_to_chunk_factor())
    }

    /// Transform from euclidean chunk coordinates to hyperbolic node space
    pub fn chunk_to_node_f64(self) -> na::Matrix4<f64> {
        na::Matrix4::from(*self.dual_to_node_f64())
            * na::Matrix4::new_scaling(1.0 / Self::dual_to_chunk_factor_f64())
    }

    /// Transform from hyperbolic node space to euclidean chunk coordinates
    pub fn node_to_chunk(self) -> na::Matrix4<f32> {
        na::Matrix4::new_scaling(Self::dual_to_chunk_factor())
            * na::Matrix4::from(*self.node_to_dual())
    }

    /// Transform from hyperbolic node space to euclidean chunk coordinates
    pub fn node_to_chunk_f64(self) -> na::Matrix4<f64> {
        na::Matrix4::new_scaling(Self::dual_to_chunk_factor_f64())
            * na::Matrix4::from(*self.node_to_dual_f64())
    }

    /// Transform from cube-centric coordinates to dodeca-centric coordinates
    pub fn dual_to_node(self) -> &'static MIsometry<f32> {
        &data::DUAL_TO_NODE_F32[self as usize]
    }

    /// Transform from cube-centric coordinates to dodeca-centric coordinates
    pub fn dual_to_node_f64(self) -> &'static MIsometry<f64> {
        &data::DUAL_TO_NODE_F64[self as usize]
    }

    /// Transform from dodeca-centric coordinates to cube-centric coordinates
    pub fn node_to_dual(self) -> &'static MIsometry<f32> {
        &data::NODE_TO_DUAL_F32[self as usize]
    }

    /// Transform from dodeca-centric coordinates to cube-centric coordinates
    pub fn node_to_dual_f64(self) -> &'static MIsometry<f64> {
        &data::NODE_TO_DUAL_F64[self as usize]
    }

    /// Scale factor used in conversion from cube-centric coordinates to euclidean chunk coordinates.
    /// Scaling the x, y, and z components of a vector in cube-centric coordinates by this value
    /// and dividing them by the w coordinate will yield euclidean chunk coordinates.
    pub fn dual_to_chunk_factor() -> f32 {
        *data::DUAL_TO_CHUNK_FACTOR_F32
    }

    /// Scale factor used in conversion from cube-centric coordinates to euclidean chunk coordinates.
    /// Scaling the x, y, and z components of a vector in cube-centric coordinates by this value
    /// and dividing them by the w coordinate will yield euclidean chunk coordinates.
    pub fn dual_to_chunk_factor_f64() -> f64 {
        *data::DUAL_TO_CHUNK_FACTOR_F64
    }

    /// Scale factor used in conversion from euclidean chunk coordinates to cube-centric coordinates.
    /// Scaling the x, y, and z components of a vector in homogeneous euclidean chunk coordinates by this value
    /// and lorentz-normalizing the result will yield cube-centric coordinates.
    pub fn chunk_to_dual_factor() -> f32 {
        *data::CHUNK_TO_DUAL_FACTOR_F32
    }

    /// Scale factor used in conversion from euclidean chunk coordinates to cube-centric coordinates.
    /// Scaling the x, y, and z components of a vector in homogeneous euclidean chunk coordinates by this value
    /// and lorentz-normalizing the result will yield cube-centric coordinates.
    pub fn chunk_to_dual_factor_f64() -> f64 {
        *data::CHUNK_TO_DUAL_FACTOR_F64
    }

    /// In dodeca-centric coordinates, the center of the smallest sphere that contains the entire
    /// chunk defined by this vertex.
    pub fn chunk_bounding_sphere_center(self) -> &'static MPoint<f32> {
        &data::CHUNK_BOUNDING_SPHERE_CENTERS_F32[self as usize]
    }

    /// In dodeca-centric coordinates, the center of the smallest sphere that contains the entire
    /// chunk defined by this vertex.
    pub fn chunk_bounding_sphere_center_f64(self) -> &'static MPoint<f64> {
        &data::CHUNK_BOUNDING_SPHERE_CENTERS_F64[self as usize]
    }

    /// Convenience method for `self.chunk_to_node().determinant() < 0`.
    pub fn parity(self) -> bool {
        data::CHUNK_TO_NODE_PARITY[self as usize]
    }
}

pub const BOUNDING_SPHERE_RADIUS_F64: f64 = 1.2264568712514068;
pub const BOUNDING_SPHERE_RADIUS: f32 = BOUNDING_SPHERE_RADIUS_F64 as f32;

pub const CHUNK_BOUNDING_SPHERE_RADIUS_F64: f64 = BOUNDING_SPHERE_RADIUS_F64 * 0.5;
pub const CHUNK_BOUNDING_SPHERE_RADIUS: f32 = CHUNK_BOUNDING_SPHERE_RADIUS_F64 as f32;

mod data {
    use std::array;
    use std::sync::LazyLock;

    use crate::dodeca::{SIDE_COUNT, Side, VERTEX_COUNT, Vertex};
    use crate::math::{MDirection, MIsometry, MPoint, MVector, PermuteXYZ};
    use crate::voxel_math::ChunkAxisPermutation;

    /// Whether two sides share an edge
    pub static ADJACENT: LazyLock<[[bool; SIDE_COUNT]; SIDE_COUNT]> = LazyLock::new(|| {
        Side::VALUES.map(|side0| {
            Side::VALUES.map(|side1| {
                // Two sides can have the following values when taking the mip
                // of their normals:
                // - When identical: 1
                // - When adjacent: 0
                // - When two steps away: -1.618 = -phi
                // - When antipodal: -2.618 = -phi - 1
                // Therefore, the range (-0.5..0.5) only contains adjacent sides
                // and is robust to numerical precision limits.
                (-0.5..0.5).contains(&side0.normal_f64().mip(side1.normal_f64()))
            })
        })
    });

    /// Vector corresponding to the outer normal of each side
    pub static SIDE_NORMALS_F64: LazyLock<[MDirection<f64>; SIDE_COUNT]> = LazyLock::new(|| {
        // In Euclidean geometry, the coordinates of a dodecahedron's sides'
        // normals are the same as the coordinates of the vertices of an
        // icosahedron centered at the origin. There is a formula for these
        // vertices' coordinates based on the golden ratio, which we take
        // advantage of here.

        // To set the w-coordinate of these normals, we add an additional
        // constraint: The `mip` of two adjacent normals must be 0 (since this
        // is a right-angled dodechadron). Solving for `w` gives us our
        // `template_normal`. We also make sure to normalize it.

        // All other normals are based on this template normal, with permuations
        // and sign changes.
        let phi = libm::sqrt(1.25) + 0.5; // golden ratio
        let template_normal = MVector::new(1.0, phi, 0.0, libm::sqrt(phi)).normalized_direction();
        let signed_template_normals = {
            let n = template_normal;
            [
                MDirection::new_unchecked(n.x, n.y, n.z, n.w),
                MDirection::new_unchecked(-n.x, n.y, -n.z, n.w),
                MDirection::new_unchecked(n.x, -n.y, -n.z, n.w),
                MDirection::new_unchecked(-n.x, -n.y, n.z, n.w),
            ]
        };

        Side::VALUES.map(|side| {
            let signed_template_normal = signed_template_normals[side as usize / 3];
            signed_template_normal.tuv_to_xyz((3 - side as usize % 3) % 3)
        })
    });

    /// Transform that moves from a neighbor to a reference node, for each side
    pub static REFLECTIONS_F64: LazyLock<[MIsometry<f64>; SIDE_COUNT]> =
        LazyLock::new(|| SIDE_NORMALS_F64.map(|r| MIsometry::reflection(&r)));

    /// Sides incident to a vertex, in canonical order
    pub static VERTEX_CANONICAL_SIDES: LazyLock<[[Side; 3]; VERTEX_COUNT]> = LazyLock::new(|| {
        let mut result: Vec<[Side; 3]> = Vec::new();

        // Rather than trying to work this out mathematically or by hand, we
        // take the brute force approach of checking every unique triplet of
        // vertices, adding a new vertex to the list whenever a new triplet of
        // mutually-adjacent sides is discovered.
        for a in Side::VALUES.iter().copied() {
            for b in Side::VALUES[a as usize + 1..].iter().copied() {
                for c in Side::VALUES[b as usize + 1..].iter().copied() {
                    if !a.adjacent_to(b) || !b.adjacent_to(c) || !c.adjacent_to(a) {
                        continue;
                    }
                    result.push([a, b, c]);
                }
            }
        }

        result.try_into().expect("exactly 20 vertices expected")
    });

    /// Which vertices are adjacent to other vertices and opposite the canonical sides
    pub static ADJACENT_VERTICES: LazyLock<[[Vertex; 3]; VERTEX_COUNT]> = LazyLock::new(|| {
        Vertex::VALUES.map(|vertex| {
            let canonical_sides = vertex.canonical_sides();
            array::from_fn(|canonical_sides_index| {
                // Try every possible side to find an adjacent vertex.
                for test_side in Side::iter() {
                    if test_side == canonical_sides[canonical_sides_index] {
                        continue;
                    }
                    let mut test_sides = canonical_sides;
                    test_sides[canonical_sides_index] = test_side;
                    if let Some(adjacent_vertex) = Vertex::from_sides(test_sides) {
                        return adjacent_vertex;
                    }
                }
                panic!("No suitable vertex found");
            })
        })
    });

    /// Which transformations have to be done after a reflection to switch reference frames from one vertex
    /// to one of its adjacent vertices (ordered similarly to ADJACENT_VERTICES)
    pub static CHUNK_AXIS_PERMUTATIONS: LazyLock<[[ChunkAxisPermutation; 3]; VERTEX_COUNT]> =
        LazyLock::new(|| {
            Vertex::VALUES.map(|vertex| {
                let canonical_sides = vertex.canonical_sides();
                array::from_fn(|canonical_sides_index| {
                    // Try every possible side to find an adjacent vertex.
                    for test_side in Side::iter() {
                        if test_side == canonical_sides[canonical_sides_index] {
                            continue;
                        }
                        let mut test_sides = canonical_sides;
                        test_sides[canonical_sides_index] = test_side;
                        let Some(adjacent_vertex) = Vertex::from_sides(test_sides) else {
                            continue;
                        };
                        // Compare the natural permutation of sides after a reflection from `vertex` to `adjacent_vertex`
                        // to the canonical permutation of the sides for `adjacent_vertex`.
                        return ChunkAxisPermutation::from_permutation(
                            test_sides,
                            adjacent_vertex.canonical_sides(),
                        );
                    }
                    panic!("No suitable vertex found");
                })
            })
        });

    /// Transform that converts from cube-centric coordinates to dodeca-centric coordinates
    pub static DUAL_TO_NODE_F64: LazyLock<[MIsometry<f64>; VERTEX_COUNT]> = LazyLock::new(|| {
        let mip_origin_normal = MVector::origin().mip(Side::A.normal_f64()); // This value is the same for every side
        Vertex::VALUES.map(|vertex| {
            let [a, b, c] = vertex.canonical_sides();

            // The matrix we want to produce is a change-of-basis matrix,
            // consistint of four columns representing vectors with
            // dodeca-centric coordinates, where each vector represents one of
            // the basis vectors in cube-centric coordinates.

            // Since adjacent normals are already orthogonal, we can use them
            // as-is for the first three columns of this matrix. We just need to
            // negate them so that they point towards the origin instead of away
            // because the dodeca's origin has positive cube-centric
            // coordinates.

            // As for the last column of the change-of-basis matrix, that would
            // be the cube-centric origin in dodeca-centric coordinates, or in
            // other words, the vertex's location in dodeca-centric coordinates.
            // To find this, we start at the origin and project the vector to be
            // orthogonal to each of the three normals, one at a time. Because
            // these three normals are orthogonal to each other, the resulting
            // formula is simple.

            // Note that part of the projection formula requires taking the
            // `mip` of a normal vector and the origin, but this is a constant
            // value that doesn't depend on the normal vector, so the formula
            // used here takes advantage of that.
            let vertex_position = (MVector::origin()
                - (a.normal_f64().as_ref() + b.normal_f64().as_ref() + c.normal_f64().as_ref())
                    * mip_origin_normal)
                .normalized_point();
            MIsometry::from_columns_unchecked(
                &[-a.normal_f64(), -b.normal_f64(), -c.normal_f64()],
                vertex_position,
            )
        })
    });

    /// Transform that converts from dodeca-centric coordinates to cube-centric coordinates
    pub static NODE_TO_DUAL_F64: LazyLock<[MIsometry<f64>; VERTEX_COUNT]> =
        LazyLock::new(|| DUAL_TO_NODE_F64.map(|m| m.inverse()));

    pub static DUAL_TO_CHUNK_FACTOR_F64: LazyLock<f64> =
        LazyLock::new(|| (2.0 + 5.0f64.sqrt()).sqrt());

    pub static CHUNK_TO_DUAL_FACTOR_F64: LazyLock<f64> =
        LazyLock::new(|| 1.0 / *DUAL_TO_CHUNK_FACTOR_F64);

    pub static CHUNK_BOUNDING_SPHERE_CENTERS_F64: LazyLock<[MPoint<f64>; VERTEX_COUNT]> =
        LazyLock::new(|| {
            Vertex::VALUES.map(|vertex| {
                // Chunks are most stretched between the origin and the dodeca's vertex, so finding
                // the midpoint of these two extremes allows one to find the bounding sphere.
                // Note that this also means that the bounding sphere radius is half the dodeca's
                // bounding sphere radius.
                (vertex.dual_to_node_f64() * MPoint::origin()).midpoint(&MPoint::origin())
            })
        });

    /// Vertex shared by 3 sides
    pub static SIDES_TO_VERTEX: LazyLock<[[[Option<Vertex>; SIDE_COUNT]; SIDE_COUNT]; SIDE_COUNT]> =
        LazyLock::new(|| {
            let mut result = [[[None; SIDE_COUNT]; SIDE_COUNT]; SIDE_COUNT];
            for vertex in Vertex::iter() {
                let [a, b, c] = vertex.canonical_sides().map(|side| side as usize);
                result[a][b][c] = Some(vertex);
                result[a][c][b] = Some(vertex);
                result[b][a][c] = Some(vertex);
                result[b][c][a] = Some(vertex);
                result[c][a][b] = Some(vertex);
                result[c][b][a] = Some(vertex);
            }
            result
        });

    /// Whether the determinant of the dual-to-node transform is negative
    pub static CHUNK_TO_NODE_PARITY: LazyLock<[bool; VERTEX_COUNT]> =
        LazyLock::new(|| Vertex::VALUES.map(|vertex| vertex.dual_to_node().parity()));

    pub static SIDE_NORMALS_F32: LazyLock<[MDirection<f32>; SIDE_COUNT]> =
        LazyLock::new(|| SIDE_NORMALS_F64.map(|n| n.cast()));

    pub static REFLECTIONS_F32: LazyLock<[MIsometry<f32>; SIDE_COUNT]> =
        LazyLock::new(|| REFLECTIONS_F64.map(|n| n.cast()));

    pub static DUAL_TO_NODE_F32: LazyLock<[MIsometry<f32>; VERTEX_COUNT]> =
        LazyLock::new(|| DUAL_TO_NODE_F64.map(|n| n.cast()));

    pub static NODE_TO_DUAL_F32: LazyLock<[MIsometry<f32>; VERTEX_COUNT]> =
        LazyLock::new(|| NODE_TO_DUAL_F64.map(|n| n.cast()));

    pub static DUAL_TO_CHUNK_FACTOR_F32: LazyLock<f32> =
        LazyLock::new(|| *DUAL_TO_CHUNK_FACTOR_F64 as f32);

    pub static CHUNK_TO_DUAL_FACTOR_F32: LazyLock<f32> =
        LazyLock::new(|| *CHUNK_TO_DUAL_FACTOR_F64 as f32);

    pub static CHUNK_BOUNDING_SPHERE_CENTERS_F32: LazyLock<[MPoint<f32>; VERTEX_COUNT]> =
        LazyLock::new(|| CHUNK_BOUNDING_SPHERE_CENTERS_F64.map(|p| p.cast()));
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::*;

    #[test]
    fn vertex_sides_consistent() {
        use std::collections::HashSet;
        let triples = Vertex::iter()
            .map(|v| v.canonical_sides())
            .collect::<HashSet<_>>();
        assert_eq!(triples.len(), VERTEX_COUNT);
        for triple in Vertex::iter().map(|v| v.canonical_sides()) {
            let mut sorted = triple;
            sorted.sort_unstable();
            assert_eq!(triple, sorted);
            assert!(triple[0].adjacent_to(triple[1]));
            assert!(triple[1].adjacent_to(triple[2]));
            assert!(triple[2].adjacent_to(triple[0]));
        }
    }

    #[test]
    fn sides_to_vertex() {
        for v in Vertex::iter() {
            let [a, b, c] = v.canonical_sides();
            assert_eq!(v, Vertex::from_sides([a, b, c]).unwrap());
            assert_eq!(v, Vertex::from_sides([a, c, b]).unwrap());
            assert_eq!(v, Vertex::from_sides([b, a, c]).unwrap());
            assert_eq!(v, Vertex::from_sides([b, c, a]).unwrap());
            assert_eq!(v, Vertex::from_sides([c, a, b]).unwrap());
            assert_eq!(v, Vertex::from_sides([c, b, a]).unwrap());
        }
    }

    #[test]
    fn adjacent_chunk_axis_permutations() {
        // Assumptions for this test to be valid. If any assertions in this section fail, the test itself
        // needs to be modified
        assert_eq!(Vertex::A.canonical_sides(), [Side::A, Side::B, Side::C]);
        assert_eq!(Vertex::B.canonical_sides(), [Side::A, Side::B, Side::E]);

        assert_eq!(Vertex::F.canonical_sides(), [Side::B, Side::C, Side::F]);
        assert_eq!(Vertex::J.canonical_sides(), [Side::C, Side::F, Side::H]);

        // Test cases

        // Variables with name vertex_?_canonical_sides_reflected refer to the canonical sides
        // of a particular vertex after a reflection that moves it to another vertex.
        // For instance, vertex_a_canonical_sides_reflected is similar to Vertex::A.canonical_sides(),
        // but one of the sides is changed to match Vertex B, but the order of the other two sides is left alone.
        let vertex_a_canonical_sides_reflected = [Side::A, Side::B, Side::E];
        let vertex_b_canonical_sides_reflected = [Side::A, Side::B, Side::C];
        assert_eq!(
            Vertex::A.chunk_axis_permutations()[2],
            ChunkAxisPermutation::from_permutation(
                vertex_a_canonical_sides_reflected,
                Vertex::B.canonical_sides()
            )
        );
        assert_eq!(
            Vertex::B.chunk_axis_permutations()[2],
            ChunkAxisPermutation::from_permutation(
                vertex_b_canonical_sides_reflected,
                Vertex::A.canonical_sides()
            )
        );

        let vertex_f_canonical_sides_reflected = [Side::H, Side::C, Side::F];
        let vertex_j_canonical_sides_reflected = [Side::C, Side::F, Side::B];
        assert_eq!(
            Vertex::F.chunk_axis_permutations()[0],
            ChunkAxisPermutation::from_permutation(
                vertex_f_canonical_sides_reflected,
                Vertex::J.canonical_sides()
            )
        );
        assert_eq!(
            Vertex::J.chunk_axis_permutations()[2],
            ChunkAxisPermutation::from_permutation(
                vertex_j_canonical_sides_reflected,
                Vertex::F.canonical_sides()
            )
        );
    }

    #[test]
    fn side_is_facing() {
        for side in Side::iter() {
            assert!(!side.is_facing(&MPoint::origin()));
            assert!(side.is_facing(&(*side.reflection() * MPoint::origin())));
        }
    }

    #[test]
    fn radius() {
        let corner = *Vertex::A.dual_to_node_f64() * MPoint::origin();
        assert_abs_diff_eq!(
            BOUNDING_SPHERE_RADIUS_F64,
            corner.distance(&MPoint::origin()),
            epsilon = 1e-10
        );
        let phi = (1.0 + 5.0f64.sqrt()) / 2.0; // Golden ratio
        assert_abs_diff_eq!(
            BOUNDING_SPHERE_RADIUS_F64,
            (1.5 * phi).sqrt().asinh(),
            epsilon = 1e-10
        );
    }

    #[test]
    fn chunk_bounding_sphere() {
        let corner = *Vertex::A.dual_to_node_f64() * MPoint::origin();
        let bounding_sphere_center = Vertex::A.chunk_bounding_sphere_center_f64();
        assert_abs_diff_eq!(
            CHUNK_BOUNDING_SPHERE_RADIUS_F64,
            corner.distance(bounding_sphere_center),
            epsilon = 1e-10
        );
        assert_abs_diff_eq!(
            CHUNK_BOUNDING_SPHERE_RADIUS_F64,
            MPoint::origin().distance(bounding_sphere_center),
            epsilon = 1e-10
        );
    }

    #[test]
    fn chunk_to_node() {
        // Chunk coordinates of (1, 1, 1) should be at the center of a dodecahedron.
        let mut chunk_corner_in_node_coordinates =
            Vertex::A.chunk_to_node_f64() * na::Vector4::new(1.0, 1.0, 1.0, 1.0);
        chunk_corner_in_node_coordinates /= chunk_corner_in_node_coordinates.w;
        assert_abs_diff_eq!(
            chunk_corner_in_node_coordinates,
            na::Vector4::new(0.0, 0.0, 0.0, 1.0),
            epsilon = 1e-10
        );
    }

    #[test]
    fn node_to_chunk() {
        assert_abs_diff_eq!(
            Vertex::A.chunk_to_node_f64().try_inverse().unwrap(),
            Vertex::A.node_to_chunk_f64(),
            epsilon = 1e-10
        );
    }
}
