//! Tools for processing the geometry of a right dodecahedron

use data::*;
use serde::{Deserialize, Serialize};

<<<<<<< HEAD
use crate::math;
use crate::math::{MVector,MIsometry};
=======
use crate::voxel_math::ChunkAxisPermutation;
>>>>>>> d49df99d371ca6354789deefff6900b1eea46533

/// Sides of a right dodecahedron
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

impl Side {
    #[inline]
    pub fn from_index(x: usize) -> Self {
        use Side::*;
        const VALUES: [Side; SIDE_COUNT] = [A, B, C, D, E, F, G, H, I, J, K, L];
        VALUES[x]
    }

    pub fn iter() -> impl ExactSizeIterator<Item = Self> {
        use Side::*;
        [A, B, C, D, E, F, G, H, I, J, K, L].into_iter()
    }

    /// Whether `self` and `other` share an edge
    ///
    /// `false` when `self == other`.
    #[inline]
    pub fn adjacent_to(self, other: Side) -> bool {
        adjacent()[self as usize][other as usize]
    }

    /// Outward normal vector of this side
    #[inline]
<<<<<<< HEAD
    pub fn normal(self) -> &'static MVector<f64> {
        &SIDE_NORMALS[self as usize]
=======
    pub fn normal(self) -> &'static na::Vector4<f32> {
        &side_normals_f32()[self as usize]
    }

    /// Outward normal vector of this side
    #[inline]
    pub fn normal_f64(self) -> &'static na::Vector4<f64> {
        &side_normals_f64()[self as usize]
>>>>>>> d49df99d371ca6354789deefff6900b1eea46533
    }

    /// Reflection across this side
    #[inline]
<<<<<<< HEAD
    pub fn reflection(self) -> &'static MIsometry<f64> {
        &REFLECTIONS[self as usize]
=======
    pub fn reflection(self) -> &'static na::Matrix4<f32> {
        &reflections_f32()[self as usize]
    }

    /// Reflection across this side
    #[inline]
    pub fn reflection_f64(self) -> &'static na::Matrix4<f64> {
        &reflections_f64()[self as usize]
>>>>>>> d49df99d371ca6354789deefff6900b1eea46533
    }

    /// Whether `p` is opposite the dodecahedron across the plane containing `self`
    #[inline]
<<<<<<< HEAD
    pub fn is_facing<N: na::RealField + Copy>(self, p: &MVector<N>) -> bool {
        let r = na::convert::<_, na::RowVector4<N>>(self.reflection().row(3).clone_owned());
        (r * *p).x < p.w
=======
    pub fn is_facing<N: na::RealField + Copy>(self, p: &na::Vector4<N>) -> bool {
        let r = na::convert::<_, na::RowVector4<N>>(self.reflection_f64().row(3).clone_owned());
        (r * p).x < p.w
>>>>>>> d49df99d371ca6354789deefff6900b1eea46533
    }
}

/// Vertices of a right dodecahedron
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

impl Vertex {
    pub fn iter() -> impl ExactSizeIterator<Item = Self> {
        use Vertex::*;
        [A, B, C, D, E, F, G, H, I, J, K, L, M, N, O, P, Q, R, S, T]
            .iter()
            .cloned()
    }

    /// Vertex shared by three sides, if any
    #[inline]
    pub fn from_sides(a: Side, b: Side, c: Side) -> Option<Self> {
        sides_to_vertex()[a as usize][b as usize][c as usize]
    }

    /// Sides incident to this vertex, in canonical order.
    ///
    /// This canonical order determines the X, Y, and Z axes of the chunk
    /// corresponding to the vertex.
    #[inline]
    pub fn canonical_sides(self) -> [Side; 3] {
        vertex_sides()[self as usize]
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
        adjacent_vertices()[self as usize]
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
        &chunk_axis_permutations()[self as usize]
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
<<<<<<< HEAD
    pub fn chunk_to_node(self) -> na::Matrix4<f64> {
        <MIsometry<f64> as Into<na::Matrix4<f64>>>::into(*self.dual_to_node()) * na::Matrix4::new_scaling(1.0 / Self::dual_to_chunk_factor())
=======
    pub fn chunk_to_node(self) -> na::Matrix4<f32> {
        self.dual_to_node() * na::Matrix4::new_scaling(1.0 / Self::dual_to_chunk_factor())
>>>>>>> d49df99d371ca6354789deefff6900b1eea46533
    }

    /// Transform from euclidean chunk coordinates to hyperbolic node space
    pub fn chunk_to_node_f64(self) -> na::Matrix4<f64> {
        self.dual_to_node_f64() * na::Matrix4::new_scaling(1.0 / Self::dual_to_chunk_factor_f64())
    }

    /// Transform from hyperbolic node space to euclidean chunk coordinates
<<<<<<< HEAD
    pub fn node_to_chunk(self) -> na::Matrix4<f64> {
        na::Matrix4::new_scaling(Self::dual_to_chunk_factor()) * <MIsometry<f64> as Into<na::Matrix4<f64>>>::into(*self.node_to_dual())
=======
    pub fn node_to_chunk(self) -> na::Matrix4<f32> {
        na::Matrix4::new_scaling(Self::dual_to_chunk_factor()) * self.node_to_dual()
>>>>>>> d49df99d371ca6354789deefff6900b1eea46533
    }

    /// Transform from hyperbolic node space to euclidean chunk coordinates
    pub fn node_to_chunk_f64(self) -> na::Matrix4<f64> {
        na::Matrix4::new_scaling(Self::dual_to_chunk_factor_f64()) * self.node_to_dual_f64()
    }

    /// Transform from cube-centric coordinates to dodeca-centric coordinates
<<<<<<< HEAD
    pub fn dual_to_node(self) -> &'static MIsometry<f64> {
        &DUAL_TO_NODE[self as usize]
    }

    /// Transform from dodeca-centric coordinates to cube-centric coordinates
    pub fn node_to_dual(self) -> &'static MIsometry<f64> {
        &NODE_TO_DUAL[self as usize]
=======
    pub fn dual_to_node(self) -> &'static na::Matrix4<f32> {
        &dual_to_node_f32()[self as usize]
    }

    /// Transform from cube-centric coordinates to dodeca-centric coordinates
    pub fn dual_to_node_f64(self) -> &'static na::Matrix4<f64> {
        &dual_to_node_f64()[self as usize]
    }

    /// Transform from dodeca-centric coordinates to cube-centric coordinates
    pub fn node_to_dual(self) -> &'static na::Matrix4<f32> {
        &node_to_dual_f32()[self as usize]
    }

    /// Transform from dodeca-centric coordinates to cube-centric coordinates
    pub fn node_to_dual_f64(self) -> &'static na::Matrix4<f64> {
        &node_to_dual_f64()[self as usize]
>>>>>>> d49df99d371ca6354789deefff6900b1eea46533
    }

    /// Scale factor used in conversion from cube-centric coordinates to euclidean chunk coordinates.
    /// Scaling the x, y, and z components of a vector in cube-centric coordinates by this value
    /// and dividing them by the w coordinate will yield euclidean chunk coordinates.
    pub fn dual_to_chunk_factor() -> f32 {
        dual_to_chunk_factor_f32()
    }

    /// Scale factor used in conversion from cube-centric coordinates to euclidean chunk coordinates.
    /// Scaling the x, y, and z components of a vector in cube-centric coordinates by this value
    /// and dividing them by the w coordinate will yield euclidean chunk coordinates.
    pub fn dual_to_chunk_factor_f64() -> f64 {
        dual_to_chunk_factor_f64()
    }

    /// Scale factor used in conversion from euclidean chunk coordinates to cube-centric coordinates.
    /// Scaling the x, y, and z components of a vector in homogeneous euclidean chunk coordinates by this value
    /// and lorentz-normalizing the result will yield cube-centric coordinates.
    pub fn chunk_to_dual_factor() -> f32 {
        chunk_to_dual_factor_f32()
    }

    /// Scale factor used in conversion from euclidean chunk coordinates to cube-centric coordinates.
    /// Scaling the x, y, and z components of a vector in homogeneous euclidean chunk coordinates by this value
    /// and lorentz-normalizing the result will yield cube-centric coordinates.
    pub fn chunk_to_dual_factor_f64() -> f64 {
        chunk_to_dual_factor_f64()
    }

    /// Convenience method for `self.chunk_to_node().determinant() < 0`.
    pub fn parity(self) -> bool {
<<<<<<< HEAD
        DUAL_TO_NODE_PARITY[self as usize]
=======
        chunk_to_node_parity()[self as usize]
>>>>>>> d49df99d371ca6354789deefff6900b1eea46533
    }
}

pub const VERTEX_COUNT: usize = 20;
pub const SIDE_COUNT: usize = 12;
pub const BOUNDING_SPHERE_RADIUS_F64: f64 = 1.2264568712514068;
pub const BOUNDING_SPHERE_RADIUS: f32 = BOUNDING_SPHERE_RADIUS_F64 as f32;

mod data {
    use std::array;
    use std::sync::OnceLock;

    use crate::dodeca::{Side, Vertex, SIDE_COUNT, VERTEX_COUNT};
    use crate::math;
    use crate::voxel_math::ChunkAxisPermutation;

    /// Whether two sides share an edge
    pub fn adjacent() -> &'static [[bool; SIDE_COUNT]; SIDE_COUNT] {
        static LOCK: OnceLock<[[bool; SIDE_COUNT]; SIDE_COUNT]> = OnceLock::new();
        LOCK.get_or_init(|| {
            let mut result = [[false; SIDE_COUNT]; SIDE_COUNT];
            for (i, side) in result.iter_mut().enumerate() {
                for (j, is_adjacent) in side.iter_mut().enumerate() {
                    let cosh_distance = (reflections_f64()[i] * reflections_f64()[j])[(3, 3)];
                    // Possile cosh_distances: 1, 4.23606 = 2+sqrt(5), 9.47213 = 5+2*sqrt(5), 12.70820 = 6+3*sqrt(5);
                    // < 2.0 indicates identical faces; < 5.0 indicates adjacent faces; > 5.0 indicates non-adjacent faces
                    *is_adjacent = (2.0..5.0).contains(&cosh_distance);
                }
            }
            result
        })
    }

    /// Vector corresponding to the outer normal of each side
<<<<<<< HEAD
    static ref SIDE_NORMALS: [MVector<f64>; SIDE_COUNT] = {
        let phi = libm::sqrt(1.25) + 0.5; // golden ratio
        let f = MVector::new(1.0, phi, 0.0, libm::sqrt(phi)).lorentz_normalize();

        let mut result: [MVector<f64>; SIDE_COUNT] = [MVector::zero(); SIDE_COUNT];
        let mut i = 0;
        for (x, y, z, w) in [
            (f.x, f.y, f.z, f.w),
            (-f.x, f.y, -f.z, f.w),
            (f.x, -f.y, -f.z, f.w),
            (-f.x, -f.y, f.z, f.w),
        ]
        {
            for (x, y, z, w) in [(x, y, z, w), (y, z, x, w), (z, x, y, w)] {
                result[i] = MVector::new(x, y, z, w);
                i += 1;
=======
    pub fn side_normals_f64() -> &'static [na::Vector4<f64>; SIDE_COUNT] {
        static LOCK: OnceLock<[na::Vector4<f64>; SIDE_COUNT]> = OnceLock::new();
        LOCK.get_or_init(|| {
            let phi = libm::sqrt(1.25) + 0.5; // golden ratio
            let f = math::lorentz_normalize(&na::Vector4::new(1.0, phi, 0.0, libm::sqrt(phi)));

            let mut result: [na::Vector4<f64>; SIDE_COUNT] = [na::zero(); SIDE_COUNT];
            let mut i = 0;
            for (x, y, z, w) in [
                (f.x, f.y, f.z, f.w),
                (-f.x, f.y, -f.z, f.w),
                (f.x, -f.y, -f.z, f.w),
                (-f.x, -f.y, f.z, f.w),
            ] {
                for (x, y, z, w) in [(x, y, z, w), (y, z, x, w), (z, x, y, w)] {
                    result[i] = na::Vector4::new(x, y, z, w);
                    i += 1;
                }
>>>>>>> d49df99d371ca6354789deefff6900b1eea46533
            }
            result
        })
    }

    /// Transform that moves from a neighbor to a reference node, for each side
<<<<<<< HEAD
    static ref REFLECTIONS: [MIsometry<f64>; SIDE_COUNT] = {
        SIDE_NORMALS.map(|r| r.reflect())
    };
=======
    pub fn reflections_f64() -> &'static [na::Matrix4<f64>; SIDE_COUNT] {
        static LOCK: OnceLock<[na::Matrix4<f64>; SIDE_COUNT]> = OnceLock::new();
        LOCK.get_or_init(|| side_normals_f64().map(|r| math::reflect(&r)))
    }
>>>>>>> d49df99d371ca6354789deefff6900b1eea46533

    /// Sides incident to a vertex, in canonical order
    pub fn vertex_sides() -> &'static [[Side; 3]; VERTEX_COUNT] {
        static LOCK: OnceLock<[[Side; 3]; VERTEX_COUNT]> = OnceLock::new();
        LOCK.get_or_init(|| {
            let mut result = [[Side::A; 3]; VERTEX_COUNT];
            let mut vertex = 0;
            // Kind of a hack, but working this out by hand isn't any fun.
            for a in 0..SIDE_COUNT {
                for b in (a + 1)..SIDE_COUNT {
                    for c in (b + 1)..SIDE_COUNT {
                        if !adjacent()[a][b] || !adjacent()[b][c] || !adjacent()[c][a] {
                            continue;
                        }
                        result[vertex] = [
                            Side::from_index(a),
                            Side::from_index(b),
                            Side::from_index(c),
                        ];
                        vertex += 1;
                    }
                }
            }
            assert_eq!(vertex, 20);
            result
        })
    }

    // Which vertices are adjacent to other vertices and opposite the canonical sides
    pub fn adjacent_vertices() -> &'static [[Vertex; 3]; VERTEX_COUNT] {
        static LOCK: OnceLock<[[Vertex; 3]; VERTEX_COUNT]> = OnceLock::new();
        LOCK.get_or_init(|| {
            let mut result = [[Vertex::A; 3]; VERTEX_COUNT];

            for (i, triple) in result.iter_mut().enumerate() {
                for result_index in 0..3 {
                    let mut test_sides = vertex_sides()[i];
                    // Keep modifying the result_index'th element of test_sides until its three elements are all
                    // adjacent to a single vertex. That vertex is the vertex we're looking for.
                    for side in Side::iter() {
                        if side == vertex_sides()[i][result_index] {
                            continue;
                        }
                        test_sides[result_index] = side;
                        if let Some(adjacent_vertex) =
                            Vertex::from_sides(test_sides[0], test_sides[1], test_sides[2])
                        {
                            triple[result_index] = adjacent_vertex;
                        }
                    }
                }
            }
            result
        })
    }

    // Which transformations have to be done after a reflection to switch reference frames from one vertex
    // to one of its adjacent vertices (ordered similarly to ADJACENT_VERTICES)
    pub fn chunk_axis_permutations() -> &'static [[ChunkAxisPermutation; 3]; VERTEX_COUNT] {
        static LOCK: OnceLock<[[ChunkAxisPermutation; 3]; VERTEX_COUNT]> = OnceLock::new();
        LOCK.get_or_init(|| {
            array::from_fn(|vertex| {
                array::from_fn(|result_index| {
                    let mut test_sides = vertex_sides()[vertex];
                    // Keep modifying the result_index'th element of test_sides until its three elements are all
                    // adjacent to a single vertex (determined using `Vertex::from_sides`).
                    for side in Side::iter() {
                        if side == vertex_sides()[vertex][result_index] {
                            continue;
                        }
                        test_sides[result_index] = side;
                        let Some(adjacent_vertex) =
                            Vertex::from_sides(test_sides[0], test_sides[1], test_sides[2])
                        else {
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
        })
    }

    /// Transform that converts from cube-centric coordinates to dodeca-centric coordinates
<<<<<<< HEAD
    static ref DUAL_TO_NODE: [MIsometry<f64>; VERTEX_COUNT] = {
        let mip_origin_normal = MVector::origin().mip(&SIDE_NORMALS[0]); // This value is the same for every side
        let mut result = [MIsometry::identity(); VERTEX_COUNT];
        for i in 0..VERTEX_COUNT {
            let [a, b, c] = VERTEX_SIDES[i];
            let vertex_position = (MVector::origin() - (*a.normal() + *b.normal() + *c.normal()) * mip_origin_normal).lorentz_normalize();
            result[i] = MIsometry::from_columns_unchecked(&[-*a.normal(), -*b.normal(), -*c.normal(), vertex_position]);
        }
        result

    };

    /// Transform that converts from dodeca-centric coordinates to cube-centric coordinates
    static ref NODE_TO_DUAL: [MIsometry<f64>; VERTEX_COUNT] = {
        DUAL_TO_NODE.map(|m| m.mtranspose())
    };
=======
    pub fn dual_to_node_f64() -> &'static [na::Matrix4<f64>; VERTEX_COUNT] {
        static LOCK: OnceLock<[na::Matrix4<f64>; VERTEX_COUNT]> = OnceLock::new();
        LOCK.get_or_init(|| {
            let mip_origin_normal = math::mip(&math::origin(), &side_normals_f64()[0]); // This value is the same for every side
            let mut result = [na::zero(); VERTEX_COUNT];
            for (i, map) in result.iter_mut().enumerate() {
                let [a, b, c] = vertex_sides()[i];
                let vertex_position = math::lorentz_normalize(
                    &(math::origin()
                        - (a.normal_f64() + b.normal_f64() + c.normal_f64()) * mip_origin_normal),
                );
                *map = na::Matrix4::from_columns(&[
                    -a.normal_f64(),
                    -b.normal_f64(),
                    -c.normal_f64(),
                    vertex_position,
                ]);
            }
            result
        })
    }

    /// Transform that converts from dodeca-centric coordinates to cube-centric coordinates
    pub fn node_to_dual_f64() -> &'static [na::Matrix4<f64>; VERTEX_COUNT] {
        static LOCK: OnceLock<[na::Matrix4<f64>; VERTEX_COUNT]> = OnceLock::new();
        LOCK.get_or_init(|| dual_to_node_f64().map(|m| math::mtranspose(&m)))
    }
>>>>>>> d49df99d371ca6354789deefff6900b1eea46533

    pub fn dual_to_chunk_factor_f64() -> f64 {
        static LOCK: OnceLock<f64> = OnceLock::new();
        *LOCK.get_or_init(|| (2.0 + 5.0f64.sqrt()).sqrt())
    }

    pub fn chunk_to_dual_factor_f64() -> f64 {
        static LOCK: OnceLock<f64> = OnceLock::new();
        *LOCK.get_or_init(|| 1.0 / dual_to_chunk_factor_f64())
    }

    /// Vertex shared by 3 sides
    pub fn sides_to_vertex() -> &'static [[[Option<Vertex>; SIDE_COUNT]; SIDE_COUNT]; SIDE_COUNT] {
        static LOCK: OnceLock<[[[Option<Vertex>; SIDE_COUNT]; SIDE_COUNT]; SIDE_COUNT]> =
            OnceLock::new();
        LOCK.get_or_init(|| {
            let mut result = [[[None; SIDE_COUNT]; SIDE_COUNT]; SIDE_COUNT];
            let mut vertex = Vertex::iter();
            // Kind of a hack, but working this out by hand isn't any fun.
            for a in 0..SIDE_COUNT {
                for b in (a + 1)..SIDE_COUNT {
                    for c in (b + 1)..SIDE_COUNT {
                        if !Side::from_index(a).adjacent_to(Side::from_index(b))
                            || !Side::from_index(b).adjacent_to(Side::from_index(c))
                            || !Side::from_index(c).adjacent_to(Side::from_index(a))
                        {
                            continue;
                        }
                        let v = Some(vertex.next().unwrap());
                        result[a][b][c] = v;
                        result[a][c][b] = v;
                        result[b][a][c] = v;
                        result[b][c][a] = v;
                        result[c][a][b] = v;
                        result[c][b][a] = v;
                    }
                }
            }
            assert_eq!(vertex.next(), None);
            result
        })
    }

    /// Whether the determinant of the cube-to-node transform is negative
<<<<<<< HEAD
    static ref DUAL_TO_NODE_PARITY: [bool; VERTEX_COUNT] = {
        let mut result = [false; VERTEX_COUNT];

        for v in Vertex::iter() {
            result[v as usize] = v.dual_to_node().parity();
        }
=======
    pub fn chunk_to_node_parity() -> &'static [bool; VERTEX_COUNT] {
        static LOCK: OnceLock<[bool; VERTEX_COUNT]> = OnceLock::new();
        LOCK.get_or_init(|| {
            let mut result = [false; VERTEX_COUNT];

            for v in Vertex::iter() {
                result[v as usize] = math::parity(&v.chunk_to_node_f64());
            }
>>>>>>> d49df99d371ca6354789deefff6900b1eea46533

            result
        })
    }

    pub fn side_normals_f32() -> &'static [na::Vector4<f32>; SIDE_COUNT] {
        static LOCK: OnceLock<[na::Vector4<f32>; SIDE_COUNT]> = OnceLock::new();
        LOCK.get_or_init(|| side_normals_f64().map(|n| n.cast()))
    }

    pub fn reflections_f32() -> &'static [na::Matrix4<f32>; SIDE_COUNT] {
        static LOCK: OnceLock<[na::Matrix4<f32>; SIDE_COUNT]> = OnceLock::new();
        LOCK.get_or_init(|| reflections_f64().map(|n| n.cast()))
    }

    pub fn dual_to_node_f32() -> &'static [na::Matrix4<f32>; VERTEX_COUNT] {
        static LOCK: OnceLock<[na::Matrix4<f32>; VERTEX_COUNT]> = OnceLock::new();
        LOCK.get_or_init(|| dual_to_node_f64().map(|n| n.cast()))
    }

    pub fn node_to_dual_f32() -> &'static [na::Matrix4<f32>; VERTEX_COUNT] {
        static LOCK: OnceLock<[na::Matrix4<f32>; VERTEX_COUNT]> = OnceLock::new();
        LOCK.get_or_init(|| node_to_dual_f64().map(|n| n.cast()))
    }

    pub fn dual_to_chunk_factor_f32() -> f32 {
        static LOCK: OnceLock<f32> = OnceLock::new();
        *LOCK.get_or_init(|| dual_to_chunk_factor_f64() as f32)
    }

    pub fn chunk_to_dual_factor_f32() -> f32 {
        static LOCK: OnceLock<f32> = OnceLock::new();
        *LOCK.get_or_init(|| chunk_to_dual_factor_f64() as f32)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math;
    use approx::*;

    #[test]
    fn vertex_sides_consistent() {
        use std::collections::HashSet;
        let triples = vertex_sides().iter().collect::<HashSet<_>>();
        assert_eq!(triples.len(), VERTEX_COUNT);
        for &triple in vertex_sides() {
            let mut sorted = triple;
            sorted.sort_unstable();
            assert_eq!(triple, sorted);
            assert!(adjacent()[triple[0] as usize][triple[1] as usize]);
            assert!(adjacent()[triple[1] as usize][triple[2] as usize]);
            assert!(adjacent()[triple[2] as usize][triple[0] as usize]);
        }
    }

    #[test]
    fn sides_to_vertex() {
        for v in Vertex::iter() {
            let [a, b, c] = v.canonical_sides();
            assert_eq!(v, Vertex::from_sides(a, b, c).unwrap());
            assert_eq!(v, Vertex::from_sides(a, c, b).unwrap());
            assert_eq!(v, Vertex::from_sides(b, a, c).unwrap());
            assert_eq!(v, Vertex::from_sides(b, c, a).unwrap());
            assert_eq!(v, Vertex::from_sides(c, a, b).unwrap());
            assert_eq!(v, Vertex::from_sides(c, b, a).unwrap());
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
            assert!(!side.is_facing::<f32>(&MVector::origin()));
            assert!(side.is_facing(&(side.reflection() * MVector::origin())));
        }
    }

    #[test]
    fn radius() {
<<<<<<< HEAD
        let corner = Vertex::A.chunk_to_node() * MVector::origin();
        assert_abs_diff_eq!(
            BOUNDING_SPHERE_RADIUS,
            math::distance(&corner, &MVector::origin()),
=======
        let corner = Vertex::A.chunk_to_node_f64() * math::origin();
        assert_abs_diff_eq!(
            BOUNDING_SPHERE_RADIUS_F64,
            math::distance(&corner, &math::origin()),
>>>>>>> d49df99d371ca6354789deefff6900b1eea46533
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
