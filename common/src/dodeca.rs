//! Tools for processing the geometry of a right dodecahedron

use lazy_static::lazy_static;
use serde::{Deserialize, Serialize};

use crate::math;

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
        [A, B, C, D, E, F, G, H, I, J, K, L].iter().cloned()
    }

    /// Whether `self` and `other` share an edge
    ///
    /// `false` when `self == other`.
    #[inline]
    pub fn adjacent_to(self, other: Side) -> bool {
        ADJACENT[self as usize][other as usize]
    }

    /// Outward normal vector of this side
    #[inline]
    pub fn normal(self) -> &'static na::Vector4<f64> {
        &SIDE_NORMALS[self as usize]
    }

    /// Reflection across this side
    #[inline]
    pub fn reflection(self) -> &'static na::Matrix4<f64> {
        &REFLECTIONS[self as usize]
    }

    /// Whether `p` is opposite the dodecahedron across the plane containing `self`
    #[inline]
    pub fn is_facing<N: na::RealField + Copy>(self, p: &na::Vector4<N>) -> bool {
        let r = na::convert::<_, na::RowVector4<N>>(self.reflection().row(3).clone_owned());
        (r * p).x < p.w
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
        SIDES_TO_VERTEX[a as usize][b as usize][c as usize]
    }

    /// Sides incident to this vertex, in canonical order
    #[inline]
    pub fn canonical_sides(self) -> [Side; 3] {
        VERTEX_SIDES[self as usize]
    }

    /// Vertices adjacent to this vertex, opposite the sides in canonical order
    #[inline]
    pub fn adjacent_vertices(self) -> [Vertex; 3] {
        ADJACENT_VERTICES[self as usize]
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
    pub fn chunk_to_node(self) -> na::Matrix4<f64> {
        self.dual_to_node() * na::Matrix4::new_scaling(1.0 / Self::dual_to_chunk_factor())
    }

    /// Transform from hyperbolic node space to euclidean chunk coordinates
    pub fn node_to_chunk(self) -> na::Matrix4<f64> {
        na::Matrix4::new_scaling(Self::dual_to_chunk_factor()) * self.node_to_dual()
    }

    /// Transform from cube-centric coordinates to dodeca-centric coordinates
    pub fn dual_to_node(self) -> &'static na::Matrix4<f64> {
        &DUAL_TO_NODE[self as usize]
    }

    /// Transform from dodeca-centric coordinates to cube-centric coordinates
    pub fn node_to_dual(self) -> &'static na::Matrix4<f64> {
        &NODE_TO_DUAL[self as usize]
    }

    /// Scale factor used in conversion from cube-centric coordinates to euclidean chunk coordinates.
    /// Scaling the x, y, and z components of a vector in cube-centric coordinates by this value
    /// and dividing them by the w coordinate will yield euclidean chunk coordinates.
    pub fn dual_to_chunk_factor() -> f64 {
        *DUAL_TO_CHUNK_FACTOR
    }

    /// Scale factor used in conversion from euclidean chunk coordinates to cube-centric coordinates.
    /// Scaling the x, y, and z components of a vector in homogeneous euclidean chunk coordinates by this value
    /// and lorentz-normalizing the result will yield cube-centric coordinates.
    pub fn chunk_to_dual_factor() -> f64 {
        *CHUNK_TO_DUAL_FACTOR
    }

    /// Convenience method for `self.chunk_to_node().determinant() < 0`.
    pub fn parity(self) -> bool {
        CHUNK_TO_NODE_PARITY[self as usize]
    }
}

pub const VERTEX_COUNT: usize = 20;
pub const SIDE_COUNT: usize = 12;
pub const BOUNDING_SPHERE_RADIUS: f64 = 1.2264568712514068;

lazy_static! {
    /// Whether two sides share an edge
    static ref ADJACENT: [[bool; SIDE_COUNT]; SIDE_COUNT] = {
        let mut result = [[false; SIDE_COUNT]; SIDE_COUNT];
        for i in 0..SIDE_COUNT {
            for j in 0..SIDE_COUNT {
                let cosh_distance = (REFLECTIONS[i] * REFLECTIONS[j])[(3, 3)];
                // Possile cosh_distances: 1, 4.23606 = 2+sqrt(5), 9.47213 = 5+2*sqrt(5), 12.70820 = 6+3*sqrt(5);
                // < 2.0 indicates identical faces; < 5.0 indicates adjacent faces; > 5.0 indicates non-adjacent faces
                result[i][j] = (2.0..5.0).contains(&cosh_distance);
            }
        }
        result
    };

    /// Vector corresponding to the outer normal of each side
    static ref SIDE_NORMALS: [na::Vector4<f64>; SIDE_COUNT] = {
        let phi = libm::sqrt(1.25) + 0.5; // golden ratio
        let f = math::lorentz_normalize(&na::Vector4::new(1.0, phi, 0.0, libm::sqrt(phi)));

        let mut result: [na::Vector4<f64>; SIDE_COUNT] = [na::zero(); SIDE_COUNT];
        let mut i = 0;
        for (x, y, z, w) in [
            (f.x, f.y, f.z, f.w),
            (-f.x, f.y, -f.z, f.w),
            (f.x, -f.y, -f.z, f.w),
            (-f.x, -f.y, f.z, f.w),
        ]
        {
            for (x, y, z, w) in [(x, y, z, w), (y, z, x, w), (z, x, y, w)] {
                result[i] = na::Vector4::new(x, y, z, w);
                i += 1;
            }
        }
        result
    };

    /// Transform that moves from a neighbor to a reference node, for each side
    static ref REFLECTIONS: [na::Matrix4<f64>; SIDE_COUNT] = {
        SIDE_NORMALS.map(|r| math::reflect(&r))
    };

    /// Sides incident to a vertex, in canonical order
    static ref VERTEX_SIDES: [[Side; 3]; VERTEX_COUNT] = {
        let mut result = [[Side::A; 3]; VERTEX_COUNT];
        let mut vertex = 0;
        // Kind of a hack, but working this out by hand isn't any fun.
        for a in 0..SIDE_COUNT {
            for b in (a+1)..SIDE_COUNT {
                for c in (b+1)..SIDE_COUNT {
                    if !ADJACENT[a][b] || !ADJACENT[b][c] || !ADJACENT[c][a] {
                        continue;
                    }
                    result[vertex] = [Side::from_index(a), Side::from_index(b), Side::from_index(c)];
                    vertex += 1;
                }
            }
        }
        assert_eq!(vertex, 20);
        result
    };

    // Which vertices are adjacent to other vertices and opposite the canonical sides
    static ref ADJACENT_VERTICES: [[Vertex; 3]; VERTEX_COUNT] = {
        let mut result = [[Vertex::A; 3]; VERTEX_COUNT];

        for vertex in 0..VERTEX_COUNT {
            for result_index in 0..3 {
                let mut test_sides = VERTEX_SIDES[vertex];
                // Keep modifying the result_index'th element of test_sides until its three elements are all
                // adjacent to a single vertex. That vertex is the vertex we're looking for.
                for side in Side::iter() {
                    if side == VERTEX_SIDES[vertex][result_index] {
                        continue;
                    }
                    test_sides[result_index] = side;
                    if let Some(adjacent_vertex) = Vertex::from_sides(test_sides[0], test_sides[1], test_sides[2]) {
                        result[vertex][result_index] = adjacent_vertex;
                    }
                }
            }
        }
        result
    };

    /// Transform that converts from cube-centric coordinates to dodeca-centric coordinates
    static ref DUAL_TO_NODE: [na::Matrix4<f64>; VERTEX_COUNT] = {
        let mip_origin_normal = math::mip(&math::origin(), &SIDE_NORMALS[0]); // This value is the same for every side
        let mut result = [na::zero(); VERTEX_COUNT];
        for i in 0..VERTEX_COUNT {
            let [a, b, c] = VERTEX_SIDES[i];
            let vertex_position = math::lorentz_normalize(
                &(math::origin() - (a.normal() + b.normal() + c.normal()) * mip_origin_normal),
            );
            result[i] = na::Matrix4::from_columns(&[-a.normal(), -b.normal(), -c.normal(), vertex_position]);
        }
        result
    };

    /// Transform that converts from dodeca-centric coordinates to cube-centric coordinates
    static ref NODE_TO_DUAL: [na::Matrix4<f64>; VERTEX_COUNT] = {
        DUAL_TO_NODE.map(|m| math::mtranspose(&m))
    };

    static ref DUAL_TO_CHUNK_FACTOR: f64 = (2.0 + 5.0f64.sqrt()).sqrt();
    static ref CHUNK_TO_DUAL_FACTOR: f64 = 1.0 / *DUAL_TO_CHUNK_FACTOR;

    /// Vertex shared by 3 sides
    static ref SIDES_TO_VERTEX: [[[Option<Vertex>; SIDE_COUNT]; SIDE_COUNT]; SIDE_COUNT] = {
        let mut result = [[[None; SIDE_COUNT]; SIDE_COUNT]; SIDE_COUNT];
        let mut vertex = Vertex::iter();
        // Kind of a hack, but working this out by hand isn't any fun.
        for a in 0..SIDE_COUNT {
            for b in (a+1)..SIDE_COUNT {
                for c in (b+1)..SIDE_COUNT {
                    if !Side::from_index(a).adjacent_to(Side::from_index(b)) ||
                        !Side::from_index(b).adjacent_to(Side::from_index(c)) ||
                        !Side::from_index(c).adjacent_to(Side::from_index(a))
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
    };

    /// Whether the determinant of the cube-to-node transform is negative
    static ref CHUNK_TO_NODE_PARITY: [bool; VERTEX_COUNT] = {
        let mut result = [false; VERTEX_COUNT];

        for v in Vertex::iter() {
            result[v as usize] = math::parity(&v.chunk_to_node());
        }

        result
    };
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::*;

    #[test]
    fn vertex_sides() {
        use std::collections::HashSet;
        let triples = VERTEX_SIDES.iter().collect::<HashSet<_>>();
        assert_eq!(triples.len(), VERTEX_COUNT);
        for &triple in &*VERTEX_SIDES {
            let mut sorted = triple;
            sorted.sort_unstable();
            assert_eq!(triple, sorted);
            assert!(ADJACENT[triple[0] as usize][triple[1] as usize]);
            assert!(ADJACENT[triple[1] as usize][triple[2] as usize]);
            assert!(ADJACENT[triple[2] as usize][triple[0] as usize]);
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
    fn side_is_facing() {
        for side in Side::iter() {
            assert!(!side.is_facing::<f32>(&math::origin()));
            assert!(side.is_facing(&(side.reflection() * math::origin())));
        }
    }

    #[test]
    fn radius() {
        let corner = Vertex::A.chunk_to_node() * math::origin();
        assert_abs_diff_eq!(
            BOUNDING_SPHERE_RADIUS,
            math::distance(&corner, &math::origin()),
            epsilon = 1e-10
        );
        let phi = (1.0 + 5.0f64.sqrt()) / 2.0; // Golden ratio
        assert_abs_diff_eq!(
            BOUNDING_SPHERE_RADIUS,
            (1.5 * phi).sqrt().asinh(),
            epsilon = 1e-10
        );
    }

    #[test]
    fn chunk_to_node() {
        // Chunk coordinates of (1, 1, 1) should be at the center of a dodecahedron.
        let mut chunk_corner_in_node_coordinates =
            Vertex::A.chunk_to_node() * na::Vector4::new(1.0, 1.0, 1.0, 1.0);
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
            Vertex::A.chunk_to_node().try_inverse().unwrap(),
            Vertex::A.node_to_chunk(),
            epsilon = 1e-10
        );
    }
}
