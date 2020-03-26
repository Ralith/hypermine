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

    /// Reflection across this side
    #[inline]
    pub fn reflection(self) -> &'static na::Matrix4<f64> {
        &REFLECTIONS[self as usize]
    }

    /// Whether `p` is opposite the dodecahedron across the plane containing `self`
    #[inline]
    pub fn faces<N: na::RealField>(self, p: &na::Vector4<N>) -> bool {
        let r = na::convert::<_, na::RowVector4<N>>(self.reflection().row(3).clone_owned());
        (r * p).x < p.w
    }
}

/// Vertices of a right dodecahedron
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
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

    /// Returns the transform which places this cube in its canonical position
    /// relative to the node it's associated with.
    pub fn cube_to_node(self) -> na::Matrix4<f64> {
        let origin = na::Vector4::new(0.0, 0.0, 0.0, 1.0);
        let [a, b, c] = self.canonical_sides();
        na::Matrix4::from_columns(&[
            a.reflection().column(3) - origin,
            b.reflection().column(3) - origin,
            c.reflection().column(3) - origin,
            origin,
        ])
    }
}

pub const VERTEX_COUNT: usize = 20;
pub const SIDE_COUNT: usize = 12;

lazy_static! {
    /// Whether two sides share an edge
    static ref ADJACENT: [[bool; SIDE_COUNT]; SIDE_COUNT] = {
        let mut result = [[false; SIDE_COUNT]; SIDE_COUNT];
        for i in 0..SIDE_COUNT {
            for j in 0..SIDE_COUNT {
                let cosh_distance = (REFLECTIONS[i] * REFLECTIONS[j])[(3, 3)];
                // Possile cosh_distances: 1, 4.23606 = 2+sqrt(5), 9.47213 = 5+2*sqrt(5), 12.70820 = 6+3*sqrt(5);
                // < 2.0 indicates identical faces; < 5.0 indicates adjacent faces; > 5.0 indicates non-adjacent faces
                result[i][j] = cosh_distance >= 2.0 && cosh_distance < 5.0;
            }
        }
        result
    };

    /// Transform that moves from a neighbor to a reference node, for each side
    static ref REFLECTIONS: [na::Matrix4<f64>; SIDE_COUNT] = {
        let phi = 1.25f64.sqrt() + 0.5; // golden ratio
        let root_phi = phi.sqrt();
        let f = math::lorentz_normalize(&na::Vector4::new(root_phi, phi * root_phi, 0.0, phi + 2.0));

        let mut result = [na::zero(); SIDE_COUNT];
        let mut i = 0;
        for (x, y, z, w) in [
            (f.x, f.y, f.z, f.w),
            (-f.x, f.y, -f.z, f.w),
            (f.x, -f.y, -f.z, f.w),
            (-f.x, -f.y, f.z, f.w),
        ]
            .iter()
            .cloned()
        {
            for (x, y, z, w) in [(x, y, z, w), (y, z, x, w), (z, x, y, w)].iter().cloned() {
                result[i] = math::translate(&math::origin(), &na::Vector4::new(x, y, z, w))
                    * math::euclidean_reflect(&na::Vector4::new(x, y, z, 0.0))
                    * math::translate(&math::origin(), &na::Vector4::new(-x, -y, -z, w));
                i += 1;
            }
        }
        result
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
}

#[cfg(test)]
mod tests {
    use super::*;

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
    fn side_faces() {
        for side in Side::iter() {
            assert!(!side.faces::<f32>(&math::origin()));
            assert!(side.faces(&(side.reflection() * math::origin())));
        }
    }
}
