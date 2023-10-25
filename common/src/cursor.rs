use crate::dodeca::{Side, Vertex, SIDE_COUNT};
use crate::graph::{Graph, NodeId};
use crate::node::ChunkId;

use lazy_static::lazy_static;

/// Navigates the cubic dual of a graph
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct Cursor {
    node: NodeId,
    a: Side,
    b: Side,
    c: Side,
}

impl Cursor {
    /// Construct a canonical cursor for the cube at `vertex` of `node`
    pub fn from_vertex(node: NodeId, vertex: Vertex) -> Self {
        let [a, b, c] = vertex.canonical_sides();
        Self { node, a, b, c }
    }

    /// Get the neighbor towards `dir`
    pub fn step(self, graph: &Graph, dir: Dir) -> Option<Self> {
        // For a cube identified by three dodecahedral faces sharing a vertex, we identify its
        // cubical neighbors by taking each vertex incident to exactly two of the faces and the face
        // of the three it's not incident to, and selecting the cube represented by the new vertex
        // in both the dodecahedron sharing the face unique to the new vertex and that sharing the
        // face that the new vertex isn't incident to.
        let (a, b, c) = (self.a, self.b, self.c);
        let a_prime = NEIGHBORS[a as usize][b as usize][c as usize].unwrap();
        let b_prime = NEIGHBORS[b as usize][a as usize][c as usize].unwrap();
        let c_prime = NEIGHBORS[c as usize][b as usize][a as usize].unwrap();
        use Dir::*;
        let (sides, neighbor) = match dir {
            Left => ((a, b, c_prime), c),
            Right => ((a, b, c_prime), c_prime),
            Down => ((a, b_prime, c), b),
            Up => ((a, b_prime, c), b_prime),
            Forward => ((a_prime, b, c), a),
            Back => ((a_prime, b, c), a_prime),
        };
        let node = graph.neighbor(self.node, neighbor)?;
        Some(Self {
            node,
            a: sides.0,
            b: sides.1,
            c: sides.2,
        })
    }

    /// Node and dodecahedral vertex that contains the representation for this cube in the graph
    pub fn canonicalize(self, graph: &Graph) -> Option<ChunkId> {
        graph.canonicalize(ChunkId::new(
            self.node,
            Vertex::from_sides(self.a, self.b, self.c).unwrap(),
        ))
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum Dir {
    Left,
    Right,
    Down,
    Up,
    Forward,
    Back,
}
impl Dir {
    pub fn iter() -> impl ExactSizeIterator<Item = Self> + Clone {
        use Dir::*;
        [Left, Right, Down, Up, Forward, Back].iter().cloned()
    }

    /// Returns the unit vector corresponding to the direction.
    pub fn vector(self) -> na::Vector3<isize> {
        use Dir::*;
        match self {
            Up => na::Vector3::x(),
            Down => -na::Vector3::x(),
            Left => na::Vector3::y(),
            Right => -na::Vector3::y(),
            Forward => na::Vector3::z(),
            Back => -na::Vector3::z(),
        }
    }
}

/// Returns a direction's opposite direction.
impl std::ops::Neg for Dir {
    type Output = Self;
    fn neg(self) -> Self::Output {
        use Dir::*;
        match self {
            Left => Right,
            Right => Left,
            Down => Up,
            Up => Down,
            Forward => Back,
            Back => Forward,
        }
    }
}

lazy_static! {
    /// Maps every (A, B, C) sharing a vertex to A', the side that shares edges with B and C but not A
    static ref NEIGHBORS: [[[Option<Side>; SIDE_COUNT]; SIDE_COUNT]; SIDE_COUNT] = {
        let mut result = [[[None; SIDE_COUNT]; SIDE_COUNT]; SIDE_COUNT];
        for a in Side::iter() {
            for b in Side::iter() {
                for c in Side::iter() {
                    for s in Side::iter() {
                        if s == a || s == b || s == c {
                            continue;
                        }
                        let (opposite, shared) = match (s.adjacent_to(a), s.adjacent_to(b), s.adjacent_to(c)) {
                            (false, true, true) => (a, (b, c)),
                            (true, false, true) => (b, (a, c)),
                            (true, true, false) => (c, (a, b)),
                            _ => continue,
                        };
                        result[opposite as usize][shared.0 as usize][shared.1 as usize] = Some(s);
                    }
                }
            }
        }
        result
    };
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{proto::Position, traversal::ensure_nearby};

    #[test]
    fn neighbor_sanity() {
        for v in Vertex::iter() {
            let [a, b, c] = v.canonical_sides();
            assert_eq!(
                NEIGHBORS[a as usize][b as usize][c as usize],
                NEIGHBORS[a as usize][c as usize][b as usize]
            );
        }
    }

    #[test]
    fn cursor_identities() {
        let mut graph = Graph::new(1);
        ensure_nearby(&mut graph, &Position::origin(), 3.0);
        let start = Cursor::from_vertex(NodeId::ROOT, Vertex::A);
        let wiggle = |dir| {
            let x = start.step(&graph, dir).unwrap();
            assert!(x != start);
            assert_eq!(x.step(&graph, -dir).unwrap(), start);
        };
        wiggle(Dir::Left);
        wiggle(Dir::Right);
        wiggle(Dir::Down);
        wiggle(Dir::Up);
        wiggle(Dir::Forward);
        wiggle(Dir::Back);

        let vcycle = |dir| {
            // Five steps because an edge in the dual honeycomb has
            // five cubes around itself, not four as in Euclidean space.
            let looped = start
                .step(&graph, dir)
                .expect("positive")
                .step(&graph, Dir::Down)
                .expect("down")
                .step(&graph, -dir)
                .expect("negative")
                .step(&graph, Dir::Up)
                .expect("up")
                .step(&graph, dir)
                .expect("positive");
            assert_eq!(
                looped.canonicalize(&graph).unwrap(),
                ChunkId::new(NodeId::ROOT, Vertex::A),
            );
        };
        vcycle(Dir::Left);
        vcycle(Dir::Right);
        vcycle(Dir::Forward);
        vcycle(Dir::Back);
    }
}
