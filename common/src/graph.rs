use std::convert::TryFrom;
use std::fmt;
use std::num::NonZeroU32;

use fxhash::FxHashSet;
use lazy_static::lazy_static;
use serde::{Deserialize, Serialize};
use tracing::trace;

use crate::math;

/// Graph of the right dodecahedral tiling of H^3
#[derive(Debug, Clone)]
pub struct Graph<T> {
    nodes: Vec<Node<T>>,
    fresh: Vec<NodeId>,
}

impl<T> Graph<T> {
    pub fn new() -> Self {
        Self {
            nodes: vec![Node {
                cubes: Default::default(),
                parent_side: None,
                length: 0,
                neighbors: [None; SIDES],
            }],
            fresh: Vec::new(),
        }
    }

    pub fn len(&self) -> u32 {
        self.nodes.len() as u32
    }

    pub fn get_neighbors(&self, node: NodeId) -> &[Option<NodeId>; SIDES] {
        &self.nodes[node.idx()].neighbors
    }

    /// Look up the ID of a node's neighbor, creating nearby nodes if necessary
    pub fn ensure_neighbor(&mut self, node: NodeId, side: Side) -> NodeId {
        self.ensure_neighbor_inner(node, side, NeighborType::Any)
            .expect("ensuring a neighbor of unconstrained type should always succeed")
    }

    /// Nodes created since the last call to `clear_fresh`
    pub fn fresh(&self) -> &[NodeId] {
        &self.fresh
    }

    pub fn clear_fresh(&mut self) {
        self.fresh.clear();
    }

    /// Compute reflectedness and `start`-relative transforms for all cube-bearing nodes within
    /// `distance` links
    ///
    /// Because we relate nodes to their neighbors with reflection transforms, a flag indicating
    /// whether a given node is reached via an odd number of reflections allows us to render with
    /// correct vertex winding.
    pub fn nearby_cubes(
        &self,
        start: NodeId,
        distance: u32,
    ) -> Vec<(&Option<T>, bool, na::Matrix4<f32>)> {
        struct PendingNode {
            id: NodeId,
            parity: bool,
            distance: u32,
            transform: na::Matrix4<f64>,
        }

        let mut result = Vec::new();
        let mut pending = Vec::<PendingNode>::new();
        let mut visited = FxHashSet::<NodeId>::default();

        pending.push(PendingNode {
            id: start,
            parity: false,
            distance: 0,
            transform: na::Matrix4::identity(),
        });
        visited.insert(start);

        while let Some(current) = pending.pop() {
            let node = &self.nodes[current.id.idx()];
            for v in self.cubes_at(current.id) {
                result.push((
                    &node.cubes[v as usize],
                    current.parity ^ CUBE_TO_NODE_DETERMINANT_NEGATIVE[v as usize],
                    na::convert(current.transform * v.cube_to_node()),
                ));
            }
            if current.distance == distance {
                continue;
            }
            for side in Side::iter() {
                let neighbor = match node.neighbors[side as usize] {
                    None => continue,
                    Some(x) => x,
                };
                if visited.contains(&neighbor) {
                    continue;
                }
                pending.push(PendingNode {
                    id: neighbor,
                    parity: !current.parity,
                    distance: current.distance + 1,
                    transform: current.transform * REFLECTIONS[side as usize],
                });
                visited.insert(neighbor);
            }
        }

        result
    }

    pub fn cubes_at(&self, node: NodeId) -> impl Iterator<Item = Vertex> + '_ {
        let node = &self.nodes[node.idx()];
        Vertex::iter().filter(move |&v| {
            VERTEX_SIDES[v as usize].iter().all(|&side| {
                let neighbor = match node.neighbors[side as usize] {
                    None => return true,
                    Some(x) => x,
                };
                self.nodes[neighbor.idx()].length > node.length
            })
        })
    }

    /// Ensure all nodes within `distance` links of `start` exist
    pub fn ensure_nearby(&mut self, start: NodeId, distance: u32) {
        let mut pending = Vec::<(NodeId, u32)>::new();
        let mut visited = FxHashSet::<NodeId>::default();

        pending.push((start, 0));
        visited.insert(start);

        while let Some((node, current_distance)) = pending.pop() {
            for side in Side::iter() {
                let neighbor = self.ensure_neighbor(node, side);
                if visited.contains(&neighbor) || current_distance + 1 == distance {
                    continue;
                }
                visited.insert(neighbor);
                pending.push((neighbor, current_distance + 1));
            }
        }
        trace!("visited {}, fresh {}", visited.len(), self.fresh.len());
    }

    pub fn get(&self, node: NodeId, cube: Vertex) -> &Option<T> {
        &self.nodes[node.idx()].cubes[cube as usize]
    }

    pub fn get_mut(&mut self, node: NodeId, cube: Vertex) -> &mut Option<T> {
        &mut self.nodes[node.idx()].cubes[cube as usize]
    }

    fn ensure_neighbor_inner(
        &mut self,
        node: NodeId,
        side: Side,
        mode: NeighborType,
    ) -> Option<NodeId> {
        let v = &self.nodes[node.idx()];
        if let Some(x) = v.neighbors[side as usize] {
            // Neighbor already exists
            return Some(x);
        }
        // Create a new neighbor
        if v.parent_side.map_or(true, |parent_side| {
            (side != parent_side && !ADJACENT[side as usize][parent_side as usize])
                || !self.is_near_side(v.neighbors[parent_side as usize].unwrap(), side)
        }) {
            // New neighbor will be further from the origin
            return match mode {
                NeighborType::Shorter => None,
                NeighborType::Any => Some(self.new_node(node, side)),
            };
        }
        // Neighbor is closer to the origin; find it, backfilling if necessary
        let x = self.nodes[v.parent().unwrap().idx()].neighbors[side as usize].unwrap();
        let parent_side = v.parent_side.unwrap();
        let neighbor = self.ensure_neighbor(x, parent_side);
        self.link_neighbors(node, neighbor, side);
        Some(neighbor)
    }

    /// Whether `node`'s neighbor along `side` is closer than it to the origin
    fn is_near_side(&self, node: NodeId, side: Side) -> bool {
        let v = &self.nodes[node.idx()];
        v.neighbors[side as usize].map_or(false, |x| self.nodes[x.idx()].length < v.length)
    }

    fn new_node(&mut self, parent: NodeId, side: Side) -> NodeId {
        let id = NodeId::from_idx(self.nodes.len());
        let length = self.nodes[parent.idx()].length + 1;
        self.nodes.push(Node {
            cubes: Default::default(),
            parent_side: Some(side),
            length,
            neighbors: [None; SIDES],
        });
        self.link_neighbors(id, parent, side);
        for side in Side::iter() {
            self.ensure_neighbor_inner(id, side, NeighborType::Shorter);
        }
        self.fresh.push(id);
        id
    }

    /// Register `a` and `b` as adjacent along `side`
    fn link_neighbors(&mut self, a: NodeId, b: NodeId, side: Side) {
        debug_assert!(
            self.nodes[a.idx()].neighbors[side as usize].is_none()
                && self.nodes[b.idx()].neighbors[side as usize].is_none()
        );
        self.nodes[a.idx()].neighbors[side as usize] = Some(b);
        self.nodes[b.idx()].neighbors[side as usize] = Some(a);
    }
}

impl<T> Default for Graph<T> {
    fn default() -> Self {
        Self::new()
    }
}

lazy_static! {
    /// Whether two sides share an edge
    static ref ADJACENT: [[bool; SIDES]; SIDES] = {
        let mut result = [[false; SIDES]; SIDES];
        for i in 0..SIDES {
            for j in 0..SIDES {
                let cosh_distance = (REFLECTIONS[i] * REFLECTIONS[j])[(3, 3)];
                // Possile cosh_distances: 1, 4.23606 = 2+sqrt(5), 9.47213 = 5+2*sqrt(5), 12.70820 = 6+3*sqrt(5);
                result[i][j] = if cosh_distance < 2.0 {
                    // current == next
                    false
                } else if cosh_distance < 5.0 {
                    // current adjacent to next
                    true
                } else {
                    // current not adjacent
                    false
                }
            }
        }
        result
    };

    /// Transform that moves from a neighbor to a reference node, for each side
    static ref REFLECTIONS: [na::Matrix4<f64>; SIDES] = {
        let phi = 1.25f64.sqrt() + 0.5; // golden ratio
        let root_phi = phi.sqrt();
        let f = math::lorentz_normalize(&na::Vector4::new(root_phi, phi * root_phi, 0.0, phi + 2.0));

        let mut result = [na::zero(); SIDES];
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
    static ref VERTEX_SIDES: [[Side; 3]; VERTICES] = {
        let mut result = [[Side::A; 3]; VERTICES];
        let mut vertex = 0;
        // Kind of a hack, but working this out by hand isn't any fun.
        for a in 0..SIDES {
            for b in (a+1)..SIDES {
                for c in (b+1)..SIDES {
                    if !ADJACENT[a][b] || !ADJACENT[b][c] || !ADJACENT[c][a] {
                        continue;
                    }
                    result[vertex] = [Side::VALUES[a], Side::VALUES[b], Side::VALUES[c]];
                    vertex += 1;
                }
            }
        }
        assert_eq!(vertex, 20);
        result
    };

    /// Vertex shared by 3 sides
    static ref SIDES_TO_VERTEX: [[[Vertex; SIDES]; SIDES]; SIDES] = {
        let mut result = [[[Vertex::A; SIDES]; SIDES]; SIDES];
        let mut vertex = Vertex::iter();
        // Kind of a hack, but working this out by hand isn't any fun.
        for a in 0..SIDES {
            for b in (a+1)..SIDES {
                for c in (b+1)..SIDES {
                    if !ADJACENT[a][b] || !ADJACENT[b][c] || !ADJACENT[c][a] {
                        continue;
                    }
                    let v = vertex.next().unwrap();
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
    static ref CUBE_TO_NODE_DETERMINANT_NEGATIVE: [bool; VERTICES] = {
        let mut result = [false; VERTICES];

        for v in Vertex::iter() {
            let lu = na::LU::new(v.cube_to_node());
            result[v as usize] = lu.determinant() < 0.0;
        }

        result
    };
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
enum NeighborType {
    Shorter,
    Any,
}

/// Labeled dodecahedron sides
#[derive(Debug, Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash)]
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
    const VALUES: [Self; SIDES] = [
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
        use Side::*;
        [A, B, C, D, E, F, G, H, I, J, K, L].iter().cloned()
    }
}

#[derive(Copy, Clone, Eq, PartialEq, Hash, Serialize, Deserialize)]
pub struct NodeId(NonZeroU32);

impl NodeId {
    pub const ROOT: Self = Self(unsafe { NonZeroU32::new_unchecked(1) });

    fn from_idx(x: usize) -> Self {
        Self(NonZeroU32::new(u32::try_from(x + 1).expect("graph grew too large")).unwrap())
    }

    fn idx(self) -> usize {
        (self.0.get() - 1) as usize
    }
}

impl fmt::Debug for NodeId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        (self.0.get() - 1).fmt(f)
    }
}

#[derive(Debug, Clone)]
struct Node<T> {
    cubes: [Option<T>; VERTICES],
    parent_side: Option<Side>,
    /// Distance to origin via parents
    length: u32,
    neighbors: [Option<NodeId>; SIDES],
}

impl<T> Node<T> {
    fn parent(&self) -> Option<NodeId> {
        Some(self.neighbors[self.parent_side? as usize].expect("parent edge unpopulated"))
    }
}

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

    fn cube_to_node(self) -> na::Matrix4<f64> {
        let origin = na::Vector4::new(0.0, 0.0, 0.0, 1.0);
        let [a, b, c] = VERTEX_SIDES[self as usize];
        na::Matrix4::from_columns(&[
            REFLECTIONS[a as usize].column(3) - origin,
            REFLECTIONS[b as usize].column(3) - origin,
            REFLECTIONS[c as usize].column(3) - origin,
            origin,
        ])
    }
}

// Properties of dodecahedrons
const VERTICES: usize = 20;
const SIDES: usize = 12;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parent_child_relationships() {
        let mut graph = Graph::<()>::default();
        assert_eq!(graph.len(), 1);
        let a = graph.ensure_neighbor(NodeId::ROOT, Side::A);
        assert_eq!(graph.len(), 2);
        let a2 = graph.ensure_neighbor(NodeId::ROOT, Side::A);
        assert_eq!(graph.len(), 2);
        assert_eq!(a, a2);
        assert_eq!(graph.ensure_neighbor(a, Side::A), NodeId::ROOT);
        assert_eq!(graph.nodes[a.idx()].length, 1);
        let b = graph.ensure_neighbor(NodeId::ROOT, Side::B);
        assert_eq!(graph.len(), 3);
        assert_eq!(graph.ensure_neighbor(b, Side::B), NodeId::ROOT);
        let c = graph.ensure_neighbor(a, Side::C);
        assert!(graph.len() > 4);
        assert_eq!(graph.ensure_neighbor(c, Side::C), a);
        assert_eq!(graph.nodes[c.idx()].length, 2);
    }

    #[test]
    fn children_have_common_neighbor() {
        let mut graph = Graph::<()>::default();
        let a = graph.ensure_neighbor(NodeId::ROOT, Side::A);
        let b = graph.ensure_neighbor(NodeId::ROOT, Side::B);
        let a_neighbors = Side::iter()
            .map(|side| graph.ensure_neighbor(a, side))
            .collect::<Vec<_>>();
        let b_neighbors = Side::iter()
            .map(|side| graph.ensure_neighbor(b, side))
            .collect::<Vec<_>>();
        let common = a_neighbors
            .iter()
            .cloned()
            .filter(|x| b_neighbors.contains(x))
            .collect::<Vec<_>>();

        assert_eq!(
            common.len(),
            2,
            "both the root and some other node are common neighbors"
        );
        assert!(common.contains(&NodeId::ROOT));
        let other = common
            .iter()
            .cloned()
            .filter(|&x| x != NodeId::ROOT)
            .next()
            .unwrap();
        assert_eq!(graph.nodes[other.idx()].length, 2);
    }

    #[test]
    fn cube_to_node_sanity() {
        let mut graph = Graph::<()>::new();
        graph.ensure_nearby(NodeId::ROOT, 1);
        assert_eq!(graph.cube_to_node(NodeId::ROOT), None);
        let node = graph.ensure_neighbor(NodeId::ROOT, Side::A);
        assert_eq!(graph.cube_to_node(node), None);
    }

    #[test]
    fn vertex_sides() {
        use std::collections::HashSet;
        let triples = VERTEX_SIDES.iter().collect::<HashSet<_>>();
        assert_eq!(triples.len(), VERTICES);
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
            let [a, b, c] = VERTEX_SIDES[v as usize];
            assert_eq!(v, SIDES_TO_VERTEX[a as usize][b as usize][c as usize]);
            assert_eq!(v, SIDES_TO_VERTEX[a as usize][c as usize][b as usize]);
            assert_eq!(v, SIDES_TO_VERTEX[b as usize][a as usize][c as usize]);
            assert_eq!(v, SIDES_TO_VERTEX[b as usize][c as usize][a as usize]);
            assert_eq!(v, SIDES_TO_VERTEX[c as usize][a as usize][b as usize]);
            assert_eq!(v, SIDES_TO_VERTEX[c as usize][b as usize][a as usize]);
        }
    }
}
