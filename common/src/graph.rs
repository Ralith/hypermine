use std::convert::TryFrom;
use std::fmt;
use std::num::NonZeroU32;

use fxhash::FxHashSet;
use lazy_static::lazy_static;
use serde::{Deserialize, Serialize};

use crate::{
    dodeca::{Side, Vertex, SIDE_COUNT, VERTEX_COUNT},
    math,
};

/// Graph of the right dodecahedral tiling of H^3
#[derive(Debug, Clone)]
pub struct Graph<N, C> {
    nodes: Vec<Node<N, C>>,
    fresh: Vec<NodeId>,
}

impl<N, C> Graph<N, C> {
    pub fn new() -> Self {
        Self {
            nodes: vec![Node {
                value: None,
                cubes: Default::default(),
                parent_side: None,
                length: 0,
                neighbors: [None; SIDE_COUNT],
            }],
            fresh: Vec::new(),
        }
    }

    #[inline]
    pub fn len(&self) -> u32 {
        self.nodes.len() as u32
    }

    /// Look up the ID of a node's neighbor, creating nearby nodes if necessary
    pub fn ensure_neighbor(&mut self, node: NodeId, side: Side) -> NodeId {
        self.ensure_neighbor_inner(node, side, NeighborType::Any)
            .expect("ensuring a neighbor of unconstrained type should always succeed")
    }

    /// Nodes created since the last call to `clear_fresh`
    #[inline]
    pub fn fresh(&self) -> &[NodeId] {
        &self.fresh
    }

    #[inline]
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
    ) -> Vec<(NodeId, Vertex, bool, na::Matrix4<f32>)> {
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
                    current.id,
                    v,
                    current.parity ^ CUBE_TO_NODE_DETERMINANT_NEGATIVE[v as usize],
                    na::convert(current.transform * cube_to_node(v)),
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
                    transform: current.transform * side.reflection(),
                });
                visited.insert(neighbor);
            }
        }

        result
    }

    /// Enumerate the vertices of `node` which canonically correspond to cubes
    pub fn cubes_at(&self, node: NodeId) -> impl Iterator<Item = Vertex> {
        let mut exists = [false; 20];
        let node = &self.nodes[node.idx()];
        for v in Vertex::iter() {
            exists[v as usize] = v.canonical_sides().iter().all(|&side| {
                let neighbor = match node.neighbors[side as usize] {
                    None => return true,
                    Some(x) => x,
                };
                self.nodes[neighbor.idx()].length > node.length
            });
        }
        Vertex::iter().filter(move |&i| exists[i as usize])
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
    }

    #[inline]
    pub fn get(&self, node: NodeId) -> &Option<N> {
        &self.nodes[node.idx()].value
    }

    #[inline]
    pub fn get_mut(&mut self, node: NodeId) -> &mut Option<N> {
        &mut self.nodes[node.idx()].value
    }

    #[inline]
    pub fn get_cube(&self, node: NodeId, cube: Vertex) -> &Option<C> {
        &self.nodes[node.idx()].cubes[cube as usize]
    }

    #[inline]
    pub fn get_cube_mut(&mut self, node: NodeId, cube: Vertex) -> &mut Option<C> {
        &mut self.nodes[node.idx()].cubes[cube as usize]
    }

    #[inline]
    pub fn neighbor(&self, node: NodeId, which: Side) -> Option<NodeId> {
        self.nodes[node.idx()].neighbors[which as usize]
    }

    #[inline]
    pub fn length(&self, node: NodeId) -> u32 {
        self.nodes[node.idx()].length
    }

    /// Given a `transform` relative to a `reference` node, computes an equivalent transform
    /// relative to the node closest to that transform.
    pub fn normalize_transform(
        &self,
        mut reference: NodeId,
        mut transform: na::Matrix4<f64>,
    ) -> (NodeId, na::Matrix4<f64>) {
        'outer: loop {
            let location = transform * math::origin();
            for side in Side::iter() {
                if !side.faces(&location) {
                    continue;
                }
                reference = match self.neighbor(reference, side) {
                    None => continue,
                    Some(x) => x,
                };
                transform = side.reflection() * transform;
                continue 'outer;
            }
            break;
        }
        (reference, transform)
    }

    #[inline]
    pub fn parent(&self, node: NodeId) -> Option<Side> {
        self.nodes[node.idx()].parent_side
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
            (side != parent_side && !side.adjacent_to(parent_side))
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
            value: None,
            cubes: Default::default(),
            parent_side: Some(side),
            length,
            neighbors: [None; SIDE_COUNT],
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

impl<N, C> Default for Graph<N, C> {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
enum NeighborType {
    Shorter,
    Any,
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

impl From<NodeId> for u32 {
    fn from(x: NodeId) -> u32 {
        x.0.get() - 1
    }
}

impl fmt::Debug for NodeId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        (self.0.get() - 1).fmt(f)
    }
}

#[derive(Debug, Clone)]
struct Node<N, C> {
    value: Option<N>,
    cubes: [Option<C>; VERTEX_COUNT],
    parent_side: Option<Side>,
    /// Distance to origin via parents
    length: u32,
    neighbors: [Option<NodeId>; SIDE_COUNT],
}

impl<N, C> Node<N, C> {
    fn parent(&self) -> Option<NodeId> {
        Some(self.neighbors[self.parent_side? as usize].expect("parent edge unpopulated"))
    }
}

fn cube_to_node(v: Vertex) -> na::Matrix4<f64> {
    let origin = na::Vector4::new(0.0, 0.0, 0.0, 1.0);
    let [a, b, c] = v.canonical_sides();
    na::Matrix4::from_columns(&[
        a.reflection().column(3) - origin,
        b.reflection().column(3) - origin,
        c.reflection().column(3) - origin,
        origin,
    ])
}

lazy_static! {
    /// Whether the determinant of the cube-to-node transform is negative
    static ref CUBE_TO_NODE_DETERMINANT_NEGATIVE: [bool; VERTEX_COUNT] = {
        let mut result = [false; VERTEX_COUNT];

        for v in Vertex::iter() {
            result[v as usize] = math::parity(&cube_to_node(v));
        }

        result
    };
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::*;

    #[test]
    fn parent_child_relationships() {
        let mut graph = Graph::<(), ()>::default();
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
        let mut graph = Graph::<(), ()>::default();
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
    fn normalize_transform() {
        let mut graph = Graph::<(), ()>::default();
        let a = graph.ensure_neighbor(NodeId::ROOT, Side::A);
        {
            let (node, xf) = graph.normalize_transform(NodeId::ROOT, na::Matrix4::identity());
            assert_eq!(node, NodeId::ROOT);
            assert_abs_diff_eq!(xf, na::Matrix4::identity(), epsilon = 1e-5);
        }
        {
            let (node, xf) = graph
                .normalize_transform(NodeId::ROOT, Side::A.reflection() * na::Matrix4::identity());
            assert_eq!(node, a);
            assert_abs_diff_eq!(xf, na::Matrix4::identity(), epsilon = 1e-5);
        }
    }
}
