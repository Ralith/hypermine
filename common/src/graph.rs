#![allow(clippy::len_without_is_empty)]

use std::convert::TryFrom;
use std::fmt;
use std::num::NonZeroU32;

use fxhash::FxHashSet;
use serde::{Deserialize, Serialize};

use crate::{
    dodeca::{Side, Vertex, SIDE_COUNT},
    math,
    proto::Position,
};

/// Graph of the right dodecahedral tiling of H^3
#[derive(Debug, Clone)]
pub struct Graph<N> {
    nodes: Vec<Node<N>>,
    /// This field stores implicitly added nodes to ensure that they're initialized in the correct
    /// order
    fresh: Vec<NodeId>,
}

impl<N> Graph<N> {
    pub fn new() -> Self {
        Self {
            nodes: vec![Node::new(None, 0)],
            fresh: vec![NodeId::ROOT],
        }
    }

    #[inline]
    pub fn len(&self) -> u32 {
        self.nodes.len() as u32
    }

    #[inline]
    pub fn contains(&self, node: NodeId) -> bool {
        node.0.get() as usize <= self.nodes.len()
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

    /// Node and vertex that the cube around a certain vertex is canonically assigned to.
    ///
    /// Each cube is said to be canonically assigned to the shortest of the nodes it touches.
    ///
    /// A node's length is defined as a its distance from the root node.
    pub fn canonicalize(&self, mut node: NodeId, vertex: Vertex) -> Option<(NodeId, Vertex)> {
        for side in vertex.canonical_sides().iter().cloned() {
            // missing neighbors are always longer
            if let Some(neighbor) = self.neighbor(node, side) {
                if self.length(neighbor) < self.length(node) {
                    node = neighbor;
                }
            }
        }
        Some((node, vertex))
    }

    /// Returns all of the sides between the provided node and its shorter neighbors.
    ///
    /// A node's length is a its distance from the root node.
    pub fn descenders(&self, node: NodeId) -> impl ExactSizeIterator<Item = (Side, NodeId)> {
        let node_length = self.length(node);

        let mut results = [None; 3];
        let mut len = 0;

        for side in Side::iter() {
            // filtering out not-yet-allocated neighbors is fine since
            // they have to be longer than us not to be allocated yet
            if let Some(neighbor_node) = self.neighbor(node, side) {
                if self.length(neighbor_node) < node_length {
                    results[len] = Some((side, neighbor_node));
                    len += 1;
                }
            }
        }

        (0..len).map(move |i| results[i].unwrap())
    }

    /// Compute `start.node`-relative transforms of all nodes whose origins lie within `distance` of
    /// `start`
    pub fn nearby_nodes(&self, start: &Position, distance: f64) -> Vec<(NodeId, na::Matrix4<f32>)> {
        struct PendingNode {
            id: NodeId,
            transform: na::Matrix4<f64>,
        }

        let mut result = Vec::new();
        let mut pending = Vec::<PendingNode>::new();
        let mut visited = FxHashSet::<NodeId>::default();
        let start_p = start.local.map(|x| x as f64) * math::origin();

        pending.push(PendingNode {
            id: start.node,
            transform: na::Matrix4::identity(),
        });
        visited.insert(start.node);

        while let Some(current) = pending.pop() {
            let node = &self.nodes[current.id.idx()];
            let current_p = current.transform * math::origin();
            if math::distance(&start_p, &current_p) > distance {
                continue;
            }
            result.push((current.id, na::convert(current.transform)));

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
                    transform: current.transform * side.reflection(),
                });
                visited.insert(neighbor);
            }
        }

        result
    }

    // /// Ensure all nodes within `distance` of `start` exist
    // pub fn ensure_nearby(&mut self, start: &Position, distance: f64) {
    //     let mut pending = Vec::<(NodeId, na::Matrix4<f64>)>::new();
    //     let mut visited = FxHashSet::<NodeId>::default();

    //     pending.push((start.node, na::Matrix4::identity()));
    //     visited.insert(start.node);
    //     let start_p = start.local.map(|x| x as f64) * math::origin();

    //     while let Some((node, current_transform)) = pending.pop() {
    //         for side in Side::iter() {
    //             let neighbor = self.ensure_neighbor(node, side);
    //             if visited.contains(&neighbor) {
    //                 continue;
    //             }
    //             visited.insert(neighbor);
    //             let neighbor_transform = current_transform * side.reflection();
    //             let neighbor_p = neighbor_transform * math::origin();
    //             if math::distance(&start_p, &neighbor_p) > distance {
    //                 continue;
    //             }
    //             pending.push((neighbor, neighbor_transform));
    //         }
    //     }
    // }

    #[inline]
    pub fn get(&self, node: NodeId) -> &Option<N> {
        &self.nodes[node.idx()].value
    }

    #[inline]
    pub fn get_mut(&mut self, node: NodeId) -> &mut Option<N> {
        &mut self.nodes[node.idx()].value
    }

    #[inline]
    pub fn neighbor(&self, node: NodeId, which: Side) -> Option<NodeId> {
        self.nodes[node.idx()].neighbors[which as usize]
    }

    #[inline]
    pub fn length(&self, node: NodeId) -> u32 {
        self.nodes[node.idx()].length
    }

    /// Given a `transform` relative to a `reference` node, computes the node that it's closest to
    /// and the transform that moves it there
    pub fn normalize_transform<T: na::RealField>(
        &self,
        mut reference: NodeId,
        original: &na::Matrix4<T>,
    ) -> (NodeId, na::Matrix4<T>) {
        let mut transform = na::Matrix4::identity();
        let mut location = original * math::origin();
        'outer: loop {
            for side in Side::iter() {
                if !side.faces(&location) {
                    continue;
                }
                reference = match self.neighbor(reference, side) {
                    None => continue,
                    Some(x) => x,
                };
                let mat = na::convert::<_, na::Matrix4<T>>(*side.reflection());
                location = mat * location;
                transform = mat * transform;
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

    /// Iterate over every node and its parent
    pub fn tree(&self) -> TreeIter<'_, N> {
        TreeIter {
            id: NodeId::from_idx(1),
            remaining: &self.nodes[1..],
        }
    }

    pub fn ensure_neighbor(&mut self, node: NodeId, side: Side) -> NodeId {
        let v = &self.nodes[node.idx()];
        if let Some(x) = v.neighbors[side as usize] {
            // Neighbor already exists
            return x;
        }

        let neighbor_is_further = |parent_side| {
            (side != parent_side && !side.adjacent_to(parent_side))
                || !self.is_near_side(v.parent().unwrap(), side)
        };

        // Create a new neighbor
        if v.parent_side.map_or(true, neighbor_is_further) {
            // New neighbor will be further from the origin
            return self.insert_child(node, side);
        }

        // Neighbor is closer to the origin; find it, backfilling if necessary
        let x = self.nodes[v.parent().unwrap().idx()].neighbors[side as usize].unwrap();
        let parent_side = v.parent_side.unwrap();
        let neighbor = self.ensure_neighbor(x, parent_side);
        self.link_neighbors(node, neighbor, side);
        neighbor
    }

    /// Whether `node`'s neighbor along `side` is closer than it to the origin
    fn is_near_side(&self, node: NodeId, side: Side) -> bool {
        let v = &self.nodes[node.idx()];
        v.neighbors[side as usize].map_or(false, |x| self.nodes[x.idx()].length < v.length)
    }

    pub fn insert_child(&mut self, parent: NodeId, side: Side) -> NodeId {
        // Always create shorter nodes first so that self.nodes is always sorted by length, enabling
        // graceful synchronization of the graph
        let shorter_neighbors = self.populate_shorter_neighbors_of_child(parent, side);
        let id = NodeId::from_idx(self.nodes.len());
        let length = self.nodes[parent.idx()].length + 1;
        self.nodes.push(Node::new(Some(side), length));
        self.link_neighbors(id, parent, side);
        for (side, neighbor) in shorter_neighbors {
            self.link_neighbors(id, neighbor, side);
        }
        self.fresh.push(id);
        id
    }

    /// Ensure all shorter neighbors of a not-yet-created child node exist and return them
    fn populate_shorter_neighbors_of_child(
        &mut self,
        parent: NodeId,
        parent_side: Side,
    ) -> impl Iterator<Item = (Side, NodeId)> {
        let mut neighbors = [None; 3]; // Maximum number of shorter neighbors is 3
        let mut count = 0;
        for neighbor_side in Side::iter() {
            if neighbor_side == parent_side
                || !neighbor_side.adjacent_to(parent_side)
                || !self.is_near_side(parent, neighbor_side)
            {
                continue;
            }
            let x = self.nodes[parent.idx()].neighbors[neighbor_side as usize].unwrap();
            let neighbor = self.ensure_neighbor(x, parent_side);
            neighbors[count] = Some((neighbor_side, neighbor));
            count += 1;
        }
        (0..3).filter_map(move |i| neighbors[i])
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

impl<N> Default for Graph<N> {
    fn default() -> Self {
        Self::new()
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
struct Node<N> {
    value: Option<N>,
    parent_side: Option<Side>,
    /// Distance to origin via parents
    length: u32,
    neighbors: [Option<NodeId>; SIDE_COUNT],
}

impl<N> Node<N> {
    fn new(parent_side: Option<Side>, length: u32) -> Self {
        Self {
            value: None,
            parent_side,
            length,
            neighbors: [None; SIDE_COUNT],
        }
    }

    fn parent(&self) -> Option<NodeId> {
        Some(self.neighbors[self.parent_side? as usize].expect("parent edge unpopulated"))
    }
}

pub struct TreeIter<'a, N> {
    id: NodeId,
    remaining: &'a [Node<N>],
}

impl<N> Iterator for TreeIter<'_, N> {
    type Item = (Side, NodeId);

    fn next(&mut self) -> Option<Self::Item> {
        let (node, rest) = self.remaining.split_first()?;
        self.remaining = rest;
        self.id = NodeId::from_idx(self.id.idx() + 1);
        let side = node.parent_side.unwrap();
        Some((side, node.neighbors[side as usize].unwrap()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::*;

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
        let other = common.iter().cloned().find(|&x| x != NodeId::ROOT).unwrap();
        assert_eq!(graph.nodes[other.idx()].length, 2);
    }

    #[test]
    fn normalize_transform() {
        let mut graph = Graph::<()>::default();
        let a = graph.ensure_neighbor(NodeId::ROOT, Side::A);
        {
            let (node, xf) =
                graph.normalize_transform::<f32>(NodeId::ROOT, &na::Matrix4::identity());
            assert_eq!(node, NodeId::ROOT);
            assert_abs_diff_eq!(xf, na::Matrix4::identity(), epsilon = 1e-5);
        }
        {
            let (node, xf) = graph.normalize_transform(NodeId::ROOT, Side::A.reflection());
            assert_eq!(node, a);
            assert_abs_diff_eq!(xf, Side::A.reflection(), epsilon = 1e-5);
        }
    }

    #[test]
    fn rebuild_from_tree() {
        let mut a = Graph::<()>::default();
        a.ensure_nearby(&Position::origin(), 3.0);
        let mut b = Graph::<()>::default();
        for (side, parent) in a.tree() {
            b.insert_child(parent, side);
        }
        assert_eq!(a.nodes.len(), b.nodes.len());
        for (a, b) in a.nodes.iter().zip(b.nodes.iter()) {
            assert_eq!(a.parent_side, b.parent_side);
            if let Some(side) = a.parent_side {
                assert_eq!(a.neighbors[side as usize], b.neighbors[side as usize]);
            }
        }
    }
}
