#![allow(clippy::len_without_is_empty)]

use std::collections::VecDeque;

use blake3::Hasher;
use fxhash::{FxHashMap, FxHashSet};
use serde::{Deserialize, Serialize};

use crate::{
    dodeca::{Side, SIDE_COUNT},
    math::{MIsometry, MVector},
    node::{ChunkId, ChunkLayout, Node},
};

/// Graph of the right dodecahedral tiling of H^3
pub struct Graph {
    nodes: FxHashMap<NodeId, NodeContainer>,
    /// This field stores implicitly added nodes to ensure that they're initialized in the correct
    /// order
    fresh: Vec<NodeId>,
    layout: ChunkLayout,
}

impl Graph {
    pub fn new(dimension: u8) -> Self {
        let mut nodes = FxHashMap::default();
        nodes.insert(NodeId::ROOT, NodeContainer::new(None, 0));
        Self {
            nodes,
            fresh: vec![NodeId::ROOT],
            layout: ChunkLayout::new(dimension),
        }
    }

    #[inline]
    pub fn layout(&self) -> &ChunkLayout {
        &self.layout
    }

    #[inline]
    pub fn len(&self) -> u32 {
        self.nodes.len() as u32
    }

    #[inline]
    pub fn contains(&self, node: NodeId) -> bool {
        self.nodes.contains_key(&node)
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
    pub fn canonicalize(&self, mut chunk: ChunkId) -> Option<ChunkId> {
        for side in chunk.vertex.canonical_sides().into_iter() {
            // missing neighbors are always longer
            if let Some(neighbor) = self.neighbor(chunk.node, side) {
                if self.length(neighbor) < self.length(chunk.node) {
                    chunk.node = neighbor;
                }
            }
        }
        Some(chunk)
    }

    /// Returns all of the sides between the provided node and its shorter neighbors.
    ///
    /// A node's length is its distance from the root node.
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

    #[inline]
    pub fn get(&self, node: NodeId) -> &Option<Node> {
        &self.nodes[&node].value
    }

    #[inline]
    pub fn get_mut(&mut self, node: NodeId) -> &mut Option<Node> {
        &mut self.nodes.get_mut(&node).unwrap().value
    }

    #[inline]
    pub fn neighbor(&self, node: NodeId, which: Side) -> Option<NodeId> {
        self.nodes[&node].neighbors[which as usize]
    }

    #[inline]
    pub fn length(&self, node: NodeId) -> u32 {
        self.nodes[&node].length
    }

    /// Given a `transform` relative to a `reference` node, computes the node
    /// that it's closest to and the transform that moves it there
    pub fn normalize_transform(
        &self,
        mut reference: NodeId,
        original: &MIsometry<f32>,
    ) -> (NodeId, MIsometry<f32>) {
        let mut transform = MIsometry::identity();
        let mut location = *original * MVector::origin();
        'outer: loop {
            for side in Side::iter() {
                if !side.is_facing(&location) {
                    continue;
                }
                reference = match self.neighbor(reference, side) {
                    None => continue,
                    Some(x) => x,
                };
                let mat = *side.reflection();
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
        self.nodes[&node].parent_side
    }

    /// Iterate over every node and its parent except the root
    pub fn tree(&self) -> TreeIter<'_> {
        TreeIter::new(self)
    }

    /// Ensures that the neighbour node at a particular side of a particular node exists in the graph,
    /// as well as the nodes from the origin to the neighbour node.
    pub fn ensure_neighbor(&mut self, node: NodeId, side: Side) -> NodeId {
        self.nodes[&node].neighbors[side as usize]
            .unwrap_or_else(|| self.insert_neighbor(node, side))
    }

    /// Whether `node`'s neighbor along `side` is closer than it to the origin
    fn is_descender(&self, node: NodeId, side: Side) -> bool {
        let v = &self.nodes[&node];
        v.neighbors[side as usize].map_or(false, |x| self.nodes[&x].length < v.length)
    }

    /// Inserts the neighbor of the given node at the given side into the graph, ensuring that all
    /// shorter nodes it depends on are created first.
    pub fn insert_neighbor(&mut self, node: NodeId, side: Side) -> NodeId {
        // To help improve readability, we use the term "subject" to refer to the not-yet-created neighbor node, since the term.
        // "neighbor" is already used in many other places.

        // An assumption made by this function is that `side` is not a descender of `node`. This is a safe assumption to make
        // because a node cannot be constructed before its shorter neighbors. The call to `populate_shorter_neighbors_of_neighbor`
        // guarantees this.
        let shorter_neighbors_of_subject = self.populate_shorter_neighbors_of_subject(node, side);

        // Select the side along the canonical path from the origin to this node. This is guaranteed
        // to be the first entry of the `shorter_neighbors_of_subject` iterator.
        let (parent_side, parent) = shorter_neighbors_of_subject.clone().next().unwrap();
        let mut hasher = Hasher::new();
        hasher.update(&parent.0.to_le_bytes());
        hasher.update(&[parent_side as u8]);
        let mut xof = hasher.finalize_xof();
        let mut hash = [0; 16];
        xof.fill(&mut hash);
        let id = NodeId(u128::from_le_bytes(hash));

        let length = self.nodes[&node].length + 1;
        self.nodes
            .insert(id, NodeContainer::new(Some(parent_side), length));
        for (side, neighbor) in shorter_neighbors_of_subject {
            self.link_neighbors(id, neighbor, side);
        }
        self.fresh.push(id);
        id
    }

    #[inline]
    pub fn hash_of(&self, node: NodeId) -> u128 {
        node.0
    }

    #[inline]
    pub fn from_hash(&self, hash: u128) -> NodeId {
        NodeId(hash)
    }

    /// Ensure all shorter neighbors of a not-yet-created neighbor node (which we call the "subject") exist, and return them
    /// (including the given node) in the form of ordered pairs containing the side they share with this not-yet-created
    /// neighbor node, and their node ID. These ordered pairs will be sorted by side, based on enum order.
    fn populate_shorter_neighbors_of_subject(
        &mut self,
        node: NodeId,
        side: Side,
    ) -> impl Iterator<Item = (Side, NodeId)> + Clone {
        let mut shorter_neighbors_of_subject = [None; 3]; // Maximum number of shorter neighbors is 3
        let mut count = 0;
        for candidate_descender in Side::iter() {
            if candidate_descender == side {
                // The given node is included in the list of returned nodes.
                shorter_neighbors_of_subject[count] = Some((side, node));
                count += 1;
            } else if candidate_descender.adjacent_to(side)
                && self.is_descender(node, candidate_descender)
            {
                // This branch covers shorter neighbors of the subject other than the given node.
                // This is non-obvious, as it relies on the fact that a side is a descender of the subject
                // exactly when it is a descender of the given node. This is not true in general, but it is true
                // when the descender in question is adjacent to the side shared by the given node and the subject.
                // That is what allows the `self.is_near_side(node, candidate_descender_side)` condition to behave as desired.

                // We would like to return (and recursively create if needed) the shorter neighbor of the subject. This means that
                // if we label the shared side A and the descender B, the path we would like to follow from the given node is AB,
                // since A will take us to the subject, and then B will take us to its shorter neighbor. However, taking
                // the path AB is impossible because it would require the subject to already be in the graph. Fortuantely,
                // we can take the path BA instead because that will reach the same node, thanks to the fact that each edge
                // is shared by 4 dodecas.
                let shorter_neighbor_of_node = self.neighbor(node, candidate_descender).unwrap();
                let shorter_neighbor_of_subject =
                    self.ensure_neighbor(shorter_neighbor_of_node, side);
                shorter_neighbors_of_subject[count] =
                    Some((candidate_descender, shorter_neighbor_of_subject));
                count += 1;
            } else {
                // The `candidate_descender_side` is not a descender of the subject, so no action is necessary.
            }
        }
        shorter_neighbors_of_subject.into_iter().flatten()
    }

    /// Register `a` and `b` as adjacent along `side`
    fn link_neighbors(&mut self, a: NodeId, b: NodeId, side: Side) {
        debug_assert!(
            self.nodes[&a].neighbors[side as usize].is_none()
                && self.nodes[&b].neighbors[side as usize].is_none()
        );
        self.nodes.get_mut(&a).unwrap().neighbors[side as usize] = Some(b);
        self.nodes.get_mut(&b).unwrap().neighbors[side as usize] = Some(a);
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash, Serialize, Deserialize)]
pub struct NodeId(u128);

impl NodeId {
    pub const ROOT: Self = Self(0);
}

struct NodeContainer {
    value: Option<Node>,
    parent_side: Option<Side>,
    /// Distance to origin via parents
    length: u32,
    neighbors: [Option<NodeId>; SIDE_COUNT],
}

impl NodeContainer {
    fn new(parent_side: Option<Side>, length: u32) -> Self {
        Self {
            value: None,
            parent_side,
            length,
            neighbors: [None; SIDE_COUNT],
        }
    }
}

// Iterates through the graph with breadth-first search
pub struct TreeIter<'a> {
    queue: VecDeque<NodeId>,
    visited: FxHashSet<NodeId>,
    nodes: &'a FxHashMap<NodeId, NodeContainer>,
}

impl<'a> TreeIter<'a> {
    fn new(graph: &'a Graph) -> Self {
        let mut result = TreeIter {
            queue: VecDeque::from([NodeId::ROOT]),
            visited: FxHashSet::from_iter([NodeId::ROOT]),
            nodes: &graph.nodes,
        };

        // Skip the root node
        let _ = result.next_node();

        result
    }

    // Returns the next Node in the traversal. The iterator returns its parent and parent side.
    fn next_node(&mut self) -> Option<&NodeContainer> {
        let node_id = self.queue.pop_front()?;
        let node = &self.nodes[&node_id];
        for side in Side::iter() {
            if let Some(neighbor) = node.neighbors[side as usize] {
                if !self.visited.contains(&neighbor) {
                    self.queue.push_back(neighbor);
                    self.visited.insert(neighbor);
                }
            }
        }
        Some(node)
    }
}

impl Iterator for TreeIter<'_> {
    type Item = (Side, NodeId);

    fn next(&mut self) -> Option<Self::Item> {
        let node = self.next_node()?;
        let side = node.parent_side.unwrap();
        Some((side, node.neighbors[side as usize].unwrap()))
    }
}

#[cfg(test)]
mod tests {
    use crate::{proto::Position, traversal::ensure_nearby};

    use super::*;
    use approx::*;

    #[test]
    fn shorter_longer_neighbor_relationships() {
        let mut graph = Graph::new(1);
        assert_eq!(graph.len(), 1);
        let a = graph.ensure_neighbor(NodeId::ROOT, Side::A);
        assert_eq!(graph.len(), 2);
        let a2 = graph.ensure_neighbor(NodeId::ROOT, Side::A);
        assert_eq!(graph.len(), 2);
        assert_eq!(a, a2);
        assert_eq!(graph.ensure_neighbor(a, Side::A), NodeId::ROOT);
        assert_eq!(graph.nodes[&a].length, 1);
        let b = graph.ensure_neighbor(NodeId::ROOT, Side::B);
        assert_eq!(graph.len(), 3);
        assert_eq!(graph.ensure_neighbor(b, Side::B), NodeId::ROOT);
        let c = graph.ensure_neighbor(a, Side::C);
        assert!(graph.len() > 4);
        assert_eq!(graph.ensure_neighbor(c, Side::C), a);
        assert_eq!(graph.nodes[&c].length, 2);
    }

    #[test]
    fn longer_nodes_have_common_neighbor() {
        let mut graph = Graph::new(1);
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
        let other = common.into_iter().find(|&x| x != NodeId::ROOT).unwrap();
        assert_eq!(graph.nodes[&other].length, 2);
    }

    #[test]
    fn normalize_transform() {
        let mut graph = Graph::new(1);
        let a = graph.ensure_neighbor(NodeId::ROOT, Side::A);
        {
            let (node, xf) = graph.normalize_transform(NodeId::ROOT, &MIsometry::identity());
            assert_eq!(node, NodeId::ROOT);
            assert_abs_diff_eq!(xf, MIsometry::identity(), epsilon = 1e-5);
        }
        {
            let (node, xf) = graph.normalize_transform(NodeId::ROOT, Side::A.reflection());
            assert_eq!(node, a);
            assert_abs_diff_eq!(xf, Side::A.reflection(), epsilon = 1e-5);
        }
    }

    #[test]
    fn rebuild_from_tree() {
        let mut a = Graph::new(1);
        ensure_nearby(&mut a, &Position::origin(), 3.0);
        let mut b = Graph::new(1);
        for (side, parent) in a.tree() {
            b.insert_neighbor(parent, side);
        }
        assert_eq!(a.len(), b.len());
        for (c, d) in a.tree().zip(b.tree()) {
            assert_eq!(c.0, d.0);
            assert_eq!(a.neighbor(c.1, c.0), b.neighbor(c.1, c.0));
        }
    }

    #[test]
    fn hash_consistency() {
        let h1 = {
            let mut g = Graph::new(1);
            let n1 = g.ensure_neighbor(NodeId::ROOT, Side::A);
            let n2 = g.ensure_neighbor(n1, Side::B);
            let n3 = g.ensure_neighbor(n2, Side::C);
            g.ensure_neighbor(n3, Side::D)
        };
        let h2 = {
            let mut g = Graph::new(1);
            let n1 = g.ensure_neighbor(NodeId::ROOT, Side::C);
            let n2 = g.ensure_neighbor(n1, Side::A);
            let n3 = g.ensure_neighbor(n2, Side::B);
            g.ensure_neighbor(n3, Side::D)
        };

        assert_eq!(h1, h2);
    }
}
