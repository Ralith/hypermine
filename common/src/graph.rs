#![allow(clippy::len_without_is_empty)]

use std::collections::VecDeque;

use blake3::Hasher;
use fxhash::{FxHashMap, FxHashSet};
use serde::{Deserialize, Serialize};

use crate::{
    dodeca::{Side, SIDE_COUNT},
    math,
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
        for side in chunk.vertex.canonical_sides().iter().cloned() {
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
    pub fn normalize_transform<T: na::RealField + Copy>(
        &self,
        mut reference: NodeId,
        original: &na::Matrix4<T>,
    ) -> (NodeId, na::Matrix4<T>) {
        let mut transform = na::Matrix4::identity();
        let mut location = original * math::origin();
        'outer: loop {
            for side in Side::iter() {
                if !side.is_facing(&location) {
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
        self.nodes[&node].parent_side
    }

    /// Iterate over every node and its parent except the root
    pub fn tree(&self) -> TreeIter<'_> {
        TreeIter::new(self)
    }

    /// Ensures that the neighbour node at a particular side of a particular node exists in the graph,
    /// as well as the nodes from the origin to the neighbour node.
    pub fn ensure_neighbor(&mut self, node: NodeId, side: Side) -> NodeId {
        let v = &self.nodes[&node];
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
        let x = self.nodes[&v.parent().unwrap()].neighbors[side as usize].unwrap();
        let parent_side = v.parent_side.unwrap();
        let neighbor = self.ensure_neighbor(x, parent_side);
        self.link_neighbors(node, neighbor, side);
        neighbor
    }

    /// Whether `node`'s neighbor along `side` is closer than it to the origin
    fn is_near_side(&self, node: NodeId, side: Side) -> bool {
        let v = &self.nodes[&node];
        v.neighbors[side as usize].map_or(false, |x| self.nodes[&x].length < v.length)
    }

    pub fn insert_child(&mut self, parent: NodeId, side: Side) -> NodeId {
        // Always create shorter nodes first so that self.nodes always puts parent nodes before their child nodes, enabling
        // graceful synchronization of the graph
        let shorter_neighbors = self.populate_shorter_neighbors_of_child(parent, side);

        // Select the side along the canonical path from the origin to this node. Future work: make
        // this the parent to reduce the number of concepts.
        let (path_side, predecessor) = shorter_neighbors
            .clone()
            .chain(Some((side, parent)))
            .min_by_key(|&(side, _)| side)
            .unwrap();
        let mut hasher = Hasher::new();
        hasher.update(&predecessor.0.to_le_bytes());
        hasher.update(&[path_side as u8]);
        let mut xof = hasher.finalize_xof();
        let mut hash = [0; 16];
        xof.fill(&mut hash);
        let id = NodeId(u128::from_le_bytes(hash));

        let length = self.nodes[&parent].length + 1;
        self.nodes
            .insert(id, NodeContainer::new(Some(side), length));
        self.link_neighbors(id, parent, side);
        for (side, neighbor) in shorter_neighbors {
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

    /// Ensure all shorter neighbors of a not-yet-created child node exist and return them, excluding the given parent node
    fn populate_shorter_neighbors_of_child(
        &mut self,
        parent: NodeId,
        parent_side: Side,
    ) -> impl Iterator<Item = (Side, NodeId)> + Clone {
        let mut neighbors = [None; 2]; // Maximum number of shorter neighbors other than the given parent is 2
        let mut count = 0;
        for neighbor_side in Side::iter() {
            if neighbor_side == parent_side
                || !neighbor_side.adjacent_to(parent_side)
                || !self.is_near_side(parent, neighbor_side)
            {
                continue;
            }
            let x = self.nodes[&parent].neighbors[neighbor_side as usize].unwrap();
            let neighbor = self.ensure_neighbor(x, parent_side);
            neighbors[count] = Some((neighbor_side, neighbor));
            count += 1;
        }
        (0..2).filter_map(move |i| neighbors[i])
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

    fn parent(&self) -> Option<NodeId> {
        Some(self.neighbors[self.parent_side? as usize].expect("parent edge unpopulated"))
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
    fn parent_child_relationships() {
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
    fn children_have_common_neighbor() {
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
        let other = common.iter().cloned().find(|&x| x != NodeId::ROOT).unwrap();
        assert_eq!(graph.nodes[&other].length, 2);
    }

    #[test]
    fn normalize_transform() {
        let mut graph = Graph::new(1);
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
        let mut a = Graph::new(1);
        ensure_nearby(&mut a, &Position::origin(), 3.0);
        let mut b = Graph::new(1);
        for (side, parent) in a.tree() {
            b.insert_child(parent, side);
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
