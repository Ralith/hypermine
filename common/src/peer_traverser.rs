use std::sync::LazyLock;

use arrayvec::ArrayVec;

use crate::{
    dodeca::Side,
    graph::{Graph, NodeId},
};

/// Details relating to a specific peer node. For a given base node, a peer node is
/// any node of the same depth as the base node where it is possible to reach the
/// same node from both the base and the peer node without "going backwards". Going backwards
/// in this sense means going from a node with a higher depth to a node with a lower depth.
///
/// Peer nodes are important because if worldgen produces a structure at a given base node
/// and another structure at a given peer node, those two structures could potentially intersect
/// if care is not taken. Checking the peer nodes in advance will prevent this.
pub struct PeerNode {
    node_id: NodeId,
    parent_path: ArrayVec<Side, 2>,
    child_path: ArrayVec<Side, 2>,
}

impl PeerNode {
    /// The ID of the peer node
    #[inline]
    pub fn node(&self) -> NodeId {
        self.node_id
    }

    /// The sequence of sides that takes you from the peer node to the shared child node
    #[inline]
    pub fn path_from_peer(&self) -> impl ExactSizeIterator<Item = Side> + Clone + use<> {
        self.parent_path.clone().into_iter().rev()
    }

    /// The sequence of sides that takes you from the base node to the shared child node
    #[inline]
    pub fn path_from_base(&self) -> impl ExactSizeIterator<Item = Side> + Clone + use<> {
        self.child_path.clone().into_iter()
    }
}

/// Allows traversal through all `PeerNode`s for a given base node.
pub struct PeerTraverser {
    /// The path leading from the base node to the ancestor node we're currently looking at
    parent_path: ArrayVec<Side, 2>,

    /// The nodes traversed by `parent_path`, including the start and end nodes. The length of `parent_path_nodes`
    /// is always one greater than the length of `parent_path`.
    parent_path_nodes: ArrayVec<NodeId, 3>,

    /// Index into the appropriate element of `DEPTH1_CHILD_PATHS` or `DEPTH2_CHILD_PATHS`
    /// that the iterator is currently on
    child_path_index: usize,
}

// This implementation is rather complicated and can be difficult to follow. It is recommended to see the
// `alternative_implementation` test for an equivalent algorithm. Most of the logic here is just maintaining
// state to allow `PeerTraverser` to work much like an iterator instead of storing everything into an array or vec.
impl PeerTraverser {
    pub fn new(base_node: NodeId) -> Self {
        PeerTraverser {
            parent_path: ArrayVec::new(),
            parent_path_nodes: ArrayVec::from_iter([base_node]),
            child_path_index: 0,
        }
    }

    /// Assuming `parent_path` obeys shortlex rules up to right before the last
    /// element for a given depth, returns whether it still obeys shortlex rules.
    fn parent_path_end_is_shortlex(&self, depth: usize) -> bool {
        if depth <= 1 {
            // One-element node strings are always shortlex.
            return true;
        };
        let last = self.parent_path[depth - 1];
        let second_last = self.parent_path[depth - 2];
        if last == second_last {
            // Backtracking is not valid (short part of shortlex)
            return false;
        }
        if last.adjacent_to(second_last) && (last as usize) < (second_last as usize) {
            // Unnecessarily having a higher side index first is not valid (lex part of shortlex).
            // This is because adjacent sides in a path can always be swapped, still leading to the same node.
            return false;
        }
        true
    }

    /// Assuming `parent_path` and `parent_path_nodes` is already valid apart from possibly the last node,
    /// increments to the next valid `parent_path` for the given depth and updates `parent_path_nodes`.
    /// If `allow_unchanged_path` is true, the `parent_path` will not be incremented if it is already valid, but
    /// `parent_path_nodes` will still be updated.
    /// Returns `false` if no such valid path exists.
    #[must_use]
    fn increment_parent_path_for_depth(
        &mut self,
        graph: &Graph,
        depth: usize,
        mut allow_unchanged_path: bool,
    ) -> bool {
        if depth == 0 {
            // Empty paths are always valid, but they cannot be incremented.
            return allow_unchanged_path;
        }
        loop {
            // Check if we're done incrementing and exit if so.
            if allow_unchanged_path && self.parent_path_end_is_shortlex(depth) {
                if let Some(node) = graph.neighbor(
                    self.parent_path_nodes[depth - 1],
                    self.parent_path[depth - 1],
                ) {
                    if graph.length(self.parent_path_nodes[depth - 1]) == graph.length(node) + 1 {
                        self.parent_path_nodes[depth] = node;
                        return true;
                    }
                }
            }

            // Otherwise, increment.
            let mut current_side = self.parent_path[depth - 1];
            current_side = Side::VALUES[(current_side as usize + 1) % Side::VALUES.len()]; // Cycle the current side
            if current_side == Side::A {
                // We looped, so make sure to increment an earlier part of the path.
                if !self.increment_parent_path_for_depth(graph, depth - 1, false) {
                    return false;
                }
            }
            self.parent_path[depth - 1] = current_side;

            allow_unchanged_path = true; // The path has changed, so it won't necessarily need to be changed again.
        }
    }

    /// Increments to the next valid `parent_path` for the given depth and updates `parent_path_nodes`.
    /// Returns `false` if no such valid path exists.
    #[must_use]
    fn increment_parent_path(&mut self, graph: &Graph) -> bool {
        let depth = self.parent_path.len();
        if depth == 0 {
            // We're on the first iteration. If there are any peers at all, there will be one at depth 1.
            self.parent_path.push(Side::A);
            self.parent_path_nodes.push(NodeId::ROOT);
            return self.increment_parent_path_for_depth(graph, 1, true);
        } else if depth == 1 {
            // Last time this was called, we were on a depth 1 path. Check if there's another depth 1 path to use.
            if self.increment_parent_path_for_depth(graph, 1, false) {
                return true;
            }
            // Otherwise, switch to depth 2 paths.
            self.parent_path.fill(Side::A);
            self.parent_path.push(Side::A);
            self.parent_path_nodes.push(NodeId::ROOT);
            // We're on the first depth 2 path, so we cannot make any assumptions. Therefore, to assure the resulting
            // path is valid, we need to call `increment_parent_path_for_depth` for its depth 1 subpath first before
            // calling it for the whole path.
            return self.increment_parent_path_for_depth(graph, 1, true)
                && self.increment_parent_path_for_depth(graph, 2, true);
        } else if depth == 2 {
            // Last time this was called, we were on a depth 2 path. Check if there's another depth 2 path to use.
            return self.increment_parent_path_for_depth(graph, 2, false);
        }
        false
    }

    /// Increments `child_path_index` to the index of the next valid path and returns the associated `PeerNode`.
    /// If `allow_unchanged_path` is true, the `child_path_index` will not be incremented if it is already valid.
    /// Returns `None` if no such valid path exists.
    #[must_use]
    fn increment_child_path(
        &mut self,
        graph: &mut impl GraphRef,
        mut allow_unchanged_path: bool,
    ) -> Option<PeerNode> {
        if self.parent_path.len() == 1 {
            let child_paths = &DEPTH1_CHILD_PATHS[self.parent_path[0] as usize];
            loop {
                if allow_unchanged_path {
                    if self.child_path_index >= child_paths.len() {
                        return None;
                    }
                    let child_side = child_paths[self.child_path_index];
                    let mut current_node = self.parent_path_nodes[1];
                    current_node = graph.neighbor(current_node, child_side);
                    if graph.length(current_node) == graph.length(self.parent_path_nodes[0]) {
                        return Some(PeerNode {
                            node_id: current_node,
                            parent_path: self.parent_path.clone(),
                            child_path: ArrayVec::from_iter([child_side]),
                        });
                    }
                }
                self.child_path_index += 1;
                allow_unchanged_path = true;
            }
        } else if self.parent_path.len() == 2 {
            let child_paths =
                &DEPTH2_CHILD_PATHS[self.parent_path[0] as usize][self.parent_path[1] as usize];
            loop {
                if allow_unchanged_path {
                    if self.child_path_index >= child_paths.len() {
                        return None;
                    }
                    let child_path = &child_paths[self.child_path_index];
                    let mut current_node = self.parent_path_nodes[2];
                    for &side in child_path {
                        current_node = graph.neighbor(current_node, side);
                    }
                    if graph.length(current_node) == graph.length(self.parent_path_nodes[0]) {
                        let mut result_child_path = ArrayVec::new();
                        result_child_path.push(child_path[0]);
                        result_child_path.push(child_path[1]);
                        return Some(PeerNode {
                            node_id: current_node,
                            parent_path: self.parent_path.clone(),
                            child_path: result_child_path,
                        });
                    }
                }
                self.child_path_index += 1;
                allow_unchanged_path = true;
            }
        }
        None
    }

    /// Returns the next peer node, or `None` if there are no peer nodes left. The implementation of `GraphRef`
    /// is used to decide how to handle traversing through potentially not-yet-created nodes.
    fn next_impl(&mut self, mut graph: impl GraphRef) -> Option<PeerNode> {
        let mut allow_unchanged_path = false;
        loop {
            // The parent path is guaranteed to be valid here. It starts valid before iteration because empty
            // paths are valid, and it remains valid because it is only modified with `increment_parent_path`.
            // Therefore, if we can increment the child path here, that is all we need to do.
            if let Some(node) = self.increment_child_path(&mut graph, allow_unchanged_path) {
                return Some(node);
            }
            // If there is no valid child path, we need to increment the parent path.
            if !self.increment_parent_path(graph.as_ref()) {
                return None;
            }
            allow_unchanged_path = true; // The path has changed, so it won't necessarily need to be changed again.
            self.child_path_index = 0; // Since we incremented the parent, we should make sure to reset the child path index.
        }
    }

    /// Assumes the graph is expanded enough to traverse peer nodes and returns the next peer node,
    /// or `None` if there are no peer nodes left. Panics if this assumption is false.
    pub fn next(&mut self, graph: &Graph) -> Option<PeerNode> {
        self.next_impl(AssertingGraphRef { graph })
    }

    /// Returns the next peer node, expanding the graph if necessary, or `None` if there are no peer nodes left.
    pub fn ensure_next(&mut self, graph: &mut Graph) -> Option<PeerNode> {
        self.next_impl(ExpandingGraphRef { graph })
    }
}

/// All paths that are compatible with the given parent path of length 1
static DEPTH1_CHILD_PATHS: LazyLock<[ArrayVec<Side, 5>; Side::VALUES.len()]> =
    LazyLock::new(|| {
        Side::VALUES.map(|parent_side| {
            // The main constraint is that all parent sides need to be adjacent to all child sides.
            let mut path_list: ArrayVec<Side, 5> = ArrayVec::new();
            for child_side in Side::iter() {
                if !child_side.adjacent_to(parent_side) {
                    continue;
                }
                path_list.push(child_side);
            }
            path_list
        })
    });

/// All paths that are compatible with the given parent path of length 2
static DEPTH2_CHILD_PATHS: LazyLock<
    [[ArrayVec<[Side; 2], 2>; Side::VALUES.len()]; Side::VALUES.len()],
> = LazyLock::new(|| {
    Side::VALUES.map(|parent_side0| {
        Side::VALUES.map(|parent_side1| {
            let mut path_list: ArrayVec<[Side; 2], 2> = ArrayVec::new();
            if parent_side0 == parent_side1 {
                // Backtracking parent paths are irrelevant and may result in more child paths than
                // can fit in the ArrayVec, so skip these.
                return path_list;
            }
            // The main constraint is that all parent sides need to be adjacent to all child sides.
            for child_side0 in Side::iter() {
                if !child_side0.adjacent_to(parent_side0) || !child_side0.adjacent_to(parent_side1)
                {
                    // Child paths need to have both parts adjacent to parent paths.
                    continue;
                }
                for child_side1 in Side::iter() {
                    // To avoid redundancies, only look at child paths that obey shortlex rules.
                    if child_side0 == child_side1 {
                        // Child path backtracks and should be discounted.
                        continue;
                    }
                    if child_side0.adjacent_to(child_side1)
                        && (child_side0 as usize) > (child_side1 as usize)
                    {
                        // There is a lexicographically earlier child path, so this should be discounted.
                        continue;
                    }
                    if !child_side1.adjacent_to(parent_side0)
                        || !child_side1.adjacent_to(parent_side1)
                    {
                        // Child paths need to have both parts adjacent to parent paths.
                        continue;
                    }
                    path_list.push([child_side0, child_side1]);
                }
            }
            path_list
        })
    })
});

/// A reference to the graph used by `PeerTraverser` to decide how to handle not-yet-created nodes
trait GraphRef: AsRef<Graph> {
    fn length(&self, node: NodeId) -> u32;
    fn neighbor(&mut self, node: NodeId, side: Side) -> NodeId;
}

/// A `GraphRef` that asserts that all the nodes it needs already exist
struct AssertingGraphRef<'a> {
    graph: &'a Graph,
}

impl AsRef<Graph> for AssertingGraphRef<'_> {
    #[inline]
    fn as_ref(&self) -> &Graph {
        self.graph
    }
}

impl GraphRef for AssertingGraphRef<'_> {
    #[inline]
    fn length(&self, node: NodeId) -> u32 {
        self.graph.length(node)
    }

    #[inline]
    fn neighbor(&mut self, node: NodeId, side: Side) -> NodeId {
        self.graph.neighbor(node, side).unwrap()
    }
}

/// A `GraphRef` that expands the graph as necessary
struct ExpandingGraphRef<'a> {
    graph: &'a mut Graph,
}

impl GraphRef for ExpandingGraphRef<'_> {
    #[inline]
    fn length(&self, node: NodeId) -> u32 {
        self.graph.length(node)
    }

    #[inline]
    fn neighbor(&mut self, node: NodeId, side: Side) -> NodeId {
        self.graph.ensure_neighbor(node, side)
    }
}

impl AsRef<Graph> for ExpandingGraphRef<'_> {
    #[inline]
    fn as_ref(&self) -> &Graph {
        self.graph
    }
}

#[cfg(test)]
mod tests {
    use fxhash::FxHashSet;

    use super::*;

    // Returns the `NodeId` corresponding to the given path
    fn node_from_path(
        graph: &mut Graph,
        start_node: NodeId,
        path: impl IntoIterator<Item = Side>,
    ) -> NodeId {
        let mut current_node = start_node;
        for side in path {
            current_node = graph.ensure_neighbor(current_node, side);
        }
        current_node
    }

    #[test]
    fn peer_traverser_example() {
        let mut graph = Graph::new(1);
        let base_node = node_from_path(
            &mut graph,
            NodeId::ROOT,
            [Side::B, Side::D, Side::C, Side::A],
        );

        let expected_paths: &[(&[Side], &[Side])] = &[
            (&[Side::A], &[Side::B]),
            (&[Side::A], &[Side::E]),
            (&[Side::A], &[Side::I]),
            (&[Side::C], &[Side::B]),
            (&[Side::C], &[Side::F]),
            (&[Side::C], &[Side::H]),
            (&[Side::D], &[Side::H]),
            (&[Side::D], &[Side::I]),
            (&[Side::D], &[Side::K]),
            (&[Side::C, Side::A], &[Side::B, Side::D]),
            (&[Side::D, Side::A], &[Side::I, Side::C]),
            (&[Side::D, Side::C], &[Side::H, Side::A]),
        ];

        let mut traverser = PeerTraverser::new(base_node);
        for expected_path in expected_paths {
            let peer = traverser.ensure_next(&mut graph).unwrap();
            assert_eq!(
                peer.path_from_peer().collect::<Vec<_>>(),
                expected_path.0.to_vec(),
            );
            assert_eq!(
                peer.path_from_base().collect::<Vec<_>>(),
                expected_path.1.to_vec(),
            );
        }

        assert!(traverser.ensure_next(&mut graph).is_none());
    }

    #[test]
    fn peer_definition_holds() {
        let mut graph = Graph::new(1);
        let base_node = node_from_path(
            &mut graph,
            NodeId::ROOT,
            [Side::B, Side::D, Side::C, Side::A],
        );
        let mut found_peer_nodes = FxHashSet::default();
        let mut traverser = PeerTraverser::new(base_node);
        while let Some(peer) = traverser.ensure_next(&mut graph) {
            let peer_node = peer.node();

            assert!(
                found_peer_nodes.insert(peer_node),
                "The same peer node must not be returned more than once."
            );

            let destination_from_base =
                node_from_path(&mut graph, base_node, peer.path_from_base());
            let destination_from_peer =
                node_from_path(&mut graph, peer_node, peer.path_from_peer());

            assert_eq!(
                graph.length(base_node),
                graph.length(peer_node),
                "The base and peer nodes must have the same depth in the graph."
            );
            assert_eq!(
                graph.length(base_node) + peer.path_from_base().len() as u32,
                graph.length(destination_from_base),
                "path_from_base must not backtrack to a parent node."
            );
            assert_eq!(
                graph.length(peer_node) + peer.path_from_peer().len() as u32,
                graph.length(destination_from_peer),
                "path_from_peer must not backtrack to a parent node."
            );
            assert_eq!(
                destination_from_base, destination_from_peer,
                "path_from_base and path_from_peer must lead to the same node."
            );
        }
    }

    #[test]
    fn alternative_implementation() {
        // Tests that the traverser's implementation is equivalent to a much simpler implementation that returns
        // everything at once instead of maintaining a state machine.
        let mut graph = Graph::new(1);
        let base_node = node_from_path(
            &mut graph,
            NodeId::ROOT,
            [Side::B, Side::D, Side::C, Side::A],
        );
        let mut traverser = PeerTraverser::new(base_node);

        // Depth 1 paths
        for (parent_side, parent_node) in graph.descenders(base_node) {
            for &child_side in &DEPTH1_CHILD_PATHS[parent_side as usize] {
                let peer_node = graph.ensure_neighbor(parent_node, child_side);
                if graph.length(peer_node) == graph.length(base_node) {
                    assert_peer_node_eq(
                        PeerNode {
                            node_id: peer_node,
                            parent_path: ArrayVec::from_iter([parent_side]),
                            child_path: ArrayVec::from_iter([child_side]),
                        },
                        traverser.ensure_next(&mut graph).unwrap(),
                    );
                }
            }
        }

        // Depth 2 paths
        for (parent_side0, parent_node0) in graph.descenders(base_node) {
            for (parent_side1, parent_node1) in graph.descenders(parent_node0) {
                // Avoid redundancies by enforcing shortlex order
                if parent_side1.adjacent_to(parent_side0)
                    && (parent_side1 as usize) < (parent_side0 as usize)
                {
                    continue;
                }
                for &child_sides in
                    &DEPTH2_CHILD_PATHS[parent_side0 as usize][parent_side1 as usize]
                {
                    let peer_node_parent = graph.ensure_neighbor(parent_node1, child_sides[0]);
                    let peer_node = graph.ensure_neighbor(peer_node_parent, child_sides[1]);
                    if graph.length(peer_node) == graph.length(base_node) {
                        assert_peer_node_eq(
                            PeerNode {
                                node_id: peer_node,
                                parent_path: ArrayVec::from_iter([parent_side0, parent_side1]),
                                child_path: ArrayVec::from_iter(child_sides),
                            },
                            traverser.ensure_next(&mut graph).unwrap(),
                        );
                    }
                }
            }
        }

        assert!(traverser.ensure_next(&mut graph).is_none());
    }

    fn assert_peer_node_eq(left: PeerNode, right: PeerNode) {
        assert_eq!(left.node_id, right.node_id);
        assert_eq!(left.parent_path, right.parent_path);
        assert_eq!(left.child_path, right.child_path);
    }
}
