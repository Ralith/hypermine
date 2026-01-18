use std::sync::LazyLock;

use arrayvec::ArrayVec;

use crate::{
    dodeca::{SIDE_COUNT, Side},
    graph::{Graph, NodeId},
};

/// Assumes the graph is expanded enough to traverse peer nodes and returns all peer nodes
/// for the given base node. Panics if this assumption is false. See documentation of `PeerNode`
/// for a definition of what a "peer node" is.
pub fn expect_peer_nodes(graph: &Graph, base_node: NodeId) -> Vec<PeerNode> {
    peer_nodes_impl(AssertingGraphRef { graph }, base_node)
}

/// Returns all peer nodes for the given base node, expanding the graph if necessary.  See
/// documentation of `PeerNode` for a definition of what a "peer node" is.
pub fn ensure_peer_nodes(graph: &mut Graph, base_node: NodeId) -> Vec<PeerNode> {
    peer_nodes_impl(ExpandingGraphRef { graph }, base_node)
}

/// Internal implementation of peer node traversal, using a `GraphRef` to be generic over
/// whether a mutable or immutable graph reference is available
fn peer_nodes_impl(mut graph: impl GraphRef, base_node: NodeId) -> Vec<PeerNode> {
    let mut nodes = Vec::new();

    // Depth 1 paths
    for parent_side in Side::iter() {
        let parent_node = graph.neighbor(base_node, parent_side);
        if graph.depth(parent_node) >= graph.depth(base_node) {
            continue;
        }
        for &child_side in &DEPTH1_CHILD_PATHS[parent_side as usize] {
            let peer_node = graph.neighbor(parent_node, child_side);
            if graph.depth(peer_node) == graph.depth(base_node) {
                nodes.push(PeerNode {
                    node_id: peer_node,
                    parent_path: ArrayVec::from_iter([parent_side]),
                    child_path: ArrayVec::from_iter([child_side]),
                });
            }
        }
    }

    // Depth 2 paths
    for (parent_side0, parent_node0) in graph.parents(base_node) {
        for (parent_side1, parent_node1) in graph.parents(parent_node0) {
            // Avoid redundancies by enforcing shortlex order
            if parent_side1.adjacent_to(parent_side0)
                && (parent_side1 as usize) < (parent_side0 as usize)
            {
                continue;
            }
            for &child_sides in &DEPTH2_CHILD_PATHS[parent_side0 as usize][parent_side1 as usize] {
                let peer_node_parent = graph.neighbor(parent_node1, child_sides[0]);
                let peer_node = graph.neighbor(peer_node_parent, child_sides[1]);
                if graph.depth(peer_node) == graph.depth(base_node) {
                    nodes.push(PeerNode {
                        node_id: peer_node,
                        parent_path: ArrayVec::from_iter([parent_side0, parent_side1]),
                        child_path: ArrayVec::from_iter(child_sides),
                    });
                }
            }
        }
    }

    nodes
}

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
    pub fn peer_to_shared(&self) -> impl ExactSizeIterator<Item = Side> + Clone + use<> {
        self.parent_path.clone().into_iter().rev()
    }

    /// The sequence of sides that takes you from the base node to the shared child node
    #[inline]
    pub fn base_to_shared(&self) -> impl ExactSizeIterator<Item = Side> + Clone + use<> {
        self.child_path.clone().into_iter()
    }
}

/// All paths that can potentially lead to a peer node after following the given parent path of length 1
static DEPTH1_CHILD_PATHS: LazyLock<[ArrayVec<Side, 5>; SIDE_COUNT]> = LazyLock::new(|| {
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

/// All paths that can potentially lead to a peer node after following the given parent path of length 2
static DEPTH2_CHILD_PATHS: LazyLock<[[ArrayVec<[Side; 2], 2>; SIDE_COUNT]; SIDE_COUNT]> =
    LazyLock::new(|| {
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
                    if !child_side0.adjacent_to(parent_side0)
                        || !child_side0.adjacent_to(parent_side1)
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
    fn depth(&self, node: NodeId) -> u32;
    fn neighbor(&mut self, node: NodeId, side: Side) -> NodeId;
    fn parents(&self, node: NodeId) -> impl ExactSizeIterator<Item = (Side, NodeId)> + use<Self>;
}

/// A `GraphRef` that asserts that all the nodes it needs already exist
struct AssertingGraphRef<'a> {
    graph: &'a Graph,
}

impl AsRef<Graph> for AssertingGraphRef<'_> {
    fn as_ref(&self) -> &Graph {
        self.graph
    }
}

impl<'a> GraphRef for AssertingGraphRef<'a> {
    fn depth(&self, node: NodeId) -> u32 {
        self.graph.depth(node)
    }

    fn neighbor(&mut self, node: NodeId, side: Side) -> NodeId {
        self.graph.neighbor(node, side).unwrap()
    }

    fn parents(&self, node: NodeId) -> impl ExactSizeIterator<Item = (Side, NodeId)> + use<'a> {
        self.graph.parents(node)
    }
}

/// A `GraphRef` that expands the graph as necessary
struct ExpandingGraphRef<'a> {
    graph: &'a mut Graph,
}

impl<'a> GraphRef for ExpandingGraphRef<'a> {
    fn depth(&self, node: NodeId) -> u32 {
        self.graph.depth(node)
    }

    fn neighbor(&mut self, node: NodeId, side: Side) -> NodeId {
        self.graph.ensure_neighbor(node, side)
    }

    fn parents(&self, node: NodeId) -> impl ExactSizeIterator<Item = (Side, NodeId)> + use<'a> {
        self.graph.parents(node)
    }
}

impl AsRef<Graph> for ExpandingGraphRef<'_> {
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

        let peers = ensure_peer_nodes(&mut graph, base_node);
        assert_eq!(peers.len(), expected_paths.len());
        for (peer, expected_path) in peers.into_iter().zip(expected_paths) {
            assert_eq!(
                peer.peer_to_shared().collect::<Vec<_>>(),
                expected_path.0.to_vec(),
            );
            assert_eq!(
                peer.base_to_shared().collect::<Vec<_>>(),
                expected_path.1.to_vec(),
            );
        }
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
        for peer in ensure_peer_nodes(&mut graph, base_node) {
            let peer_node = peer.node();

            assert!(
                found_peer_nodes.insert(peer_node),
                "The same peer node must not be returned more than once."
            );

            let destination_from_base =
                node_from_path(&mut graph, base_node, peer.base_to_shared());
            let destination_from_peer =
                node_from_path(&mut graph, peer_node, peer.peer_to_shared());

            assert_eq!(
                graph.depth(base_node),
                graph.depth(peer_node),
                "The base and peer nodes must have the same depth in the graph."
            );
            assert_eq!(
                graph.depth(base_node) + peer.base_to_shared().len() as u32,
                graph.depth(destination_from_base),
                "path_from_base must not backtrack to a parent node."
            );
            assert_eq!(
                graph.depth(peer_node) + peer.peer_to_shared().len() as u32,
                graph.depth(destination_from_peer),
                "path_from_peer must not backtrack to a parent node."
            );
            assert_eq!(
                destination_from_base, destination_from_peer,
                "path_from_base and path_from_peer must lead to the same node."
            );
        }
    }
}
