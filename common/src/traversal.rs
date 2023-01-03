use fxhash::FxHashSet;

use crate::{
    dodeca::Side,
    graph::{Graph, NodeId},
    math,
    proto::Position,
};

/// Ensure all nodes within `distance` of `start` exist
pub fn ensure_nearby<N>(graph: &mut Graph<N>, start: &Position, distance: f64) {
    let mut pending = Vec::<(NodeId, na::Matrix4<f64>)>::new();
    let mut visited = FxHashSet::<NodeId>::default();

    pending.push((start.node, na::Matrix4::identity()));
    visited.insert(start.node);
    let start_p = start.local.map(|x| x as f64) * math::origin();

    while let Some((node, current_transform)) = pending.pop() {
        for side in Side::iter() {
            let neighbor = graph.ensure_neighbor(node, side);
            if visited.contains(&neighbor) {
                continue;
            }
            visited.insert(neighbor);
            let neighbor_transform = current_transform * side.reflection();
            let neighbor_p = neighbor_transform * math::origin();
            if math::distance(&start_p, &neighbor_p) > distance {
                continue;
            }
            pending.push((neighbor, neighbor_transform));
        }
    }
}

/// Compute `start.node`-relative transforms of all nodes whose origins lie within `distance` of
/// `start`
pub fn nearby_nodes<N>(
    graph: &Graph<N>,
    start: &Position,
    distance: f64,
) -> Vec<(NodeId, na::Matrix4<f32>)> {
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
        let current_p = current.transform * math::origin();
        if math::distance(&start_p, &current_p) > distance {
            continue;
        }
        result.push((current.id, na::convert(current.transform)));

        for side in Side::iter() {
            let neighbor = match graph.neighbor(current.id, side) {
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
