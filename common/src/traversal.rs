use std::collections::VecDeque;

use fxhash::FxHashSet;

use crate::{
    collision_math::Ray,
    dodeca::{self, Side, Vertex},
    graph::{Graph, NodeId},
    math,
    math::{MVector,MIsometry},
    node::ChunkId,
    proto::Position,
};

/// Ensure all nodes within `distance` of `start` exist
<<<<<<< HEAD
pub fn ensure_nearby(graph: &mut Graph, start: &Position, distance: f64) {
    let mut pending = Vec::<(NodeId, MIsometry<f64>)>::new();
    let mut visited = FxHashSet::<NodeId>::default();

    pending.push((start.node, MIsometry::identity()));
    visited.insert(start.node);
    let start_p = start.local.map(|x| x as f64) * MVector::origin();
=======
pub fn ensure_nearby(graph: &mut Graph, start: &Position, distance: f32) {
    // We do a breadth-first instead of a depth-first traversal here to ensure that we take the
    // minimal path to each node. This greatly helps prevent error from accumulating due to
    // hundreds of transformations being composed.
    let mut pending = VecDeque::<(NodeId, na::Matrix4<f32>)>::new();
    let mut visited = FxHashSet::<NodeId>::default();

    pending.push_back((start.node, na::Matrix4::identity()));
    visited.insert(start.node);
    let start_p = start.local * math::origin();
>>>>>>> d49df99d371ca6354789deefff6900b1eea46533

    while let Some((node, current_transform)) = pending.pop_front() {
        for side in Side::iter() {
            let neighbor = graph.ensure_neighbor(node, side);
            if visited.contains(&neighbor) {
                continue;
            }
            visited.insert(neighbor);
<<<<<<< HEAD
            let neighbor_transform = current_transform * *side.reflection();
            let neighbor_p = neighbor_transform * MVector::origin();
            if math::distance(&start_p, &neighbor_p) > distance {
=======
            let neighbor_transform = current_transform * side.reflection();
            let neighbor_p = neighbor_transform * math::origin();
            if -math::mip(&start_p, &neighbor_p) > distance.cosh() {
>>>>>>> d49df99d371ca6354789deefff6900b1eea46533
                continue;
            }
            pending.push_back((neighbor, neighbor_transform));
        }
    }
}

/// Compute `start.node`-relative transforms of all nodes whose origins lie within `distance` of
/// `start`
pub fn nearby_nodes(
    graph: &Graph,
    start: &Position,
<<<<<<< HEAD
    distance: f64,
) -> Vec<(NodeId, MIsometry<f32>)> {
    struct PendingNode {
        id: NodeId,
        transform: MIsometry<f64>,
    }

    let mut result = Vec::new();
    let mut pending = Vec::<PendingNode>::new();
    let mut visited = FxHashSet::<NodeId>::default();
    let start_p = start.local.map(|x| x as f64) * MVector::origin();
=======
    distance: f32,
) -> Vec<(NodeId, na::Matrix4<f32>)> {
    struct PendingNode {
        id: NodeId,
        transform: na::Matrix4<f32>,
    }

    let mut result = Vec::new();
>>>>>>> d49df99d371ca6354789deefff6900b1eea46533

    // We do a breadth-first instead of a depth-first traversal here to ensure that we take the
    // minimal path to each node. This greatly helps prevent error from accumulating due to
    // hundreds of transformations being composed.
    let mut pending = VecDeque::<PendingNode>::new();
    let mut visited = FxHashSet::<NodeId>::default();
    let start_p = start.local * math::origin();

    pending.push_back(PendingNode {
        id: start.node,
        transform: MIsometry::identity(),
    });
    visited.insert(start.node);

<<<<<<< HEAD
    while let Some(current) = pending.pop() {
        let current_p = current.transform * MVector::origin();
        if math::distance(&start_p, &current_p) > distance {
=======
    while let Some(current) = pending.pop_front() {
        let current_p = current.transform * math::origin();
        if -math::mip(&start_p, &current_p) > distance.cosh() {
>>>>>>> d49df99d371ca6354789deefff6900b1eea46533
            continue;
        }
        result.push((current.id, current.transform.to_f32()));

        for side in Side::iter() {
            let neighbor = match graph.neighbor(current.id, side) {
                None => continue,
                Some(x) => x,
            };
            if visited.contains(&neighbor) {
                continue;
            }
            pending.push_back(PendingNode {
                id: neighbor,
                transform: current.transform * *side.reflection(),
            });
            visited.insert(neighbor);
        }
    }

    result
}

pub struct RayTraverser<'a> {
    graph: &'a Graph,
    ray: &'a Ray,
    radius: f32,
    /// Chunks that have already been added to `iterator_queue` and shouldn't be added again
    visited_chunks: FxHashSet<ChunkId>,
    /// Chunks that should be returned by `next` in the future
    iterator_queue: VecDeque<(Option<NodeId>, Vertex, MIsometry<f32>)>,
    /// Chunks whose neighbors should be queried in the future
    search_queue: VecDeque<(Option<NodeId>, Vertex, MIsometry<f32>)>,
    klein_lower_boundary: f32,
    klein_upper_boundary: f32,
}

impl<'a> RayTraverser<'a> {
    pub fn new(graph: &'a Graph, position: Position, ray: &'a Ray, radius: f32) -> Self {
        // Pick the vertex closest to position.local as the vertex of the chunk to use to start collision checking
        let mut closest_vertex = Vertex::A;
        let mut closest_vertex_cosh_distance = f32::INFINITY;
        for vertex in Vertex::iter() {
<<<<<<< HEAD
            let vertex_cosh_distance =
                (vertex.node_to_dual().to_f32() * position.local * MVector::origin()).w;
=======
            let vertex_cosh_distance = (vertex.node_to_dual() * position.local * math::origin()).w;
>>>>>>> d49df99d371ca6354789deefff6900b1eea46533
            if vertex_cosh_distance < closest_vertex_cosh_distance {
                closest_vertex = vertex;
                closest_vertex_cosh_distance = vertex_cosh_distance;
            }
        }
        let start_vertex = closest_vertex;

        let mut visited_chunks = FxHashSet::<ChunkId>::default();
        visited_chunks.insert(ChunkId::new(position.node, start_vertex));
        let mut iterator_queue = VecDeque::new();
        iterator_queue.push_back((Some(position.node), start_vertex, position.local));

        // Precalculate the chunk boundaries for collision purposes. If the collider goes outside these bounds,
        // the corresponding neighboring chunk will also be used for collision checking.
        let klein_lower_boundary = radius.tanh();
        let klein_upper_boundary = (Vertex::chunk_to_dual_factor().atanh() - radius).tanh();

        Self {
            graph,
            radius,
            ray,
            visited_chunks,
            iterator_queue,
            search_queue: VecDeque::new(),
            klein_lower_boundary,
            klein_upper_boundary,
        }
    }

    pub fn next(&mut self, tanh_distance: f32) -> Option<(Option<ChunkId>, MIsometry<f32>)> {
        loop {
            // Return the next entry that's queued up
            if let Some(entry @ (node, vertex, node_transform)) = self.iterator_queue.pop_front() {
                self.search_queue.push_back(entry);
                // Combine node and vertex, and convert node transform to chunk transform
                return Some((
                    node.map(|node| ChunkId::new(node, vertex)),
<<<<<<< HEAD
                    vertex.node_to_dual().to_f32() * node_transform,
=======
                    vertex.node_to_dual() * node_transform,
>>>>>>> d49df99d371ca6354789deefff6900b1eea46533
                ));
            }

            // If no entries are queued up, continue the breadth-first search to queue up new entries.
            let (node, vertex, node_transform) = self.search_queue.pop_front()?;
            let Some(node) = node else {
                // Cannot branch from chunks that are outside the graph
                continue;
            };

<<<<<<< HEAD
            let local_ray = vertex.node_to_dual().to_f32() * node_transform * self.ray;
=======
            let local_ray = vertex.node_to_dual() * node_transform * self.ray;
>>>>>>> d49df99d371ca6354789deefff6900b1eea46533

            // Compute the Klein-Beltrami coordinates of the ray segment's endpoints. To check whether neighboring chunks
            // are needed, we need to check whether the endpoints of the line segments lie outside the boundaries of the square
            // bounded by `klein_lower_boundary` and `klein_upper_boundary`.
            let klein_ray_start = na::Point3::from_homogeneous(local_ray.position).unwrap();
            let klein_ray_end =
                na::Point3::from_homogeneous(local_ray.ray_point(tanh_distance)).unwrap();

            // Add neighboring chunks as necessary based on a conservative AABB check, using one coordinate at a time.
            for axis in 0..3 {
                // Check for neighboring nodes
                if klein_ray_start[axis] <= self.klein_lower_boundary
                    || klein_ray_end[axis] <= self.klein_lower_boundary
                {
                    let side = vertex.canonical_sides()[axis];
<<<<<<< HEAD
                    let next_node_transform = side.reflection().to_f32() * node_transform;
=======
                    let next_node_transform = side.reflection() * node_transform;
>>>>>>> d49df99d371ca6354789deefff6900b1eea46533
                    // Crude check to ensure that the neighboring chunk's node can be in the path of the ray. For simplicity, this
                    // check treats each node as a sphere and assumes the ray is pointed directly towards its center. The check is
                    // needed because chunk generation uses this approximation, and this check is not guaranteed to pass near corners
                    // because the AABB check can have false positives.
                    let ray_node_distance = (next_node_transform * self.ray.position).w.acosh();
                    let ray_length = tanh_distance.atanh();
                    if ray_node_distance - ray_length - self.radius > dodeca::BOUNDING_SPHERE_RADIUS
                    {
                        // Ray cannot intersect node
                        continue;
                    }
                    // Add the new chunk to the queue.
                    if let Some(neighbor) = self.graph.neighbor(node, side) {
                        if self.visited_chunks.insert(ChunkId::new(neighbor, vertex)) {
                            self.iterator_queue.push_back((
                                Some(neighbor),
                                vertex,
                                next_node_transform,
                            ));
                        }
                    } else {
                        // There's `NodeId` for the requested chunk, so substitute `None`.
                        self.iterator_queue
                            .push_back((None, vertex, next_node_transform));
                    }
                }

                // Check for neighboring chunks within the same node
                if klein_ray_start[axis] >= self.klein_upper_boundary
                    || klein_ray_end[axis] >= self.klein_upper_boundary
                {
                    let next_vertex = vertex.adjacent_vertices()[axis];
                    if self.visited_chunks.insert(ChunkId::new(node, next_vertex)) {
                        self.iterator_queue
                            .push_back((Some(node), next_vertex, node_transform));
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_abs_diff_eq;

    use super::*;

    // Make sure that ensure_nearby and nearby_nodes finish even for a relatively large radius
    // and traverse the expected number of nodes
    #[test]
    fn traversal_functions_example() {
        let mut graph = Graph::new(1);
        ensure_nearby(&mut graph, &Position::origin(), 6.0);
        assert_abs_diff_eq!(graph.len(), 502079, epsilon = 50);

        // TODO: nearby_nodes has a stricter interpretation of distance than
        // ensure_nearby, resulting in far fewer nodes. Getting these two
        // functions to align may be a future performance improvement
        // opportunity.
        let nodes = nearby_nodes(&graph, &Position::origin(), 6.0);
        assert_abs_diff_eq!(nodes.len(), 60137, epsilon = 5);
    }
}
