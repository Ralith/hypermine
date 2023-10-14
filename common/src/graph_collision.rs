use std::collections::VecDeque;

use fxhash::FxHashSet;

use crate::{
    chunk_collision::chunk_sphere_cast,
    dodeca::{self, Vertex},
    graph::Graph,
    math,
    node::{Chunk, ChunkId, ChunkLayout},
    proto::Position,
};

/// Performs sphere casting (swept collision query) against the voxels in the `Graph`
///
/// The `ray` parameter and any resulting hit normals are given in the local coordinate system of `position`.
///
/// The `tanh_distance` is the hyperbolic tangent of the cast_distance, or the distance along the ray to check for hits.
///
/// This function may return a `SphereCastError` if not enough chunks are generated, even if the ray never reaches an
/// ungenerated chunk. To prevent these errors, make sure that the distance between the ray's start point and the center of
/// the closest node with ungenerated chunks is greater than `cast_distance + collider_radius + dodeca::BOUNDING_SPHERE_RADIUS`
pub fn sphere_cast(
    collider_radius: f32,
    graph: &Graph,
    layout: &ChunkLayout,
    position: &Position,
    ray: &Ray,
    tanh_distance: f32,
) -> Result<Option<GraphCastHit>, SphereCastError> {
    // A collision check is assumed to be a miss until a collision is found.
    // This `hit` variable gets updated over time before being returned.
    let mut hit: Option<GraphCastHit> = None;

    // Pick the vertex closest to position.local as the vertex of the chunk to use to start collision checking
    let start_vertex = Vertex::iter()
        .map(|v| {
            (
                v,
                (v.node_to_dual().cast::<f32>() * position.local * math::origin()).w,
            )
        })
        .min_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
        .unwrap()
        .0;

    // Start a breadth-first search of the graph's chunks, performing collision checks in each relevant chunk.
    // The `chunk_queue` contains ordered pairs containing the `ChunkId` and the transformation needed to switch
    // from the original node coordinates to the current chunk's node coordinates.
    let mut visited_chunks = FxHashSet::<ChunkId>::default();
    let mut chunk_queue: VecDeque<(ChunkId, na::Matrix4<f32>)> = VecDeque::new();
    chunk_queue.push_back((ChunkId::new(position.node, start_vertex), position.local));

    // Precalculate the chunk boundaries for collision purposes. If the collider goes outside these bounds,
    // the corresponding neighboring chunk will also be used for collision checking.
    let klein_lower_boundary = collider_radius.tanh();
    let klein_upper_boundary =
        ((Vertex::chunk_to_dual_factor() as f32).atanh() - collider_radius).tanh();

    // Breadth-first search loop
    while let Some((chunk, node_transform)) = chunk_queue.pop_front() {
        let Chunk::Populated {
            voxels: ref voxel_data,
            ..
        } = graph[chunk]
        else {
            // Collision checking on unpopulated chunk
            return Err(SphereCastError::OutOfBounds);
        };
        let local_ray = chunk.vertex.node_to_dual().cast::<f32>() * node_transform * ray;

        // Check collision within a single chunk
        let current_tanh_distance = hit.as_ref().map_or(tanh_distance, |hit| hit.tanh_distance);
        hit = chunk_sphere_cast(
            collider_radius,
            voxel_data,
            layout,
            &local_ray,
            current_tanh_distance,
        )
        .map_or(hit, |hit| {
            Some(GraphCastHit {
                tanh_distance: hit.tanh_distance,
                chunk,
                normal: math::mtranspose(&node_transform)
                    * chunk.vertex.dual_to_node().cast()
                    * hit.normal,
            })
        });

        // Compute the Klein-Beltrami coordinates of the ray segment's endpoints. To check whether neighboring chunks
        // are needed, we need to check whether the endpoints of the line segments lie outside the boundaries of the square
        // bounded by `klein_lower_boundary` and `klein_upper_boundary`.
        let klein_ray_start = na::Point3::from_homogeneous(local_ray.position).unwrap();
        let klein_ray_end =
            na::Point3::from_homogeneous(local_ray.ray_point(current_tanh_distance)).unwrap();

        // Add neighboring chunks as necessary based on a conservative AABB check, using one coordinate at a time.
        for axis in 0..3 {
            // Check for neighboring nodes
            if klein_ray_start[axis] <= klein_lower_boundary
                || klein_ray_end[axis] <= klein_lower_boundary
            {
                let side = chunk.vertex.canonical_sides()[axis];
                let next_node_transform = side.reflection().cast::<f32>() * node_transform;
                // Crude check to ensure that the neighboring chunk's node can be in the path of the ray. For simplicity, this
                // check treats each node as a sphere and assumes the ray is pointed directly towards its center. The check is
                // needed because chunk generation uses this approximation, and this check is not guaranteed to pass near corners
                // because the AABB check can have false positives.
                let ray_node_distance = (next_node_transform * ray.position).w.acosh();
                let ray_length = current_tanh_distance.atanh();
                if ray_node_distance - ray_length - collider_radius
                    > dodeca::BOUNDING_SPHERE_RADIUS as f32
                {
                    // Ray cannot intersect node
                    continue;
                }
                // If we have to do collision checking on nodes that don't exist in the graph, we cannot have a conclusive result.
                let Some(neighbor) = graph.neighbor(chunk.node, side) else {
                    // Collision checking on nonexistent node
                    return Err(SphereCastError::OutOfBounds);
                };
                // Assuming everything goes well, add the new chunk to the queue.
                let next_chunk = ChunkId::new(neighbor, chunk.vertex);
                if visited_chunks.insert(next_chunk) {
                    chunk_queue.push_back((next_chunk, next_node_transform));
                }
            }

            // Check for neighboring chunks within the same node
            if klein_ray_start[axis] >= klein_upper_boundary
                || klein_ray_end[axis] >= klein_upper_boundary
            {
                let vertex = chunk.vertex.adjacent_vertices()[axis];
                let next_chunk = ChunkId::new(chunk.node, vertex);
                if visited_chunks.insert(next_chunk) {
                    chunk_queue.push_back((next_chunk, node_transform));
                }
            }
        }
    }

    Ok(hit)
}

#[derive(Debug)]
pub enum SphereCastError {
    OutOfBounds,
}

/// Information about the intersection at the end of a ray segment.
#[derive(Debug)]
pub struct GraphCastHit {
    /// The tanh of the distance traveled along the ray to result in this hit.
    pub tanh_distance: f32,

    /// Which chunk in the graph the hit occurred in
    pub chunk: ChunkId,

    /// Represents the normal vector of the hit surface in the original coordinate system
    /// of the sphere casting. To get the actual normal vector, project it so that it is orthogonal
    /// to the endpoint in Lorentz space.
    pub normal: na::Vector4<f32>,
}

/// A ray in hyperbolic space. The fields must be lorentz normalized, with `mip(position, position) == -1`,
/// `mip(direction, direction) == 1`, and `mip(position, direction) == 0`.
#[derive(Debug)]
pub struct Ray {
    pub position: na::Vector4<f32>,
    pub direction: na::Vector4<f32>,
}

impl Ray {
    pub fn new(position: na::Vector4<f32>, direction: na::Vector4<f32>) -> Ray {
        Ray {
            position,
            direction,
        }
    }

    /// Returns a point along this ray `atanh(tanh_distance)` units away from the origin. This point
    /// is _not_ lorentz normalized.
    pub fn ray_point(&self, tanh_distance: f32) -> na::Vector4<f32> {
        self.position + self.direction * tanh_distance
    }
}

impl std::ops::Mul<&Ray> for na::Matrix4<f32> {
    type Output = Ray;

    #[inline]
    fn mul(self, rhs: &Ray) -> Self::Output {
        Ray {
            position: self * rhs.position,
            direction: self * rhs.direction,
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::{
        dodeca::{Side, Vertex},
        graph::{Graph, NodeId},
        node::{populate_fresh_nodes, VoxelData},
        proto::Position,
        traversal::{ensure_nearby, nearby_nodes},
        world::Material,
    };

    use super::*;

    /// Convenience struct used to locate a particular voxel that should be solid in a test case.
    struct VoxelLocation<'a> {
        /// Path from the origin node to the voxel
        node_path: &'a [Side],

        /// Which chunk in the given node the voxel is in
        vertex: Vertex,

        /// The coordinates of the voxel, including margins
        coords: [usize; 3],
    }

    impl VoxelLocation<'_> {
        fn new(node_path: &[Side], vertex: Vertex, coords: [usize; 3]) -> VoxelLocation<'_> {
            VoxelLocation {
                node_path,
                vertex,
                coords,
            }
        }
    }

    struct SphereCastExampleTestCase<'a> {
        /// Which voxel the test case focuses on. Also determines the coordinate system of the ray.
        /// Any detected collision is expected to be on this voxel.
        chosen_voxel: VoxelLocation<'a>,

        /// Which voxels should be solid in the test case
        additional_populated_voxels: &'a [VoxelLocation<'a>],

        /// Grid coordinates of ray's start position relative to the root's "A" chunk
        start_chunk_relative_grid_ray_start: [f32; 3],

        /// Grid coordinates of ray's end position relative to chunk given by the chosen node and vertex
        chosen_chunk_relative_grid_ray_end: [f32; 3],

        /// What to use as the collider radius for shape casting
        collider_radius: f32,

        /// Amount to increase (or decrease) the ray's length compared to ending it at grid_ray_end
        ray_length_modifier: f32,

        /// Whether a collision should occur for the test to pass
        collision_expected: bool,
    }

    impl SphereCastExampleTestCase<'_> {
        fn execute(self) {
            let dimension: usize = 12;
            let layout = ChunkLayout::new(dimension);
            let mut graph = Graph::new();
            let graph_radius = 3.0;

            // Set up a graph with void chunks
            ensure_nearby(&mut graph, &Position::origin(), graph_radius);
            populate_fresh_nodes(&mut graph);
            for (node, _) in nearby_nodes(&graph, &Position::origin(), graph_radius) {
                for vertex in dodeca::Vertex::iter() {
                    graph[ChunkId::new(node, vertex)] = Chunk::Populated {
                        voxels: VoxelData::Solid(Material::Void),
                        surface: None,
                    };
                }
            }

            Self::populate_voxel(&mut graph, dimension, &self.chosen_voxel);

            for voxel in self.additional_populated_voxels {
                Self::populate_voxel(&mut graph, dimension, voxel);
            }

            // Find the transform of the chosen chunk
            let chosen_chunk_transform: na::Matrix4<f32> =
                self.chosen_voxel.node_path.iter().fold(
                    na::Matrix4::identity(),
                    |transform: na::Matrix4<f32>, side| transform * side.reflection().cast::<f32>(),
                ) * self.chosen_voxel.vertex.dual_to_node().cast();

            let ray_target = chosen_chunk_transform
                * math::lorentz_normalize(&na::Vector4::new(
                    self.chosen_chunk_relative_grid_ray_end[0] / layout.dual_to_grid_factor(),
                    self.chosen_chunk_relative_grid_ray_end[1] / layout.dual_to_grid_factor(),
                    self.chosen_chunk_relative_grid_ray_end[2] / layout.dual_to_grid_factor(),
                    1.0,
                ));

            let ray_position = Vertex::A.dual_to_node().cast()
                * math::lorentz_normalize(&na::Vector4::new(
                    self.start_chunk_relative_grid_ray_start[0] / layout.dual_to_grid_factor(),
                    self.start_chunk_relative_grid_ray_start[1] / layout.dual_to_grid_factor(),
                    self.start_chunk_relative_grid_ray_start[2] / layout.dual_to_grid_factor(),
                    1.0,
                ));
            let ray_direction = ray_target - ray_position;

            let ray = Ray::new(
                ray_position,
                math::lorentz_normalize(
                    &(ray_direction + ray_position * math::mip(&ray_position, &ray_direction)),
                ),
            );

            let tanh_distance = ((-math::mip(&ray_position, &ray_target)).acosh()
                + self.ray_length_modifier)
                .tanh();

            let hit = sphere_cast(
                self.collider_radius,
                &graph,
                &layout,
                &Position::origin(),
                &ray,
                tanh_distance,
            )
            .expect("conclusive collision result");

            if self.collision_expected {
                assert!(hit.is_some(), "no collision detected");
                assert_eq!(
                    hit.as_ref().unwrap().chunk,
                    Self::get_voxel_chunk(&graph, &self.chosen_voxel),
                    "collision occurred in wrong chunk"
                );
                assert!(
                    math::mip(&hit.as_ref().unwrap().normal, &ray.direction) < 0.0,
                    "normal is facing the wrong way"
                );
            } else {
                assert!(hit.is_none(), "unexpected collision detected");
            }
        }

        fn populate_voxel(graph: &mut Graph, dimension: usize, voxel_location: &VoxelLocation) {
            // Find the ChunkId of the given chunk
            let chunk = ChunkId::new(
                voxel_location
                    .node_path
                    .iter()
                    .fold(NodeId::ROOT, |node, &side| {
                        graph.neighbor(node, side).unwrap()
                    }),
                voxel_location.vertex,
            );
            let Chunk::Populated {
                voxels: voxel_data, ..
            } = graph.get_chunk_mut(chunk).unwrap()
            else {
                panic!("All chunks should be populated.");
            };

            // Populate the given voxel with dirt.
            voxel_data.data_mut(dimension as u8)[voxel_location.coords[0]
                + voxel_location.coords[1] * (dimension + 2)
                + voxel_location.coords[2] * (dimension + 2).pow(2)] = Material::Dirt;
        }

        fn get_voxel_chunk(graph: &Graph, voxel_location: &VoxelLocation) -> ChunkId {
            ChunkId::new(
                voxel_location
                    .node_path
                    .iter()
                    .fold(NodeId::ROOT, |node, &side| {
                        graph.neighbor(node, side).unwrap()
                    }),
                voxel_location.vertex,
            )
        }
    }

    /// Checks that `sphere_cast` behaves as expected under normal circumstances.
    #[test]
    fn sphere_cast_examples() {
        // Basic test case
        SphereCastExampleTestCase {
            chosen_voxel: VoxelLocation::new(&[Side::G], Vertex::I, [3, 4, 6]),
            additional_populated_voxels: &[],
            start_chunk_relative_grid_ray_start: [12.0, 12.0, 12.0], // Node center
            chosen_chunk_relative_grid_ray_end: [2.5, 3.5, 5.5],
            collider_radius: 0.02,
            ray_length_modifier: 0.0,
            collision_expected: true,
        }
        .execute();

        // Barely touching a neighboring node
        SphereCastExampleTestCase {
            chosen_voxel: VoxelLocation::new(
                &[Vertex::B.canonical_sides()[0]],
                Vertex::B,
                [1, 12, 12],
            ),
            additional_populated_voxels: &[],
            start_chunk_relative_grid_ray_start: [12.0, 12.0, 12.0], // Node center
            chosen_chunk_relative_grid_ray_end: [0.0, 12.0, 12.0],
            collider_radius: 0.02,
            ray_length_modifier: -0.019,
            collision_expected: true,
        }
        .execute();

        // Barely not touching a neighboring node
        SphereCastExampleTestCase {
            chosen_voxel: VoxelLocation::new(
                &[Vertex::B.canonical_sides()[0]],
                Vertex::B,
                [1, 12, 12],
            ),
            additional_populated_voxels: &[],
            start_chunk_relative_grid_ray_start: [12.0, 12.0, 12.0], // Node center
            chosen_chunk_relative_grid_ray_end: [0.0, 12.0, 12.0],
            collider_radius: 0.02,
            ray_length_modifier: -0.021,
            collision_expected: false,
        }
        .execute();

        // Barely touching a neighboring vertex
        {
            // This test case requires a bit of extra logic because getting the voxel coordinates
            // adjacent to a voxel in a neighboring chunk requires inspecting the canonical side
            // order of both vertices.
            let chosen_vertex = Vertex::A.adjacent_vertices()[0];
            let corresponding_axis = chosen_vertex
                .canonical_sides()
                .iter()
                .position(|side| !Vertex::A.canonical_sides().contains(side))
                .unwrap();
            let mut chosen_voxel_coords = [1, 1, 1];
            chosen_voxel_coords[corresponding_axis] = 12;
            let mut grid_ray_end = [0.0, 0.0, 0.0];
            grid_ray_end[corresponding_axis] = 12.0;
            SphereCastExampleTestCase {
                chosen_voxel: VoxelLocation::new(&[], chosen_vertex, chosen_voxel_coords),
                additional_populated_voxels: &[],
                start_chunk_relative_grid_ray_start: [0.0, 0.0, 0.0], // Node's A-vertex corner
                chosen_chunk_relative_grid_ray_end: grid_ray_end,
                collider_radius: 0.02,
                ray_length_modifier: -0.019,
                collision_expected: true,
            }
            .execute();
        }

        // Barely touching a node opposite the original node at a corner
        SphereCastExampleTestCase {
            chosen_voxel: VoxelLocation::new(
                &[
                    Vertex::D.canonical_sides()[0],
                    Vertex::D.canonical_sides()[1],
                    Vertex::D.canonical_sides()[2],
                ],
                Vertex::D,
                [1, 1, 1],
            ),
            additional_populated_voxels: &[],
            start_chunk_relative_grid_ray_start: [12.0, 12.0, 12.0], // Node center
            chosen_chunk_relative_grid_ray_end: [0.0, 0.0, 0.0],
            collider_radius: 0.02,
            ray_length_modifier: -0.019,
            collision_expected: true,
        }
        .execute();

        // Colliding with a neighboring node's voxel before the center node's voxel
        SphereCastExampleTestCase {
            chosen_voxel: VoxelLocation::new(
                &[Vertex::A.canonical_sides()[0]],
                Vertex::A,
                [1, 5, 5],
            ),
            additional_populated_voxels: &[VoxelLocation::new(&[], Vertex::A, [1, 6, 5])],
            // Because we use the "A" vertex, the two coordinate systems below coincide for x = 0.0
            start_chunk_relative_grid_ray_start: [0.0, 3.0, 4.5],
            chosen_chunk_relative_grid_ray_end: [0.0, 8.0, 4.5],
            collider_radius: 0.02,
            ray_length_modifier: 0.0,
            collision_expected: true,
        }
        .execute();

        // Colliding with the center node's voxel before a neighboring node's voxel
        SphereCastExampleTestCase {
            chosen_voxel: VoxelLocation::new(&[], Vertex::A, [1, 5, 5]),
            additional_populated_voxels: &[VoxelLocation::new(
                &[Vertex::A.canonical_sides()[0]],
                Vertex::A,
                [1, 6, 5],
            )],
            start_chunk_relative_grid_ray_start: [0.0, 3.0, 4.5],
            chosen_chunk_relative_grid_ray_end: [0.0, 8.0, 4.5],
            collider_radius: 0.02,
            ray_length_modifier: 0.0,
            collision_expected: true,
        }
        .execute();
    }

    /// Tests that a sphere cast that gets close to the corner of an unloaded chunk does not throw an error as
    /// long as the contract for sphere_cast is upheld.
    #[test]
    fn sphere_cast_near_unloaded_chunk() {
        let dimension: usize = 12;
        let layout = ChunkLayout::new(dimension);
        let mut graph = Graph::new();

        let sides = Vertex::A.canonical_sides();

        // Add six nodes surrounding the origin's Vertex::A to total 7 out of 8 nodes.
        // Only the far corner is missing.
        let first_neighbors = [
            graph.ensure_neighbor(NodeId::ROOT, sides[0]),
            graph.ensure_neighbor(NodeId::ROOT, sides[1]),
            graph.ensure_neighbor(NodeId::ROOT, sides[2]),
        ];
        let second_neighbors = [
            graph.ensure_neighbor(first_neighbors[0], sides[1]),
            graph.ensure_neighbor(first_neighbors[1], sides[2]),
            graph.ensure_neighbor(first_neighbors[2], sides[0]),
        ];

        // Populate all graph nodes
        populate_fresh_nodes(&mut graph);
        for node in [
            &[NodeId::ROOT],
            first_neighbors.as_slice(),
            second_neighbors.as_slice(),
        ]
        .concat()
        {
            for vertex in dodeca::Vertex::iter() {
                graph[ChunkId::new(node, vertex)] = Chunk::Populated {
                    voxels: VoxelData::Solid(Material::Void),
                    surface: None,
                };
            }
        }

        // The node coordinates of the corner of the missing node
        let vertex_pos = Vertex::A.dual_to_node().cast::<f32>() * math::origin();

        // Use a ray starting from the origin. The direction vector is vertex_pos with the w coordinate
        // set to 0 and normalized
        let ray = Ray::new(
            math::origin(),
            (vertex_pos - na::Vector4::w() * vertex_pos.w).normalize(),
        );
        let sphere_radius = 0.1;

        // Use a distance slightly less than the maximum possible before an error would occur.
        let distance = vertex_pos.w.acosh() - sphere_radius - 1e-4;

        let hit = sphere_cast(
            sphere_radius,
            &graph,
            &layout,
            &Position::origin(),
            &ray,
            distance.tanh(),
        );

        assert!(hit.is_ok());
    }
}
