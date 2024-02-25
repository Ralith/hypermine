use crate::{
    chunk_collision::chunk_sphere_cast,
    collision_math::Ray,
    graph::Graph,
    math,
    node::{Chunk, ChunkId},
    proto::Position,
    traversal::RayTraverser,
};

/// Performs sphere casting (swept collision query) against the voxels in the `Graph`
///
/// The `ray` parameter and any resulting hit normals are given in the local coordinate system of `position`.
///
/// The `tanh_distance` is the hyperbolic tangent of the cast_distance, or the distance along the ray to check for hits.
///
/// This function may return a `Err(OutOfBounds)` if not enough chunks are generated, even if the ray never reaches an
/// ungenerated chunk. To prevent these errors, make sure that the distance between the ray's start point and the center of
/// the closest node with ungenerated chunks is greater than `cast_distance + collider_radius + dodeca::BOUNDING_SPHERE_RADIUS`
pub fn sphere_cast(
    collider_radius: f32,
    graph: &Graph,
    position: &Position,
    ray: &Ray,
    mut tanh_distance: f32,
) -> Result<Option<GraphCastHit>, OutOfBounds> {
    // A collision check is assumed to be a miss until a collision is found.
    // This `hit` variable gets updated over time before being returned.
    let mut hit: Option<GraphCastHit> = None;

    let mut traverser = RayTraverser::new(graph, *position, ray, collider_radius);
    while let Some((chunk, transform)) = traverser.next(tanh_distance) {
        let Some(chunk) = chunk else {
            // Collision checking on chunk outside of graph
            return Err(OutOfBounds);
        };
        let Chunk::Populated {
            voxels: ref voxel_data,
            ..
        } = graph[chunk]
        else {
            // Collision checking on unpopulated chunk
            return Err(OutOfBounds);
        };

        // Check collision within a single chunk
        hit = chunk_sphere_cast(
            collider_radius,
            voxel_data,
            graph.layout(),
            &(transform * ray),
            tanh_distance,
        )
        .map_or(hit, |hit| {
            tanh_distance = hit.tanh_distance;
            Some(GraphCastHit {
                tanh_distance: hit.tanh_distance,
                chunk,
                normal: math::mtranspose(&transform) * hit.normal,
            })
        });
    }

    Ok(hit)
}

#[derive(Debug)]
pub struct OutOfBounds;

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

#[cfg(test)]
mod tests {
    use crate::{
        collision_math::Ray,
        dodeca::{self, Side, Vertex},
        graph::{Graph, NodeId},
        node::{populate_fresh_nodes, Coords, VoxelData},
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

        /// The coordinates of the voxel
        coords: Coords,
    }

    impl VoxelLocation<'_> {
        fn new(node_path: &[Side], vertex: Vertex, coords: [u8; 3]) -> VoxelLocation<'_> {
            VoxelLocation {
                node_path,
                vertex,
                coords: Coords(coords),
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
            let dimension: u8 = 12;
            let mut graph = Graph::new(dimension);
            let graph_radius = 3.0;

            // Set up a graph with void chunks
            ensure_nearby(&mut graph, &Position::origin(), graph_radius);
            populate_fresh_nodes(&mut graph);
            for (node, _) in nearby_nodes(&graph, &Position::origin(), graph_radius) {
                for vertex in dodeca::Vertex::iter() {
                    graph[ChunkId::new(node, vertex)] = Chunk::Populated {
                        voxels: VoxelData::Solid(Material::Void),
                        modified: false,
                        surface: None,
                        old_surface: None,
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
                    |transform: na::Matrix4<f32>, side| transform * side.reflection(),
                ) * self.chosen_voxel.vertex.dual_to_node();

            let dual_to_grid_factor = graph.layout().dual_to_grid_factor();
            let ray_target = chosen_chunk_transform
                * math::lorentz_normalize(&na::Vector4::new(
                    self.chosen_chunk_relative_grid_ray_end[0] / dual_to_grid_factor,
                    self.chosen_chunk_relative_grid_ray_end[1] / dual_to_grid_factor,
                    self.chosen_chunk_relative_grid_ray_end[2] / dual_to_grid_factor,
                    1.0,
                ));

            let ray_position = Vertex::A.dual_to_node()
                * math::lorentz_normalize(&na::Vector4::new(
                    self.start_chunk_relative_grid_ray_start[0] / dual_to_grid_factor,
                    self.start_chunk_relative_grid_ray_start[1] / dual_to_grid_factor,
                    self.start_chunk_relative_grid_ray_start[2] / dual_to_grid_factor,
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

        fn populate_voxel(graph: &mut Graph, dimension: u8, voxel_location: &VoxelLocation) {
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
            voxel_data.data_mut(dimension)[voxel_location.coords.to_index(dimension)] =
                Material::Dirt;
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
            chosen_voxel: VoxelLocation::new(&[Side::G], Vertex::I, [2, 3, 5]),
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
                [0, 11, 11],
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
                [0, 11, 11],
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
            let mut chosen_voxel_coords = [0, 0, 0];
            chosen_voxel_coords[corresponding_axis] = 11;
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
                [0, 0, 0],
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
                [0, 4, 4],
            ),
            additional_populated_voxels: &[VoxelLocation::new(&[], Vertex::A, [0, 5, 4])],
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
            chosen_voxel: VoxelLocation::new(&[], Vertex::A, [0, 4, 4]),
            additional_populated_voxels: &[VoxelLocation::new(
                &[Vertex::A.canonical_sides()[0]],
                Vertex::A,
                [0, 5, 4],
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
        let dimension: u8 = 12;
        let mut graph = Graph::new(dimension);

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
                    modified: false,
                    surface: None,
                    old_surface: None,
                };
            }
        }

        // The node coordinates of the corner of the missing node
        let vertex_pos = Vertex::A.dual_to_node() * math::origin();

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
            &Position::origin(),
            &ray,
            distance.tanh(),
        );

        assert!(hit.is_ok());
    }
}
