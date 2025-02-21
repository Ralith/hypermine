/*the name of this module is pretty arbitrary at the moment*/

use std::ops::{Index, IndexMut};

use serde::{Deserialize, Serialize};

use crate::collision_math::Ray;
use crate::dodeca::Vertex;
use crate::graph::{Graph, NodeId};
use crate::lru_slab::SlotId;
use crate::proto::{BlockUpdate, Position, SerializedVoxelData};
use crate::voxel_math::{ChunkDirection, CoordAxis, CoordSign, Coords};
use crate::world::Material;
use crate::worldgen::NodeState;
use crate::{Chunks, margins};

/// Unique identifier for a single chunk (1/20 of a dodecahedron) in the graph
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct ChunkId {
    pub node: NodeId,
    pub vertex: Vertex,
}

impl ChunkId {
    pub fn new(node: NodeId, vertex: Vertex) -> Self {
        ChunkId { node, vertex }
    }
}

impl Graph {
    pub fn get_chunk_mut(&mut self, chunk: ChunkId) -> Option<&mut Chunk> {
        Some(&mut self.get_mut(chunk.node).as_mut()?.chunks[chunk.vertex])
    }

    pub fn get_chunk(&self, chunk: ChunkId) -> Option<&Chunk> {
        Some(&self.get(chunk.node).as_ref()?.chunks[chunk.vertex])
    }

    /// Returns the up-direction relative to the given position, or `None` if the
    /// position is in an unpopulated node.
    pub fn get_relative_up(&self, position: &Position) -> Option<na::UnitVector3<f32>> {
        let node = self.get(position.node).as_ref()?;
        Some(na::UnitVector3::new_normalize(
            (position.local.inverse() * node.state.up_direction()).xyz(),
        ))
    }

    /// Returns the ID of the chunk neighboring the given chunk on the specified
    /// cube face side, or `None` if it's on a node the graph hasn't populated.
    pub fn get_chunk_neighbor(
        &self,
        chunk: ChunkId,
        coord_axis: CoordAxis,
        coord_sign: CoordSign,
    ) -> Option<ChunkId> {
        match coord_sign {
            CoordSign::Plus => Some(ChunkId::new(
                chunk.node,
                chunk.vertex.adjacent_vertices()[coord_axis as usize],
            )),
            CoordSign::Minus => Some(ChunkId::new(
                self.neighbor(
                    chunk.node,
                    chunk.vertex.canonical_sides()[coord_axis as usize],
                )?,
                chunk.vertex,
            )),
        }
    }

    /// Returns the block (voxel) neighboring the given block on the specified
    /// cube face side, or `None` if it's on a node the graph hasn't populated.
    pub fn get_block_neighbor(
        &self,
        mut chunk: ChunkId,
        mut coords: Coords,
        coord_axis: CoordAxis,
        coord_sign: CoordSign,
    ) -> Option<(ChunkId, Coords)> {
        if coords[coord_axis] == Coords::boundary_coord(self.layout().dimension, coord_sign) {
            match coord_sign {
                CoordSign::Plus => {
                    coords = chunk.vertex.chunk_axis_permutations()[coord_axis as usize] * coords;
                    chunk.vertex = chunk.vertex.adjacent_vertices()[coord_axis as usize];
                }
                CoordSign::Minus => {
                    chunk.node = self.neighbor(
                        chunk.node,
                        chunk.vertex.canonical_sides()[coord_axis as usize],
                    )?;
                }
            }
        } else {
            coords[coord_axis] = coords[coord_axis].wrapping_add_signed(coord_sign as i8);
        }

        Some((chunk, coords))
    }

    /// Populates a chunk with the given voxel data and ensures that margins are correctly fixed up.
    pub fn populate_chunk(&mut self, chunk: ChunkId, mut voxels: VoxelData) {
        let dimension = self.layout().dimension;
        // Fix up margins for the chunk we're inserting along with any neighboring chunks
        for chunk_direction in ChunkDirection::iter() {
            let Some(Chunk::Populated {
                voxels: neighbor_voxels,
                surface: neighbor_surface,
                old_surface: neighbor_old_surface,
            }) = self
                .get_chunk_neighbor(chunk, chunk_direction.axis, chunk_direction.sign)
                .map(|chunk_id| &mut self[chunk_id])
            else {
                continue;
            };
            margins::fix_margins(
                dimension,
                chunk.vertex,
                &mut voxels,
                chunk_direction,
                neighbor_voxels,
            );
            *neighbor_old_surface = neighbor_surface.take().or(*neighbor_old_surface);
        }

        // After clearing any margins we needed to clear, we can now insert the data into the graph
        *self.get_chunk_mut(chunk).unwrap() = Chunk::Populated {
            voxels,
            surface: None,
            old_surface: None,
        };
    }

    /// Returns the material at the specified coordinates of the specified chunk, if the chunk is generated
    pub fn get_material(&self, chunk_id: ChunkId, coords: Coords) -> Option<Material> {
        let dimension = self.layout().dimension;

        let Some(Chunk::Populated { voxels, .. }) = self.get_chunk(chunk_id) else {
            return None;
        };
        Some(voxels.get(coords.to_index(dimension)))
    }

    /// Tries to update the block at the given position to the given material.
    /// Fails and returns false if the chunk is not populated yet.
    #[must_use]
    pub fn update_block(&mut self, block_update: &BlockUpdate) -> bool {
        let dimension = self.layout().dimension;

        // Update the block
        let Some(Chunk::Populated {
            voxels,
            surface,
            old_surface,
        }) = self.get_chunk_mut(block_update.chunk_id)
        else {
            return false;
        };
        let voxel = voxels
            .data_mut(dimension)
            .get_mut(block_update.coords.to_index(dimension))
            .expect("coords are in-bounds");

        *voxel = block_update.new_material;
        *old_surface = surface.take().or(*old_surface);

        for chunk_direction in ChunkDirection::iter() {
            margins::reconcile_margin_voxels(
                self,
                block_update.chunk_id,
                block_update.coords,
                chunk_direction,
            )
        }
        true
    }
}

impl Index<ChunkId> for Graph {
    type Output = Chunk;

    fn index(&self, chunk: ChunkId) -> &Chunk {
        self.get_chunk(chunk).unwrap()
    }
}

impl IndexMut<ChunkId> for Graph {
    fn index_mut(&mut self, chunk: ChunkId) -> &mut Chunk {
        self.get_chunk_mut(chunk).unwrap()
    }
}

/// A single dodecahedral node in the graph. All information related to world
/// generation and the blocks within the node, along with auxiliary information
/// used for rendering, is stored here.
pub struct Node {
    pub state: NodeState,
    /// We can only populate chunks which lie within a cube of populated nodes, so nodes on the edge
    /// of the graph always have some `None` chunks.
    pub chunks: Chunks<Chunk>,
}

/// Stores the actual voxel data of the chunk, along with metadata used for
/// rendering. This is an enum type to account for chunks that have not been
/// fully generated yet.
#[derive(Default)]
pub enum Chunk {
    /// Worldgen has not started running on this chunk yet. This can be for
    /// multiple reasons:
    /// - It was just added to the graph and hasn't had time to be processed.
    /// - All world generation threads are occupied, and it is not this chunk's
    ///   turn yet.
    /// - Not all nodes adjacent to this chunk are in the graph yet, so this
    ///   chunk lacks information necessary for world generation to proceed.
    #[default]
    Fresh,

    /// There is an active thread generating voxels for this chunk, but this
    /// chunk has not yet received the results from this thread.
    Generating,

    /// This chunk's voxels are fully generated and ready for use.
    Populated {
        /// The voxels present in the chunk
        voxels: VoxelData,

        /// A reference to the "mesh" used to render the chunk. Set to `None` if
        /// this mesh needs to be computed or recomputed.
        surface: Option<SlotId>,

        /// An outdated (but valid) reference to the "mesh" used to render the
        /// chunk. This is used to allow the mesh to still be rendered while it
        /// is being recomputed.
        old_surface: Option<SlotId>,
    },
}

/// The voxels present in a particular chunk, along a margin.
///
/// The margin consists of voxels of adjacent chunks, which is a necessary extra
/// piece of data needed to properly compute the rendered surface of the chunk.
pub enum VoxelData {
    /// All voxels, including the margin, are the same material. This data type
    /// is used to save storage space and processing, since such chunks do not
    /// need to be rendered at all.
    // TODO: This abstraction needs some work, as it doesn't account for areas
    // underground with a mixed set of materials, such as dirt and stone.
    Solid(Material),

    /// This chunk (or its margins) may consist of multiple materials, which are
    /// represented in the given boxed slice.
    Dense(Box<[Material]>),
}

impl VoxelData {
    pub fn data_mut(&mut self, dimension: u8) -> &mut [Material] {
        match *self {
            VoxelData::Dense(ref mut d) => d,
            VoxelData::Solid(mat) => {
                *self = VoxelData::Dense(vec![mat; (usize::from(dimension) + 2).pow(3)].into());
                self.data_mut(dimension)
            }
        }
    }

    pub fn get(&self, index: usize) -> Material {
        match *self {
            VoxelData::Dense(ref d) => d[index],
            VoxelData::Solid(mat) => mat,
        }
    }

    pub fn is_solid(&self) -> bool {
        match *self {
            VoxelData::Dense(_) => false,
            VoxelData::Solid(_) => true,
        }
    }

    /// Returns a `VoxelData` with void margins based on the given `SerializedVoxelData`, or `None` if
    /// the `SerializedVoxelData` came from a `VoxelData` with the wrong dimension or an unknown material.
    pub fn deserialize(serialized: &SerializedVoxelData, dimension: u8) -> Option<Self> {
        if serialized.inner.len() != usize::from(dimension).pow(3) * 2 {
            return None;
        }

        let mut materials = serialized
            .inner
            .chunks_exact(2)
            .map(|chunk| u16::from_le_bytes([chunk[0], chunk[1]]));

        let mut data = vec![Material::Void; (usize::from(dimension) + 2).pow(3)];
        for z in 0..dimension {
            for y in 0..dimension {
                for x in 0..dimension {
                    // We cannot use a linear copy here because `data` has margins, while `serialized.inner` does not.
                    data[Coords([x, y, z]).to_index(dimension)] =
                        materials.next().unwrap().try_into().ok()?;
                }
            }
        }
        Some(VoxelData::Dense(data.into_boxed_slice()))
    }

    /// Returns a `SerializedVoxelData` corresponding to `self`. Assumes that `self` is `Dense` and
    /// has the right dimension, as it will panic or return incorrect data otherwise.
    pub fn serialize(&self, dimension: u8) -> SerializedVoxelData {
        let VoxelData::Dense(data) = self else {
            panic!("Only dense chunks can be serialized.");
        };

        let mut serialized: Vec<u8> = Vec::with_capacity(usize::from(dimension).pow(3) * 2);
        for z in 0..dimension {
            for y in 0..dimension {
                for x in 0..dimension {
                    // We cannot use a linear copy here because `data` has margins, while `serialized.inner` does not.
                    serialized
                        .extend((data[Coords([x, y, z]).to_index(dimension)] as u16).to_le_bytes());
                }
            }
        }
        SerializedVoxelData { inner: serialized }
    }
}

/// Contains the context needed to know the locations of individual cubes within a chunk in the chunk's coordinate
/// system. A given `ChunkLayout` is uniquely determined by its dimension.
pub struct ChunkLayout {
    dimension: u8,
    dual_to_grid_factor: f32,
}

impl ChunkLayout {
    pub fn new(dimension: u8) -> Self {
        ChunkLayout {
            dimension,
            dual_to_grid_factor: Vertex::dual_to_chunk_factor() * dimension as f32,
        }
    }

    /// Number of cubes on one axis of the chunk.
    #[inline]
    pub fn dimension(&self) -> u8 {
        self.dimension
    }

    /// Scale by this to convert dual coordinates to homogeneous grid coordinates.
    #[inline]
    pub fn dual_to_grid_factor(&self) -> f32 {
        self.dual_to_grid_factor
    }

    /// Converts a single coordinate from dual coordinates in the Klein-Beltrami model to an integer coordinate
    /// suitable for voxel lookup. Returns `None` if the coordinate is outside the chunk.
    #[inline]
    pub fn dual_to_voxel(&self, dual_coord: f32) -> Option<u8> {
        let floor_grid_coord = (dual_coord * self.dual_to_grid_factor).floor();

        if !(floor_grid_coord >= 0.0 && floor_grid_coord < self.dimension as f32) {
            None
        } else {
            Some(floor_grid_coord as u8)
        }
    }

    /// Converts a single coordinate from grid coordinates to dual coordiantes in the Klein-Beltrami model. This
    /// can be used to find the positions of voxel gridlines.
    #[inline]
    pub fn grid_to_dual(&self, grid_coord: u8) -> f32 {
        grid_coord as f32 / self.dual_to_grid_factor
    }

    /// Takes in a single grid coordinate and returns a range containing all voxel coordinates surrounding it.
    #[inline]
    pub fn neighboring_voxels(&self, grid_coord: u8) -> impl Iterator<Item = u8> + use<> {
        grid_coord.saturating_sub(1)..grid_coord.saturating_add(1).min(self.dimension())
    }
}

/// Ensures that every new node of the given Graph is populated with a [Node] and is
/// ready for world generation.
pub fn populate_fresh_nodes(graph: &mut Graph) {
    let fresh = graph.fresh().to_vec();
    graph.clear_fresh();
    for &node in &fresh {
        populate_node(graph, node);
    }
}

fn populate_node(graph: &mut Graph, node: NodeId) {
    *graph.get_mut(node) = Some(Node {
        state: graph
            .parent(node)
            .and_then(|i| {
                let parent_state = &graph.get(graph.neighbor(node, i)?).as_ref()?.state;
                Some(parent_state.child(graph, node, i))
            })
            .unwrap_or_else(NodeState::root),
        chunks: Chunks::default(),
    });
}

/// Represents a discretized region in the voxel grid contained by an axis-aligned bounding box.
pub struct VoxelAABB {
    // The bounds are of the form [[x_min, x_max], [y_min, y_max], [z_min, z_max]], using voxel coordinates with a one-block
    // wide margins added on both sides. This helps make sure that that we can detect if the AABB intersects the chunk's boundaries.
    bounds: [[u8; 2]; 3],
}

impl VoxelAABB {
    /// Returns a bounding box that is guaranteed to cover a given radius around a ray segment. Returns None if the
    /// bounding box lies entirely outside the chunk.
    pub fn from_ray_segment_and_radius(
        layout: &ChunkLayout,
        ray: &Ray,
        tanh_distance: f32,
        radius: f32,
    ) -> Option<VoxelAABB> {
        // Convert the ray to grid coordinates
        let grid_start = na::Point3::from_homogeneous(ray.position.into()).unwrap()
            * layout.dual_to_grid_factor();
        let grid_end = na::Point3::from_homogeneous(ray.ray_point(tanh_distance).into()).unwrap()
            * layout.dual_to_grid_factor();
        // Convert the radius to grid coordinates using a crude conservative estimate
        let max_grid_radius = radius * layout.dual_to_grid_factor();
        let mut bounds = [[0; 2]; 3];
        for axis in 0..3 {
            let grid_min = grid_start[axis].min(grid_end[axis]) - max_grid_radius;
            let grid_max = grid_start[axis].max(grid_end[axis]) + max_grid_radius;
            let voxel_min = (grid_min + 1.0).floor().max(0.0);
            let voxel_max = (grid_max + 1.0)
                .floor()
                .min(layout.dimension() as f32 + 1.0);

            // When voxel_min is greater than dimension or voxel_max is less than 1, the cube does not intersect
            // the chunk.
            if voxel_min > layout.dimension() as f32 || voxel_max < 1.0 {
                return None;
            }

            // We convert to u8 here instead of earlier because out-of-range voxel coordinates can violate casting assumptions.
            bounds[axis] = [voxel_min.floor() as u8, voxel_max.floor() as u8];
        }

        Some(VoxelAABB { bounds })
    }

    /// Iterator over grid points contained in the region, represented as ordered triples
    pub fn grid_points(
        &self,
        axis0: usize,
        axis1: usize,
        axis2: usize,
    ) -> impl Iterator<Item = (u8, u8, u8)> + use<> {
        let bounds = self.bounds;
        (bounds[axis0][0]..bounds[axis0][1]).flat_map(move |i| {
            (bounds[axis1][0]..bounds[axis1][1])
                .flat_map(move |j| (bounds[axis2][0]..bounds[axis2][1]).map(move |k| (i, j, k)))
        })
    }

    /// Iterator over grid lines intersecting the region, represented as ordered pairs determining the line's two fixed coordinates
    pub fn grid_lines(&self, axis0: usize, axis1: usize) -> impl Iterator<Item = (u8, u8)> + use<> {
        let bounds = self.bounds;
        (bounds[axis0][0]..bounds[axis0][1])
            .flat_map(move |i| (bounds[axis1][0]..bounds[axis1][1]).map(move |j| (i, j)))
    }

    /// Iterator over grid planes intersecting the region, represented as integers determining the plane's fixed coordinate
    pub fn grid_planes(&self, axis: usize) -> impl Iterator<Item = u8> + use<> {
        self.bounds[axis][0]..self.bounds[axis][1]
    }
}

#[cfg(test)]
mod tests {
    use std::collections::HashSet;

    use crate::math::{MIsometry, MVector};

    use super::*;

    /// Any voxel AABB should at least cover a capsule-shaped region consisting of all points
    /// `radius` units away from the ray's line segment. This region consists of two spheres
    /// and a cylinder. We only test planes because covered lines and points are a strict subset.
    #[test]
    fn voxel_aabb_coverage() {
        let dimension = 12;
        let layout = ChunkLayout::new(dimension);

        // Pick an arbitrary ray by transforming the positive-x-axis ray.
        let ray = MIsometry::from(na::Rotation3::from_axis_angle(&na::Vector3::z_axis(), 0.4))
            * MIsometry::translation_along(&na::Vector3::new(0.2, 0.3, 0.1))
            * &Ray::new(MVector::w(), MVector::x());

        let tanh_distance = 0.2;
        let radius = 0.1;

        // We want to test that the whole capsule-shaped region around the ray segment is covered by
        // the AABB. However, the math to test for this is complicated, so we instead check a bunch of
        // spheres along this ray segment.
        let num_ray_test_points = 20;
        let ray_test_points: Vec<_> = (0..num_ray_test_points)
            .map(|i| {
                ray.ray_point(tanh_distance * (i as f32 / (num_ray_test_points - 1) as f32))
                    .normalized()
            })
            .collect();

        let aabb =
            VoxelAABB::from_ray_segment_and_radius(&layout, &ray, tanh_distance, radius).unwrap();

        // For variable names and further comments, we use a tuv coordinate system, which
        // is a permuted xyz coordinate system.

        // Test planes in all 3 axes.
        for t_axis in 0..3 {
            let covered_planes: HashSet<_> = aabb.grid_planes(t_axis).collect();

            // Check that all uv-aligned planes that should be covered are covered
            for t in 0..=dimension {
                if covered_planes.contains(&t) {
                    continue;
                }

                let mut plane_normal = MVector::zero();
                plane_normal[t_axis] = 1.0;
                plane_normal[3] = layout.grid_to_dual(t);
                let plane_normal = plane_normal.normalized();

                for test_point in &ray_test_points {
                    assert!(
                        test_point.mip(&plane_normal).abs() > radius.sinh(),
                        "Plane not covered: t_axis={t_axis}, t={t}, test_point={test_point:?}",
                    );
                }
            }
        }

        // Test lines in all 3 axes
        for t_axis in 0..3 {
            let u_axis = (t_axis + 1) % 3;
            let v_axis = (u_axis + 1) % 3;
            let covered_lines: HashSet<_> = aabb.grid_lines(u_axis, v_axis).collect();

            // For a given axis, all lines have the same direction, so set up the appropriate vector
            // in advance.
            let mut line_direction = MVector::zero();
            line_direction[t_axis] = 1.0;
            let line_direction = line_direction;

            // Check that all t-aligned lines that should be covered are covered
            for u in 0..=dimension {
                for v in 0..=dimension {
                    if covered_lines.contains(&(u, v)) {
                        continue;
                    }

                    let mut line_position = MVector::zero();
                    line_position[u_axis] = layout.grid_to_dual(u);
                    line_position[v_axis] = layout.grid_to_dual(v);
                    line_position[3] = 1.0;
                    let line_position = line_position.normalized();

                    for test_point in &ray_test_points {
                        assert!(
                            (test_point.mip(&line_position).powi(2)
                                - test_point.mip(&line_direction).powi(2))
                            .sqrt()
                                > radius.cosh(),
                            "Line not covered: t_axis={t_axis}, u={u}, v={v}, test_point={test_point:?}",
                        );
                    }
                }
            }
        }

        // Test points
        let covered_points: HashSet<_> = aabb.grid_points(0, 1, 2).collect();

        // Check that all points that should be covered are covered
        for x in 0..=dimension {
            for y in 0..=dimension {
                for z in 0..=dimension {
                    if covered_points.contains(&(x, y, z)) {
                        continue;
                    }

                    let point_position = MVector::new(
                        layout.grid_to_dual(x),
                        layout.grid_to_dual(y),
                        layout.grid_to_dual(z),
                        1.0,
                    )
                    .normalized();

                    for test_point in &ray_test_points {
                        assert!(
                            -test_point.mip(&point_position) > radius.cosh(),
                            "Point not covered: x={x}, y={y}, z={z}, test_point={test_point:?}",
                        );
                    }
                }
            }
        }
    }
}
