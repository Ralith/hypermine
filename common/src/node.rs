/*the name of this module is pretty arbitrary at the moment*/

use std::ops::{Index, IndexMut};

use serde::{Deserialize, Serialize};

use crate::dodeca::Vertex;
use crate::graph::{Graph, NodeId};
use crate::lru_slab::SlotId;
use crate::proto::Position;
use crate::world::Material;
use crate::worldgen::NodeState;
use crate::{math, Chunks};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
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
            (math::mtranspose(&position.local) * node.state.up_direction()).xyz(),
        ))
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

/// Coordinates for a discrete voxel within a chunk, not including margins
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct Coords(pub [u8; 3]);

impl Coords {
    /// Returns the array index in `VoxelData` corresponding to these coordinates
    pub fn to_index(&self, chunk_size: u8) -> usize {
        let chunk_size_with_margin = chunk_size as usize + 2;
        (self.0[0] as usize + 1)
            + (self.0[1] as usize + 1) * chunk_size_with_margin
            + (self.0[2] as usize + 1) * chunk_size_with_margin.pow(2)
    }
}

impl Index<usize> for Coords {
    type Output = u8;

    fn index(&self, coord_axis: usize) -> &u8 {
        self.0.index(coord_axis)
    }
}

impl IndexMut<usize> for Coords {
    fn index_mut(&mut self, coord_axis: usize) -> &mut u8 {
        self.0.index_mut(coord_axis)
    }
}

pub struct Node {
    pub state: NodeState,
    /// We can only populate chunks which lie within a cube of populated nodes, so nodes on the edge
    /// of the graph always have some `None` chunks.
    pub chunks: Chunks<Chunk>,
}

#[derive(Default)]
pub enum Chunk {
    #[default]
    Fresh,
    Generating,
    Populated {
        voxels: VoxelData,
        surface: Option<SlotId>,
    },
}

pub enum VoxelData {
    Solid(Material),
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
            dual_to_grid_factor: Vertex::dual_to_chunk_factor() as f32 * dimension as f32,
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
    pub fn neighboring_voxels(&self, grid_coord: u8) -> impl Iterator<Item = u8> {
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
