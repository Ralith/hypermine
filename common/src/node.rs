/*the name of this module is pretty arbitrary at the moment*/

use std::ops::{Index, IndexMut};

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
    dimension: usize,
    dual_to_grid_factor: f32,
}

impl ChunkLayout {
    pub fn new(dimension: usize) -> Self {
        ChunkLayout {
            dimension,
            dual_to_grid_factor: Vertex::dual_to_chunk_factor() as f32 * dimension as f32,
        }
    }

    /// Number of cubes on one axis of the chunk. Margins are not included.
    #[inline]
    pub fn dimension(&self) -> usize {
        self.dimension
    }

    /// Scale by this to convert dual coordinates to homogeneous grid coordinates.
    #[inline]
    pub fn dual_to_grid_factor(&self) -> f32 {
        self.dual_to_grid_factor
    }

    /// Converts a single coordinate from dual coordinates in the Klein-Beltrami model to an integer coordinate
    /// suitable for voxel lookup. Margins are included. Returns `None` if the coordinate is outside the chunk.
    #[inline]
    pub fn dual_to_voxel(&self, dual_coord: f32) -> Option<usize> {
        let floor_grid_coord = (dual_coord * self.dual_to_grid_factor).floor();

        if !(floor_grid_coord >= 0.0 && floor_grid_coord < self.dimension as f32) {
            None
        } else {
            Some(floor_grid_coord as usize + 1)
        }
    }

    /// Converts a single coordinate from grid coordinates to dual coordiantes in the Klein-Beltrami model. This
    /// can be used to find the positions of voxel gridlines.
    #[inline]
    pub fn grid_to_dual(&self, grid_coord: usize) -> f32 {
        grid_coord as f32 / self.dual_to_grid_factor
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
