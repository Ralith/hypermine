/*the name of this module is pretty arbitrary at the moment*/

use crate::graph::Graph;
use crate::lru_slab::SlotId;
use crate::world::Material;
use crate::worldgen::NodeState;
use crate::Chunks;

pub type DualGraph = Graph<Node>;

pub struct Node {
    pub state: NodeState,
    /// We can only populate chunks which lie within a cube of populated nodes, so nodes on the edge
    /// of the graph always have some `None` chunks.
    pub chunks: Chunks<Chunk>,
}

pub enum Chunk {
    Fresh,
    Generating,
    Populated {
        voxels: VoxelData,
        surface: Option<SlotId>,
    },
}

impl Default for Chunk {
    fn default() -> Self {
        Chunk::Fresh
    }
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
