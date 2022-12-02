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
        old_surface: Option<SlotId>,
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
                let lwm = usize::from(dimension) + 2;
                let mut data = vec![Material::Void; lwm.pow(3)];
                for i in 1..(lwm - 1) {
                    for j in 1..(lwm - 1) {
                        for k in 1..(lwm - 1) {
                            data[i + j * lwm + k * lwm.pow(2)] = mat;
                        }
                    }
                }
                *self = VoxelData::Dense(data.into());
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
