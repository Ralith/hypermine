use crate::lru_slab::SlotId;
use crate::node::VoxelData;

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
