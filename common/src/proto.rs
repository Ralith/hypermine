use serde::{Deserialize, Serialize};

use crate::{
     dodeca, graph::NodeId, math::MIsometry, node::ChunkId, voxel_math::Coords, world::Material,
     EntityId, SimConfig, Step,
};

#[derive(Debug, Serialize, Deserialize)]
pub struct ClientHello {
    pub name: String,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct ServerHello {
    pub character: EntityId,
    pub sim_config: SimConfig,
}

#[derive(Debug, Serialize, Deserialize, Copy, Clone)]
pub struct Position {
    pub node: NodeId,
    pub local: MIsometry<f32>,
}

impl Position {
    pub fn origin() -> Self {
        Self {
            node: NodeId::ROOT,
            local: MIsometry::identity(),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StateDelta {
    pub step: Step,
    /// Highest input generation received prior to `step`
    pub latest_input: u16,
    pub positions: Vec<(EntityId, Position)>,
    pub character_states: Vec<(EntityId, CharacterState)>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CharacterState {
    pub velocity: na::Vector3<f32>,
    pub on_ground: bool,
    pub orientation: na::UnitQuaternion<f32>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct Spawns {
    pub step: Step,
    pub spawns: Vec<(EntityId, Vec<Component>)>,
    pub despawns: Vec<EntityId>,
    pub nodes: Vec<FreshNode>,
    pub block_updates: Vec<BlockUpdate>,
    pub voxel_data: Vec<(ChunkId, SerializedVoxelData)>,
    pub inventory_additions: Vec<(EntityId, EntityId)>,
    pub inventory_removals: Vec<(EntityId, EntityId)>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct Command {
    pub generation: u16,
    pub character_input: CharacterInput,
    pub orientation: na::UnitQuaternion<f32>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CharacterInput {
    /// Relative to the character's current position, excluding orientation
    pub movement: na::Vector3<f32>,
    pub jump: bool,
    pub no_clip: bool,
    pub block_update: Option<BlockUpdate>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BlockUpdate {
    pub chunk_id: ChunkId,
    pub coords: Coords,
    pub new_material: Material,
    pub consumed_entity: Option<EntityId>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct SerializedVoxelData {
    /// Dense 3D array of 16-bit material tags for all voxels in this chunk
    pub inner: Vec<u8>,
}

#[derive(Debug, Serialize, Deserialize)]
pub enum Component {
    Character(Character),
    Position(Position),
    Material(Material),
    Inventory(Inventory),
}

#[derive(Debug, Serialize, Deserialize)]
pub struct FreshNode {
    /// The side joining the new node to `parent`
    pub side: dodeca::Side,
    pub parent: NodeId,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct Character {
    pub name: String,
    pub state: CharacterState,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct Inventory {
    pub contents: Vec<EntityId>,
}
