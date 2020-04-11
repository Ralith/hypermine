use serde::{Deserialize, Serialize};

use crate::{dodeca, graph::NodeId, EntityId, Step};

#[derive(Debug, Serialize, Deserialize)]
pub struct ClientHello {
    pub name: String,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct ServerHello {
    pub character: EntityId,
    pub rate: u16,
    /// Number of voxels along the edge of a chunk
    pub chunk_size: u8,
    /// Maximum movement speed in absolute units
    pub movement_speed: f32,
    /// Unit conversion factor
    pub meters_to_absolute: f32,
}

#[derive(Debug, Serialize, Deserialize, Copy, Clone)]
pub struct Position {
    pub node: NodeId,
    pub local: na::Matrix4<f32>,
}

impl Position {
    pub fn origin() -> Self {
        Self {
            node: NodeId::ROOT,
            local: na::Matrix4::identity(),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StateDelta {
    pub step: Step,
    /// Highest input generation received prior to `step`
    pub latest_input: u16,
    pub positions: Vec<(EntityId, Position)>,
    pub character_orientations: Vec<(EntityId, na::UnitQuaternion<f32>)>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct Spawns {
    pub step: Step,
    pub spawns: Vec<(EntityId, Vec<Component>)>,
    pub despawns: Vec<EntityId>,
    pub nodes: Vec<FreshNode>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct Command {
    pub generation: u16,
    pub orientation: na::UnitQuaternion<f32>,
    /// Relative to the character's current position, excluding orientation
    pub velocity: na::Vector3<f32>,
}

#[derive(Debug, Serialize, Deserialize)]
pub enum Component {
    Character(Character),
    Position(Position),
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
    pub orientation: na::UnitQuaternion<f32>,
}
