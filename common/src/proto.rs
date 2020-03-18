use serde::{Deserialize, Serialize};

use crate::{dodeca, graph::NodeId, EntityId, Step};

#[derive(Debug, Serialize, Deserialize)]
pub struct ClientHello {}

#[derive(Debug, Serialize, Deserialize)]
pub struct ServerHello {
    pub character: EntityId,
}

#[derive(Debug, Serialize, Deserialize, Copy, Clone)]
pub struct Position {
    pub node: NodeId,
    pub local: na::Matrix4<f32>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct StateDelta {
    pub step: Step,
    pub positions: Vec<(EntityId, Position)>,
    pub characters: Vec<(EntityId, Character)>,
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
    pub step: Step,
    /// The node that `orientation` and `velocity` are relative to
    pub node: NodeId,
    pub orientation: na::UnitQuaternion<f32>,
    /// Relative to the character's current position
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

#[derive(Debug, Serialize, Deserialize)]
pub struct Character {
    pub orientation: na::UnitQuaternion<f32>,
}
