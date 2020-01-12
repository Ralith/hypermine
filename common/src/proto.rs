use serde::{Deserialize, Serialize};

use crate::{world::TileId, EntityId, Step};

#[derive(Debug, Serialize, Deserialize)]
pub struct ClientHello {}

#[derive(Debug, Serialize, Deserialize)]
pub struct ServerHello {
    pub character: EntityId,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct Tile {
    pub parent: TileId,
    pub voxels: Box<[u16]>,
}

#[derive(Debug, Serialize, Deserialize, Copy, Clone)]
pub struct Position {
    pub tile: TileId,
    pub local: na::Point3<f32>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct StateDelta {
    pub step: Step,
    pub positions: Vec<(EntityId, Position)>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct Spawns {
    pub step: Step,
    pub spawns: Vec<(EntityId, Vec<Component>)>,
    pub despawns: Vec<EntityId>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct Command {
    pub step: Step,
    pub velocity: na::Vector3<f32>,
    pub orientation: na::UnitQuaternion<f32>,
}

#[derive(Debug, Serialize, Deserialize)]
pub enum Component {
    Position(Position),
}
