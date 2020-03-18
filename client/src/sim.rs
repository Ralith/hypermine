use std::time::Duration;

use fxhash::FxHashMap;
use hecs::Entity;
use tracing::{error, trace};

use crate::{graphics::lru_table::SlotId, net, Net};
use common::{
    dodeca,
    graph::{Graph, NodeId},
    proto::{self, Command, Position},
    world::{Material, SUBDIVISION_FACTOR},
    EntityId, Step,
};

/// Game state
pub struct Sim {
    net: Net,

    // World state
    entity_ids: FxHashMap<EntityId, Entity>,
    world: hecs::World,
    pub graph: Graph<bool, Cube>,
    local_character: Option<EntityId>,
    orientation: na::UnitQuaternion<f32>,
    step: Option<Step>,

    // Input state
    velocity: na::Vector3<f32>,
}

impl Sim {
    pub fn new(net: Net) -> Self {
        let mut result = Self {
            net,

            graph: Graph::new(),
            entity_ids: FxHashMap::default(),
            world: hecs::World::new(),
            local_character: None,
            orientation: na::one(),
            step: None,

            velocity: na::zero(),
        };

        result.populate_node(NodeId::ROOT);

        result
    }

    pub fn rotate(&mut self, delta: &na::UnitQuaternion<f32>) {
        self.orientation *= delta;
    }

    pub fn velocity(&mut self, v: na::Vector3<f32>) {
        self.velocity = v;
    }

    pub fn step(&mut self, _dt: Duration) {
        while let Ok(msg) = self.net.incoming.try_recv() {
            self.handle_net(msg);
        }

        self.send_input();
    }

    fn handle_net(&mut self, msg: net::Message) {
        use net::Message::*;
        match msg {
            ConnectionLost(e) => {
                error!("connection lost: {}", e);
            }
            Hello(msg) => {
                self.local_character = Some(msg.character);
            }
            Spawns(msg) => self.handle_spawns(msg),
            StateDelta(msg) => {
                self.step = self.step.max(Some(msg.step));
                for &(id, new_pos) in &msg.positions {
                    match self.entity_ids.get(&id) {
                        None => error!(%id, "position update for unknown entity"),
                        Some(&entity) => match self.world.get_mut::<Position>(entity) {
                            Ok(mut pos) => {
                                *pos = new_pos;
                            }
                            Err(e) => error!(%id, "position update for unpositioned entity: {}", e),
                        },
                    }
                }
            }
        }
    }

    fn handle_spawns(&mut self, msg: proto::Spawns) {
        self.step = self.step.max(Some(msg.step));
        let mut builder = hecs::EntityBuilder::new();
        for &(id, ref components) in &msg.spawns {
            trace!(%id, "spawning entity");
            builder.add(id);
            for component in components {
                use common::proto::Component::*;
                match *component {
                    Character(_) => {}
                    Position(x) => {
                        builder.add(x);
                    }
                }
            }
            let entity = self.world.spawn(builder.build());
            if let Some(x) = self.entity_ids.insert(id, entity) {
                let _ = self.world.despawn(x);
                error!(%id, "id collision");
            }
        }
        for &id in &msg.despawns {
            match self.entity_ids.get(&id) {
                Some(&entity) => self.destroy(entity),
                None => error!(%id, "despawned unknown entity"),
            }
        }
        if !msg.nodes.is_empty() {
            trace!(count = msg.nodes.len(), "adding nodes");
        }
        for node in &msg.nodes {
            self.graph.insert_child(node.parent, node.side);
        }
        self.populate_fresh_nodes();
    }

    fn send_input(&mut self) {
        if let Some(&entity) = self.local_character.and_then(|id| self.entity_ids.get(&id)) {
            let pos = *self.world.get::<Position>(entity).unwrap();
            // Any failure here will be better handled in ConnectionLost above on the next call
            let _ = self.net.outgoing.send(Command {
                step: self.step.unwrap(),
                node: pos.node,
                orientation: self.orientation,
                velocity: self.orientation * self.velocity,
            });
        }
    }

    pub fn view(&self) -> Position {
        if let Some(&entity) = self.local_character.and_then(|id| self.entity_ids.get(&id)) {
            let mut pos = *self.world.get::<Position>(entity).unwrap();
            pos.local *= self.orientation.to_homogeneous();
            pos
        } else {
            Position {
                node: NodeId::ROOT,
                local: na::Matrix4::identity(),
            }
        }
    }

    fn destroy(&mut self, entity: Entity) {
        let id = *self.world.get::<EntityId>(entity).unwrap();
        self.entity_ids.remove(&id);
        self.world
            .despawn(entity)
            .expect("destroyed nonexistent entity");
    }

    fn populate_fresh_nodes(&mut self) {
        let fresh = self.graph.fresh().to_vec();
        self.graph.clear_fresh();
        for &node in &fresh {
            self.populate_node(node);
        }
        for &node in &fresh {
            for cube in self.graph.cubes_at(node) {
                self.populate_cube(node, cube);
            }
        }
        self.graph.clear_fresh();
    }

    fn populate_node(&mut self, node: NodeId) {
        use common::dodeca::Side;

        *self.graph.get_mut(node) = Some(match self.graph.parent(node) {
            None => true,
            Some(x) => {
                let parent_solid = self
                    .graph
                    .get(self.graph.neighbor(node, x).unwrap())
                    .unwrap();
                if x == Side::A {
                    !parent_solid
                } else {
                    parent_solid
                }
            }
        });
    }

    fn populate_cube(&mut self, node: NodeId, cube: dodeca::Vertex) {
        let contains_border = cube.canonical_sides().iter().any(|&x| x == dodeca::Side::A);

        let voxels = if contains_border {
            let mut data = (0..(SUBDIVISION_FACTOR + 2).pow(3))
                .map(|_| Material::Void)
                .collect::<Vec<_>>()
                .into_boxed_slice();

            const MAGIC: u32 = 1_000_081;
            let mut rd = ((cube as u32) + 20 * u32::from(node)) % MAGIC; // Pseudorandom value to fill chunk with
            const GAP: usize = 0;
            const XGAP: usize = (SUBDIVISION_FACTOR - 1) / 2; // dodeca::Side::A will always correspond to the x coordinate, so let`s flatten it in this direction
            for z in GAP..(SUBDIVISION_FACTOR - GAP) {
                for y in GAP..(SUBDIVISION_FACTOR - GAP) {
                    for x in XGAP..(SUBDIVISION_FACTOR - XGAP) {
                        rd = (37 * rd + 1) % MAGIC;
                        data[(x + 1)
                            + (y + 1) * (SUBDIVISION_FACTOR + 2)
                            + (z + 1) * (SUBDIVISION_FACTOR + 2).pow(2)] = if rd % 4 == 1 {
                            Material::Stone
                        } else if rd % 4 == 2 {
                            Material::Dirt
                        } else if rd % 4 == 3 {
                            Material::Sand
                        } else {
                            Material::Void
                        };
                    }
                }
            }
            VoxelData::Dense(data)
        } else {
            VoxelData::Empty
        };
        *self.graph.get_cube_mut(node, cube) = Some(Cube {
            surface: None,
            voxels,
        });
    }
}

pub struct Cube {
    pub surface: Option<SlotId>,
    pub voxels: VoxelData,
}

pub enum VoxelData {
    Empty,
    Dense(Box<[Material]>),
}
