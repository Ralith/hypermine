use std::{sync::Arc, time::Duration};

use fxhash::FxHashMap;
use hecs::Entity;
use tracing::{error, trace};

use crate::{worldgen::NodeState, graphics::lru_table::SlotId, net, Config, Net};
use common::{
    dodeca,
    cursor::Cursor,
    graph::{Graph, NodeId},
    proto::{self, Command, Position},
    world::{Material, SUBDIVISION_FACTOR},
    EntityId, Step,
};

/// Game state
pub struct Sim {
    cfg: Arc<Config>,
    net: Net,

    // World state
    entity_ids: FxHashMap<EntityId, Entity>,
    world: hecs::World,
    pub graph: Graph<NodeState, Cube>,
    local_character: Option<EntityId>,
    orientation: na::UnitQuaternion<f32>,
    step: Option<Step>,

    // Input state
    since_input_sent: Duration,
    velocity: na::Vector3<f32>,
}

impl Sim {
    pub fn new(net: Net, cfg: Arc<Config>) -> Self {
        let mut result = Self {
            cfg,
            net,

            graph: Graph::new(),
            entity_ids: FxHashMap::default(),
            world: hecs::World::new(),
            local_character: None,
            orientation: na::one(),
            step: None,

            since_input_sent: Duration::new(0, 0),
            velocity: na::zero(),
        };

        result.populate_node(NodeId::ROOT);
        for v in dodeca::Vertex::iter() {
            result.populate_cube(NodeId::ROOT, v);
        }

        result
    }

    pub fn rotate(&mut self, delta: &na::UnitQuaternion<f32>) {
        self.orientation *= delta;
    }

    pub fn velocity(&mut self, v: na::Vector3<f32>) {
        self.velocity = v;
    }

    pub fn step(&mut self, dt: Duration) {
        while let Ok(msg) = self.net.incoming.try_recv() {
            self.handle_net(msg);
        }

        self.since_input_sent += dt;
        if self.since_input_sent > Duration::from_secs(1) / (self.cfg.input_send_rate as u32) {
            self.send_input();
            self.since_input_sent = Duration::new(0, 0);
        }
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
        *self.graph.get_mut(node) = self
            .graph
            .parent(node)
            .and_then(|i| {
                let parent_state = (*self.graph.get(self.graph.neighbor(node, i)?))?;
                Some(parent_state.child(i))
            })
            .or(Some(NodeState::ROOT));
    }

    fn populate_cube(&mut self, node: NodeId, cube: dodeca::Vertex) {
        let node_state = self.graph.get(node).unwrap();
        // find the state of all nodes incident to this cube
        let voxels = NodeState::voxels(
            cube
                .dual_vertices()
                // this'll give us all the nodes that touch the cube
                .map(|paths| paths.fold((Some(node), node_state), |(acc_id, acc_state), x| {
                    let x_id = acc_id.and_then(|i| self.graph.neighbor(i, x));
                    (
                        x_id,
                        x_id.and_then(|i| *self.graph.get(i))
                            .unwrap_or_else(|| acc_state.child(x))
                    )
                }))
                .map(|(_, state)| state)
                // their states can tell us their precedence and what to render
                .collect(),
            Cursor::from_vertex(node, cube)
        );
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
