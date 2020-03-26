use std::{sync::Arc, time::Duration};

use fxhash::FxHashMap;
use hecs::Entity;
use tracing::{error, trace};

use crate::{
    graphics::lru_table::SlotId,
    net,
    worldgen::{self, NodeState},
    Config, Net,
};
use common::{
    dodeca,
    graph::{Graph, NodeId},
    proto::{self, Command, Position},
    world::{Material, SUBDIVISION_FACTOR},
    EntityId, Step,
};

pub type DualGraph = Graph<NodeState, Cube>;

/// Game state
pub struct Sim {
    cfg: Arc<Config>,
    net: Net,

    // World state
    pub graph: DualGraph,
    entity_ids: FxHashMap<EntityId, Entity>,
    world: hecs::World,
    local_character: Option<EntityId>,
    orientation: na::UnitQuaternion<f32>,
    step: Option<Step>,

    // Input state
    since_input_sent: Duration,
    velocity: na::Vector3<f32>,
}

impl Sim {
    pub fn new(net: Net, cfg: Arc<Config>) -> Self {
        Self {
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
        }
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
        populate_fresh_nodes(&mut self.graph);
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
}

pub struct Cube {
    pub surface: Option<SlotId>,
    pub voxels: VoxelData,
}

#[derive(PartialEq)]
pub enum VoxelData {
    Uninitialized,
    Solid(Material),
    Dense(Box<[Material]>),
}
impl VoxelData {
    pub fn data_mut(&mut self) -> &mut [Material] {
        match self {
            VoxelData::Dense(d) => d,
            _ => {
                *self = VoxelData::Dense(self.data());
                self.data_mut()
            }
        }
    }
    pub fn data(&self) -> Box<[Material]> {
        match self {
            VoxelData::Dense(d) => Box::clone(d),
            VoxelData::Solid(mat) => (0..(SUBDIVISION_FACTOR + 2).pow(3))
                .map(|_| *mat)
                .collect::<Vec<_>>()
                .into_boxed_slice(),
            VoxelData::Uninitialized => VoxelData::Solid(Material::Void).data(),
        }
    }
}

fn populate_fresh_nodes(graph: &mut DualGraph) {
    let fresh = graph.fresh().to_vec();
    graph.clear_fresh();
    for &node in &fresh {
        populate_node(graph, node);
    }
    for &node in &fresh {
        let mut d = graph.descenders(node).map(|(side, _node)| side);
        // If all of a node's neighbors are loaded, it will have three descenders
        if let (Some(a), Some(b), Some(c)) = (d.next(), d.next(), d.next()) {
            let vert = dodeca::Vertex::from_sides(a, b, c).unwrap();
            let (node, _) = graph.canonicalize(node, vert).unwrap();
            populate_cube(graph, node, vert);
        }
    }
    graph.clear_fresh();
}

fn populate_node(graph: &mut DualGraph, node: NodeId) {
    *graph.get_mut(node) = graph
        .parent(node)
        .and_then(|i| {
            let parent_state = graph.get(graph.neighbor(node, i)?).as_ref()?;
            Some(parent_state.child(graph, node, i))
        })
        .or_else(|| Some(NodeState::root()));
}

fn populate_cube(graph: &mut DualGraph, node: NodeId, cube: dodeca::Vertex) {
    *graph.get_cube_mut(node, cube) = Some(Cube {
        surface: None,
        voxels: worldgen::voxels(graph, node, cube),
    });
}
