use std::time::Duration;

use fxhash::FxHashMap;
use hecs::Entity;
use tracing::{debug, error, trace};

use crate::{
    graphics::lru_table::SlotId, net, prediction::PredictedMotion, worldgen, worldgen::NodeState,
    Net,
};
use common::{
    dodeca,
    graph::{Graph, NodeId},
    math,
    proto::{self, Character, Command, Component, Position},
    sanitize_motion_input,
    world::{Material, SUBDIVISION_FACTOR},
    Chunks, EntityId, GraphEntities, Step,
};

pub type DualGraph = Graph<Node>;

pub struct Node {
    pub state: NodeState,
    pub chunks: Chunks<Option<Chunk>>,
}

/// Game state
pub struct Sim {
    net: Net,

    // World state
    pub graph: DualGraph,
    pub graph_entities: GraphEntities,
    entity_ids: FxHashMap<EntityId, Entity>,
    pub world: hecs::World,
    local_character_id: Option<EntityId>,
    pub local_character: Option<Entity>,
    orientation: na::UnitQuaternion<f32>,
    step_interval: Option<Duration>,
    step: Option<Step>,

    // Input state
    since_input_sent: Duration,
    /// Most recent input
    instantaneous_velocity: na::Vector3<f32>,
    /// Average input over the current time step. The portion of the timestep which has not yet
    /// elapsed is considered to have zero input.
    average_velocity: na::Vector3<f32>,
    prediction: PredictedMotion,
}

impl Sim {
    pub fn new(net: Net) -> Self {
        Self {
            net,

            graph: Graph::new(),
            graph_entities: GraphEntities::new(),
            entity_ids: FxHashMap::default(),
            world: hecs::World::new(),
            local_character_id: None,
            local_character: None,
            orientation: na::one(),
            step_interval: None,
            step: None,

            since_input_sent: Duration::new(0, 0),
            instantaneous_velocity: na::zero(),
            average_velocity: na::zero(),
            prediction: PredictedMotion::new(proto::Position {
                node: NodeId::ROOT,
                local: na::one(),
            }),
        }
    }

    pub fn connected(&self) -> bool {
        self.step_interval.is_some()
    }

    pub fn rotate(&mut self, delta: &na::UnitQuaternion<f32>) {
        self.orientation *= delta;
    }

    pub fn velocity(&mut self, v: na::Vector3<f32>) {
        self.instantaneous_velocity = v;
    }

    pub fn step(&mut self, dt: Duration) {
        self.orientation.renormalize_fast();

        while let Ok(msg) = self.net.incoming.try_recv() {
            self.handle_net(msg);
        }

        self.since_input_sent += dt;
        if let Some(step_interval) = self.step_interval {
            if let Some(overflow) = self.since_input_sent.checked_sub(step_interval) {
                self.average_velocity += self.instantaneous_velocity
                    * (dt - overflow).as_secs_f32()
                    / step_interval.as_secs_f32();
                self.send_input();
                if overflow > step_interval {
                    // If it's been more than two timesteps since we last sent input, skip ahead
                    // rather than spamming the server.
                    self.average_velocity = na::zero();
                    self.since_input_sent = Duration::new(0, 0);
                } else {
                    self.average_velocity = self.instantaneous_velocity * overflow.as_secs_f32()
                        / step_interval.as_secs_f32();
                    // Send the next input a little sooner if necessary to stay in sync
                    self.since_input_sent = overflow;
                }
            } else {
                self.average_velocity +=
                    self.instantaneous_velocity * dt.as_secs_f32() / step_interval.as_secs_f32();
            }
        }
    }

    fn handle_net(&mut self, msg: net::Message) {
        use net::Message::*;
        match msg {
            ConnectionLost(e) => {
                error!("connection lost: {}", e);
            }
            Hello(msg) => {
                self.local_character_id = Some(msg.character);
                self.step_interval = Some(Duration::from_secs(1) / msg.rate as u32);
            }
            Spawns(msg) => self.handle_spawns(msg),
            StateDelta(msg) => {
                // Discard out-of-order messages, taking care to account for step counter wrapping.
                if self.step.map_or(false, |x| x.wrapping_sub(msg.step) >= 0) {
                    return;
                }
                self.step = Some(msg.step);
                for &(id, new_pos) in &msg.positions {
                    self.update_position(msg.latest_input, id, new_pos);
                }
                for &(id, orientation) in &msg.character_orientations {
                    match self.entity_ids.get(&id) {
                        None => debug!(%id, "character orientation update for unknown entity"),
                        Some(&entity) => match self.world.get_mut::<Character>(entity) {
                            Ok(mut ch) => {
                                ch.orientation = orientation;
                            }
                            Err(e) => {
                                error!(%id, "character orientation update for non-character entity {}", e)
                            }
                        },
                    }
                }
            }
        }
    }

    fn update_position(&mut self, latest_input: u16, id: EntityId, new_pos: Position) {
        if self.local_character_id == Some(id) {
            self.prediction.reconcile(latest_input, new_pos);
        }
        match self.entity_ids.get(&id) {
            None => debug!(%id, "position update for unknown entity"),
            Some(&entity) => match self.world.get_mut::<Position>(entity) {
                Ok(mut pos) => {
                    if pos.node != new_pos.node {
                        self.graph_entities.remove(pos.node, entity);
                        self.graph_entities.insert(new_pos.node, entity);
                    }
                    *pos = new_pos;
                }
                Err(e) => error!(%id, "position update for unpositioned entity {}", e),
            },
        }
    }

    fn handle_spawns(&mut self, msg: proto::Spawns) {
        self.step = self.step.max(Some(msg.step));
        let mut builder = hecs::EntityBuilder::new();
        for (id, components) in msg.spawns {
            self.spawn(&mut builder, id, components);
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

    fn spawn(
        &mut self,
        builder: &mut hecs::EntityBuilder,
        id: EntityId,
        components: Vec<Component>,
    ) {
        trace!(%id, "spawning entity");
        builder.add(id);
        let mut node = None;
        for component in components {
            use common::proto::Component::*;
            match component {
                Character(x) => {
                    builder.add(x);
                }
                Position(x) => {
                    node = Some(x.node);
                    builder.add(x);
                }
            };
        }
        let entity = self.world.spawn(builder.build());
        if let Some(node) = node {
            self.graph_entities.insert(node, entity);
        }
        if id == self.local_character_id.expect("spawn before ServerHello") {
            self.local_character = Some(entity);
        }
        if let Some(x) = self.entity_ids.insert(id, entity) {
            self.destroy_idless(x);
            error!(%id, "id collision");
        }
    }

    fn send_input(&mut self) {
        let (direction, speed) = sanitize_motion_input(self.orientation * self.average_velocity);
        let generation = self.prediction.push(
            &direction,
            speed * self.step_interval.as_ref().unwrap().as_secs_f32(),
        );

        // Any failure here will be better handled in handle_net's ConnectionLost case
        let _ = self.net.outgoing.send(Command {
            generation,
            orientation: self.orientation,
            velocity: direction.into_inner() * speed,
        });
    }

    pub fn view(&self) -> Position {
        let mut result = *self.prediction.predicted();
        result.local *= self.orientation.to_homogeneous();
        if let Some(step_interval) = self.step_interval {
            // Apply input that hasn't been sent yet
            let (direction, speed) = sanitize_motion_input(self.average_velocity);
            // We multiply by the entire timestep rather than the time so far because
            // self.average_velocity is always over the entire timestep, filling in zeroes for the
            // future.
            let distance = speed * step_interval.as_secs_f32();
            result.local *= math::translate_along(&direction, distance);
        }
        result
    }

    /// Destroy all aspects of an entity
    fn destroy(&mut self, entity: Entity) {
        let id = *self
            .world
            .get::<EntityId>(entity)
            .expect("destroyed nonexistent entity");
        self.entity_ids.remove(&id);
        self.destroy_idless(entity);
    }

    /// Destroy an entity without an EntityId mapped
    fn destroy_idless(&mut self, entity: Entity) {
        if let Ok(position) = self.world.get::<Position>(entity) {
            self.graph_entities.remove(position.node, entity);
        }
        self.world
            .despawn(entity)
            .expect("destroyed nonexistent entity");
    }
}

pub struct Chunk {
    pub surface: Option<SlotId>,
    pub voxels: VoxelData,
}

#[derive(PartialEq)]
pub enum VoxelData {
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
        // If this node has three descenders, it completes a cube, and we can populate all eight
        // chunks within that cube
        if let (Some(a), Some(b), Some(c)) = (d.next(), d.next(), d.next()) {
            let vert = dodeca::Vertex::from_sides(a, b, c).expect("descenders are adjacent");
            for (_, path) in vert.dual_vertices() {
                let node = path.fold(node, |node, side| {
                    graph
                        .neighbor(node, side)
                        .expect("chunks whose outward-facing neighbors are all along descenders lie between populated cubes")
                });
                populate_chunk(graph, node, vert);
            }
        }
    }
    graph.clear_fresh();
}

fn populate_node(graph: &mut DualGraph, node: NodeId) {
    *graph.get_mut(node) = Some(Node {
        state: graph
            .parent(node)
            .and_then(|i| {
                let parent_state = &graph.get(graph.neighbor(node, i)?).as_ref()?.state;
                Some(parent_state.child(graph, node, i))
            })
            .unwrap_or_else(NodeState::root),
        chunks: Chunks::default(),
    });
}

fn populate_chunk(graph: &mut DualGraph, node: NodeId, chunk: dodeca::Vertex) {
    graph
        .get_mut(node)
        .as_mut()
        .expect("must not be called on an unpopulated node")
        .chunks[chunk] = Some(Chunk {
        surface: None,
        voxels: worldgen::voxels(graph, node, chunk),
    });
}
