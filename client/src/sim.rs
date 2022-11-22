use std::time::Duration;

use fxhash::FxHashMap;
use hecs::Entity;
use tracing::{debug, error, trace};

use crate::{net, prediction::PredictedMotion, Net};
use common::{
    graph::{Graph, NodeId},
    math,
    node::{DualGraph, Node},
    proto::{self, Character, Command, Component, Position},
    sanitize_motion_input,
    worldgen::NodeState,
    Chunks, EntityId, GraphEntities, Step,
};

/// Game state
pub struct Sim {
    net: Net,

    // World state
    pub graph: DualGraph,
    pub graph_entities: GraphEntities,
    entity_ids: FxHashMap<EntityId, Entity>,
    pub world: hecs::World,
    pub params: Option<Parameters>,
    pub local_character: Option<Entity>,
    position: Position,
    yaw: f32,
    pitch: f32,
    step: Option<Step>,

    // Input state
    since_input_sent: Duration,
    /// Most recent input
    ///
    /// Units are relative to movement speed.
    instantaneous_velocity: na::Vector3<f32>,
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
            params: None,
            local_character: None,
            position: Position::origin(),
            yaw: 0.0,
            pitch: 0.0,
            step: None,

            since_input_sent: Duration::new(0, 0),
            instantaneous_velocity: na::zero(),
            prediction: PredictedMotion::new(),
        }
    }

    pub fn rotate(&mut self, yaw: f32, pitch: f32) {
        self.yaw += yaw;
        while self.yaw > std::f32::consts::TAU {
            self.yaw -= std::f32::consts::TAU;
        }
        while self.yaw < 0.0 {
            self.yaw += std::f32::consts::TAU;
        }

        self.pitch += pitch;
        if self.pitch > std::f32::consts::FRAC_PI_2 {
            self.pitch = std::f32::consts::FRAC_PI_2;
        }
        if self.pitch < -std::f32::consts::FRAC_PI_2 {
            self.pitch = -std::f32::consts::FRAC_PI_2;
        }
    }

    fn get_orientation(&self) -> na::Rotation3<f32> {
        na::Rotation::from_axis_angle(&na::Vector3::y_axis(), -self.yaw)
            * na::Rotation::from_axis_angle(&na::Vector3::x_axis(), -self.pitch)
    }

    pub fn velocity(&mut self, v: na::Vector3<f32>) {
        self.instantaneous_velocity = v;
    }

    pub fn params(&self) -> Option<&Parameters> {
        self.params.as_ref()
    }

    pub fn step(&mut self, dt: Duration) {
        while let Ok(msg) = self.net.incoming.try_recv() {
            self.handle_net(msg);
        }

        if let Some(step_interval) = self.params.as_ref().map(|x| x.step_interval) {
            self.since_input_sent += dt;
            if let Some(overflow) = self.since_input_sent.checked_sub(step_interval) {
                // Send fresh input
                self.send_input();

                // Reset state for the next step
                if overflow > step_interval {
                    // If it's been more than two timesteps since we last sent input, skip ahead
                    // rather than spamming the server.
                    self.since_input_sent = Duration::new(0, 0);
                } else {
                    // Send the next input a little sooner if necessary to stay in sync
                    self.since_input_sent = overflow;
                }
            }

            // Update average velocity for the time within the current step
            let movement_speed = self.params.as_ref().unwrap().movement_speed;
            let (direction, speed) = sanitize_motion_input(
                self.get_orientation() * self.instantaneous_velocity * movement_speed,
            );
            self.position.local *= math::translate_along(&direction, speed * dt.as_secs_f32());
            self.position.local = math::renormalize_isometry(&self.position.local);

            let (next_node, transition_xf) = self
                .graph
                .normalize_transform(self.position.node, &self.position.local);
            if next_node != self.position.node {
                self.position.node = next_node;
                self.position.local = transition_xf * self.position.local;
            }

            self.align_with_gravity();
        }
    }

    fn align_with_gravity(&mut self) {
        // TODO: This is not quite precise, as it uses the sin(x) = x approximation.
        // There is likely a mathematically nicer way to handle this.
        let relative_down = self.get_relative_down();
        let axisangle = relative_down
            .xyz()
            .normalize()
            .cross(&(-na::Vector3::y_axis()));
        self.position.local *= na::Rotation3::from_scaled_axis(axisangle).to_homogeneous();
    }

    fn get_relative_down(&self) -> na::Vector4<f32> {
        let node = self.graph.get(self.position.node).as_ref().unwrap();
        self.position.local.try_inverse().unwrap() * node.state.surface().normal().cast()
    }

    fn handle_net(&mut self, msg: net::Message) {
        use net::Message::*;
        match msg {
            ConnectionLost(e) => {
                error!("connection lost: {}", e);
            }
            Hello(msg) => {
                self.params = Some(Parameters {
                    character_id: msg.character,
                    step_interval: Duration::from_secs(1) / u32::from(msg.rate),
                    chunk_size: msg.chunk_size,
                    meters_to_absolute: msg.meters_to_absolute,
                    movement_speed: msg.movement_speed,
                });
                // Populate the root node
                populate_fresh_nodes(&mut self.graph);
            }
            Spawns(msg) => self.handle_spawns(msg),
            StateDelta(msg) => {
                // Discard out-of-order messages, taking care to account for step counter wrapping.
                if self.step.map_or(false, |x| x.wrapping_sub(msg.step) >= 0) {
                    return;
                }
                self.step = Some(msg.step);
                for &(id, new_pos) in &msg.positions {
                    if !self.params.as_ref().map_or(false, |x| x.character_id == id) {
                        self.update_position(id, new_pos);
                    }
                }
                for &(id, orientation) in &msg.character_orientations {
                    match self.entity_ids.get(&id) {
                        None => debug!(%id, "character orientation update for unknown entity"),
                        Some(&entity) => match self.world.get::<&mut Character>(entity) {
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

    fn update_position(&mut self, id: EntityId, new_pos: Position) {
        match self.entity_ids.get(&id) {
            None => debug!(%id, "position update for unknown entity"),
            Some(&entity) => match self.world.get::<&mut Position>(entity) {
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
        if id == self.params.as_ref().unwrap().character_id {
            self.local_character = Some(entity);
        }
        if let Some(x) = self.entity_ids.insert(id, entity) {
            self.destroy_idless(x);
            error!(%id, "id collision");
        }
    }

    fn send_input(&mut self) {
        let generation = self.prediction.push();

        // Any failure here will be better handled in handle_net's ConnectionLost case
        let _ = self.net.outgoing.send(Command {
            generation,
            orientation: self.get_orientation().into(),
            position: self.position,
        });
    }

    pub fn view(&self) -> Position {
        let mut result = self.position;
        result.local *= self.get_orientation().to_homogeneous();
        result
    }

    /// Destroy all aspects of an entity
    fn destroy(&mut self, entity: Entity) {
        let id = *self
            .world
            .get::<&EntityId>(entity)
            .expect("destroyed nonexistent entity");
        self.entity_ids.remove(&id);
        self.destroy_idless(entity);
    }

    /// Destroy an entity without an EntityId mapped
    fn destroy_idless(&mut self, entity: Entity) {
        if let Ok(position) = self.world.get::<&Position>(entity) {
            self.graph_entities.remove(position.node, entity);
        }
        self.world
            .despawn(entity)
            .expect("destroyed nonexistent entity");
    }
}

/// Simulation details received on connect
pub struct Parameters {
    pub step_interval: Duration,
    pub chunk_size: u8,
    pub meters_to_absolute: f32,
    /// Absolute units
    pub movement_speed: f32,
    pub character_id: EntityId,
}

fn populate_fresh_nodes(graph: &mut DualGraph) {
    let fresh = graph.fresh().to_vec();
    graph.clear_fresh();
    for &node in &fresh {
        populate_node(graph, node);
    }
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
