use std::time::Duration;

use fxhash::FxHashMap;
use hecs::Entity;
use tracing::{debug, error, trace};

use crate::{
    chunk_ray_tracer::{RayTracingResult, RayTracingResultHandle},
    graph_ray_tracer, net,
    prediction::PredictedMotion,
    sphere_chunk_ray_tracer::SphereChunkRayTracer,
    Net,
};
use common::{
    graph::{Graph, NodeId},
    math,
    node::{DualGraph, Node},
    proto::{self, Character, Command, Component, Position},
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
    position_local: na::Matrix4<f64>,
    position_node: NodeId,
    yaw: f32,
    pitch: f32,
    step: Option<Step>,

    // Input state
    since_input_sent: Duration,
    /// Most recent input
    ///
    /// Units are relative to movement speed.
    instantaneous_velocity: na::Vector3<f32>,
    vel: na::Vector4<f64>,
    prediction: PredictedMotion,

    max_ground_speed: f64,
    ground_acceleration: f64,
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
            position_local: math::translate_along(&na::Vector3::y_axis(), 1.1),
            position_node: NodeId::ROOT,
            yaw: 0.0,
            pitch: 0.0,
            step: None,

            since_input_sent: Duration::new(0, 0),
            instantaneous_velocity: na::zero(),
            vel: na::zero(),
            prediction: PredictedMotion::new(),

            max_ground_speed: 0.2,
            ground_acceleration: 1.0,
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

            PlayerPhysicsPass { sim: self, dt }.step();
        }
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
            position: Position {
                local: self.position_local.cast(),
                node: self.position_node,
            },
        });
    }

    pub fn view(&self) -> Position {
        Position {
            local: self.position_local.cast::<f32>() * self.get_orientation().to_homogeneous(),
            node: self.position_node,
        }
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

struct PlayerPhysicsPass<'a> {
    sim: &'a mut Sim,
    dt: Duration,
}

impl PlayerPhysicsPass<'_> {
    const MAX_COLLISION_ITERATIONS: u32 = 5;

    fn step(&mut self) {
        self.apply_ground_controls();
        self.apply_velocity();
        self.align_with_gravity();
        self.renormalize_transform();
    }

    fn apply_ground_controls(&mut self) {
        let mut target_unit_vel = (self.sim.get_orientation() * self.sim.instantaneous_velocity)
            .to_homogeneous()
            .cast();
        if target_unit_vel.norm_squared() > 1. {
            target_unit_vel.normalize_mut();
        }

        let target_dvel = target_unit_vel * self.sim.max_ground_speed - self.sim.vel;
        let ground_acceleration_impulse = self.sim.ground_acceleration * self.dt.as_secs_f64();
        if target_dvel.norm_squared() > ground_acceleration_impulse.powi(2) {
            self.sim.vel += target_dvel.normalize() * ground_acceleration_impulse;
        } else {
            self.sim.vel += target_dvel;
        }
    }

    fn apply_velocity(&mut self) {
        let initial_velocity_norm = self.sim.vel.norm();
        let mut remaining_dt = self.dt.as_secs_f64();
        for _ in 0..Self::MAX_COLLISION_ITERATIONS {
            let (ray_tracing_result, ray_tracing_transform) =
                self.trace_ray(&(self.sim.vel * remaining_dt));

            self.sim.position_local *= ray_tracing_transform;

            // TODO: Will need to allow two collision normals to act at once, especially in 3D
            if let Some(intersection) = ray_tracing_result.intersection {
                self.sim.vel = math::project_ortho(&self.sim.vel, &intersection.normal);
                remaining_dt -= ray_tracing_result.t.atanh() / initial_velocity_norm;
            } else {
                break;
            }
        }
    }

    fn align_with_gravity(&mut self) {
        self.sim.position_local *= math::translate2(&na::Vector4::y(), &self.get_relative_up());
    }

    fn renormalize_transform(&mut self) {
        self.sim.position_local = math::renormalize_isometry(&self.sim.position_local);

        let (next_node, transition_xf) = self
            .sim
            .graph
            .normalize_transform(self.sim.position_node, &self.sim.position_local);
        if next_node != self.sim.position_node {
            self.sim.position_node = next_node;
            self.sim.position_local = transition_xf * self.sim.position_local;
        }
    }

    fn get_relative_up(&self) -> na::Vector4<f64> {
        let node = self.sim.graph.get(self.sim.position_node).as_ref().unwrap();
        let mut relative_up =
            self.sim.position_local.try_inverse().unwrap() * node.state.surface().normal();
        relative_up.w = 0.0;
        relative_up.normalize()
    }

    fn trace_ray(
        &self,
        relative_displacement: &na::Vector4<f64>,
    ) -> (RayTracingResult, na::Matrix4<f64>) {
        // Corrective constant to avoid punching through walls with floating point rounding errors
        const EPSILON: f64 = 1e-5;

        let displacement_sqr = math::mip(relative_displacement, relative_displacement);
        if displacement_sqr < 1e-16 {
            return (RayTracingResult::new(0.0), na::Matrix4::identity());
        }

        let displacement_norm = displacement_sqr.sqrt();
        let displacement_normalized = relative_displacement / displacement_norm;

        let mut ray_tracing_result = RayTracingResult::new(displacement_norm.tanh() + EPSILON);
        graph_ray_tracer::trace_ray(
            &self.sim.graph,
            self.sim.params.as_ref().unwrap().chunk_size as usize,
            &SphereChunkRayTracer { radius: 0.02 },
            self.sim.position_node,
            &(self.sim.position_local * na::Vector4::w()),
            &(self.sim.position_local * displacement_normalized),
            &mut RayTracingResultHandle::new(
                &mut ray_tracing_result,
                self.sim.position_local.try_inverse().unwrap(),
            ),
        );

        // TODO: A more robust and complex margin system will likely be needed once the
        // overall algorithm settles more.
        let t_with_epsilon = (ray_tracing_result.t - EPSILON).max(0.0);

        ray_tracing_result.t = t_with_epsilon;
        if let Some(intersection) = ray_tracing_result.intersection.as_mut() {
            intersection.normal.w = 0.0;
            intersection.normal.normalize_mut();
        }

        (
            ray_tracing_result,
            math::translate(
                &na::Vector4::w(),
                &math::lorentz_normalize(
                    &(na::Vector4::w() + displacement_normalized * t_with_epsilon),
                ),
            ),
        )
    }
}
