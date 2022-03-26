use std::time::Duration;

use fxhash::FxHashMap;
use hecs::Entity;
use tracing::{debug, error, trace};

use crate::{net, prediction::PredictedMotion, Net};
use common::{
    collision::BoundingBox,
    force,
    graph::{Graph, NodeId},
    math,
    node::{Chunk, DualGraph, Node},
    proto::{self, Character, Command, Component, Position},
    sanitize_motion_input,
    world::Material,
    worldgen::NodeState,
    Chunks, EntityId, GraphEntities, Step,
};

const CHARACTER_RADIUS: f64 = 0.025_f64;
const CHARACTER_SLOWDOWN_FACTOR: f32 = 6_f32;

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
    character_velocity: na::Vector4<f32>, // velocity of the character controller that carries over from frame to frame.
    orientation: na::UnitQuaternion<f32>,
    step: Option<Step>,

    // Input state
    since_input_sent: Duration,
    /// Most recent input
    ///
    /// Units are relative to movement speed.
    instantaneous_velocity: na::Vector3<f32>,
    /// Average input over the current time step. The portion of the timestep which has not yet
    /// elapsed is considered to have zero input.
    ///
    /// Units are relative to movement speed.
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
            params: None,
            local_character: None,
            character_velocity: na::Vector4::new(0_f32, 0_f32, 0_f32, 0_f32),
            orientation: na::one(),
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

    pub fn rotate(&mut self, delta: &na::UnitQuaternion<f32>) {
        self.orientation *= delta;
    }

    pub fn velocity(&mut self, v: na::Vector3<f32>) {
        self.instantaneous_velocity = v;
    }

    pub fn params(&self) -> Option<&Parameters> {
        self.params.as_ref()
    }

    pub fn step(&mut self, dt: Duration) {
        self.orientation.renormalize_fast();

        while let Ok(msg) = self.net.incoming.try_recv() {
            self.handle_net(msg);
        }

        self.handle_forces(dt);

        if let Some(step_interval) = self.params.as_ref().map(|x| x.step_interval) {
            self.since_input_sent += dt;
            if let Some(overflow) = self.since_input_sent.checked_sub(step_interval) {
                // At least one step interval has passed since we last sent input, so it's time to
                // send again.

                // Update average velocity for the time between the last input sample and the end of
                // the previous step. dt > overflow because we check whether a step has elapsed
                // after each increment.
                self.average_velocity += self.instantaneous_velocity
                    * (dt - overflow).as_secs_f32()
                    / step_interval.as_secs_f32();

                // Send fresh input
                self.send_input();

                // Reset state for the next step
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
                // Update average velocity for the time within the current step
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
                for &(id, new_pos, node_transform) in &msg.positions {
                    self.update_position(msg.latest_input, id, new_pos, node_transform);
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

    fn update_position(
        &mut self,
        latest_input: u16,
        id: EntityId,
        new_pos: Position,
        node_transform: na::Matrix4<f32>,
    ) {
        let id_is_character = self.params.as_ref().map_or(false, |x| x.character_id == id);

        if id_is_character {
            self.prediction.reconcile(latest_input, new_pos);
        }

        match self.entity_ids.get(&id) {
            None => debug!(%id, "position update for unknown entity"),
            Some(&entity) => match self.world.get_mut::<Position>(entity) {
                Ok(mut pos) => {
                    if id_is_character {
                        let previous_charvel = self.character_velocity;

                        // velocity-related stuff.
                        let m = math::mip(&self.character_velocity, &self.character_velocity);

                        let (v_mag, v_norm) = match m <= 0_f32 {
                            true => (0_f32, new_pos.local * na::Vector4::new(1_f32, 0_f32, 0_f32, 0_f32)),
                            false => {
                                let v_mag = m.sqrt();
                                let v_norm = self.character_velocity / v_mag;
                                (v_mag, v_norm)
                            },
                        };
                        
                        let p0 = pos.local
                        * node_transform.try_inverse().unwrap_or(na::Matrix4::zeros())
                        * math::origin();

                        let p1 = new_pos.local * math::origin();

                        assert!((math::mip(&p0, &p0) < 0.0), "Error, p0 is not in the format of a point" );
                        assert!((math::mip(&p1, &p1) < 0.0), "Error, p1 is not in the format of a point" );
                        assert!((math::mip(&v_norm, &v_norm) >= 0.0), "Error, v_norm is not in the format of a vector");

                        self.character_velocity = math::translate_normal(
                            p0,
                            p1,
                            v_norm,
                        ) * v_mag;

                        for n in self.character_velocity.iter() {
                            assert!(!n.is_nan(), "Error, character velocity is not a number. character_velocity was {}. v_mag is {}. 
                            p0 is {}. p1 is {}. Their difference is {}.", previous_charvel, v_mag, p0, p1, (p1 - p0));
                        }
                    }
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
        let params = self.params.as_ref().unwrap();
        let (direction, speed) = sanitize_motion_input(
            self.orientation * self.average_velocity / CHARACTER_SLOWDOWN_FACTOR // lower player abilities
                + na::Vector3::new(
                    self.character_velocity[0],
                    self.character_velocity[1],
                    self.character_velocity[2],
                ) / params.movement_speed,
        );

        let generation = self.prediction.push(
            &direction,
            speed
                * params.movement_speed
                * self.params.as_ref().unwrap().step_interval.as_secs_f32(),
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
        if let Some(ref params) = self.params {
            // Apply input that hasn't been sent yet
            let (direction, speed) = sanitize_motion_input(
                // TODO: Incorparate the gravity into the view calculations.
                self.average_velocity / CHARACTER_SLOWDOWN_FACTOR,
                /*+ self.orientation
                * na::Vector3::new(
                    self.character_velocity[0],
                    self.character_velocity[1],
                    self.character_velocity[2],
                )
                / params.movement_speed,*/
            );
            // We multiply by the entire timestep rather than the time so far because
            // self.average_velocity is always over the entire timestep, filling in zeroes for the
            // future.
            let distance = speed * params.movement_speed * params.step_interval.as_secs_f32();
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

    fn handle_forces(&mut self, time: Duration) {
        // eventually this should be expanded to work on every entity with a physics property, but for now it is just the player
        match self.local_character {
            Some(entity) => match self.world.get_mut::<Position>(entity) {
                Ok(character_position) => {
                    // starting with simplier method for testing purposese
                    let is_colliding = self.check_collision(*character_position);

                    let down_info = &self
                        .graph
                        .get(character_position.node)
                        .as_ref()
                        .unwrap()
                        .state;

                    let character_v4f64 = na::convert::<_, na::Vector4<f64>>(
                        character_position.local * math::origin(),
                    );

                    let down_temp = math::lorentz_normalize(&math::orthogonalize(
                        down_info.surface().normal(),
                        &character_v4f64,
                    ));
                    for n in down_temp.iter() {
                        assert!(!n.is_nan(), "error, down direction is not a nunber");
                    }

                    // the meaning of the surface plane varies based on which side of it you are
                    let down = match down_info.is_sky() {
                        true => down_temp,
                        false => -down_temp,
                    };

                    // let is_colliding = !down_info.is_sky();

                    let height = down_info.surface().distance_to(&character_v4f64);

                    let acceleration_total = force::gravity(&down, height)
                        // currently not doing drag or bouyant as added forces.
                        // + force::normal_buoyant(&down, height, is_colliding)
                        /*+ force::normal_drag(
                            is_colliding,
                            na::convert::<_, na::Vector4<f64>>(self.character_velocity),
                        )*/;

                    if is_colliding {
                        self.character_velocity = na::convert::<_, na::Vector4<f32>>(-down * 0.05_f64); 
                    } else {
                        self.character_velocity *= 0.70_f32.powf(time.as_secs_f32());
                    }

                    self.character_velocity +=
                        na::convert::<_, na::Vector4<f32>>(acceleration_total) * time.as_secs_f32();

                    self.character_velocity = math::normalize_vector(character_position.local, self.character_velocity);
                    
                    for n in self.character_velocity.iter() {
                        assert!(!n.is_nan(), "Error, character velocity is not a number");
                    }
                }
                Err(_e) => (),
            },
            None => (),
        }
    }

    // helper function for handle_forces
    fn check_collision(&self, pos: Position) -> bool {
        let params = self.params.as_ref().unwrap();

        let bb = BoundingBox::create_aabb(
            pos.node,
            na::convert::<na::Vector4<f32>, na::Vector4<f64>>(
                pos.local * na::Vector4::new(0_f32, 0_f32, 0_f32, 1_f32),
            ),
            CHARACTER_RADIUS,
            &self.graph,
            params.chunk_size,
        );

        for cbb in bb.bounding_boxes {
            if match self
                .graph
                .get(cbb.node)
                .as_ref()
                .expect("nodes must be populated before collisons can occur")
                .chunks[cbb.chunk]
            {
                Chunk::Generating => true,
                Chunk::Fresh => true,
                Chunk::Populated {
                    surface: _,
                    ref voxels,
                } => {
                    for voxel_address in cbb.every_voxel() {
                        if match voxels.get(voxel_address as usize) {
                            Material::Void => false,
                            _ => true,
                        } {
                            return true; // I think this is correct
                        }
                    }
                    false
                }
            } {
                return true;
            }
        }
        false
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
