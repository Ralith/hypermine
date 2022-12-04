use std::time::Duration;

use fxhash::FxHashMap;
use hecs::Entity;
use tracing::{debug, error, trace};

use crate::{
    chunk_ray_tracer::{RayTracingResult, RayTracingResultHandle},
    graph_ray_tracer, net,
    point_chunk_ray_tracer::PointChunkRayTracer,
    prediction::PredictedMotion,
    single_block_sphere_collision_checker::SingleBlockSphereCollisionChecker,
    sphere_chunk_ray_tracer::SphereChunkRayTracer,
    Net,
};
use common::{
    dodeca::Vertex,
    graph::{Graph, NodeId},
    math,
    node::{Chunk, DualGraph, Node, VoxelData},
    proto::{self, Character, Command, Component, Position},
    sanitize_motion_input,
    world::Material,
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
    noclip: bool,
    step: Option<Step>,

    // Input state
    since_input_sent: Duration,
    /// Most recent input
    ///
    /// Units are relative to movement speed.
    instantaneous_velocity: na::Vector3<f32>,
    jumping: bool,
    breaking_block: bool,
    placing_block: bool,
    break_block_timer: Option<f64>,
    place_block_timer: Option<f64>,

    prediction: PredictedMotion,

    vel: na::Vector4<f64>,
    ground_normal: Option<na::Vector4<f64>>,

    max_ground_speed: f64,
    ground_acceleration: f64,
    air_acceleration: f64,
    jump_speed: f64,
    max_cos_slope: f64,
    radius: f64,
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
            noclip: false,
            step: None,

            since_input_sent: Duration::new(0, 0),
            instantaneous_velocity: na::zero(),
            jumping: false,
            breaking_block: false,
            placing_block: false,
            break_block_timer: None,
            place_block_timer: None,

            prediction: PredictedMotion::new(),

            vel: na::zero(),
            ground_normal: None,

            max_ground_speed: 0.2,
            ground_acceleration: 1.0,
            air_acceleration: 0.2,
            jump_speed: 0.4,
            max_cos_slope: 0.5,
            radius: 0.02,
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

    pub fn jump(&mut self) {
        self.jumping = true;
    }

    pub fn set_noclip(&mut self, noclip: bool) {
        self.noclip = noclip;
    }

    pub fn break_block(&mut self) {
        self.breaking_block = true;
        self.break_block_timer = Some(0.2);
    }

    pub fn place_block(&mut self) {
        self.placing_block = true;
        self.place_block_timer = Some(0.2);
    }

    pub fn keep_breaking_blocks(&mut self, keep_breaking_blocks: bool) {
        if !keep_breaking_blocks {
            self.break_block_timer = None;
        }
    }

    pub fn keep_placing_blocks(&mut self, keep_placing_blocks: bool) {
        if !keep_placing_blocks {
            self.place_block_timer = None;
        }
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

            if self.noclip {
                self.vel = na::Vector4::zeros();
                let movement_speed = self.params.as_ref().unwrap().movement_speed;
                let (direction, speed) = sanitize_motion_input(
                    self.get_orientation() * self.instantaneous_velocity * movement_speed,
                );
                self.position_local *=
                    math::translate_along(&direction.cast(), speed as f64 * dt.as_secs_f64());
                PlayerPhysicsPass { sim: self, dt }.align_with_gravity();
                PlayerPhysicsPass { sim: self, dt }.renormalize_transform();
            } else {
                PlayerPhysicsPass { sim: self, dt }.step();
            }
            self.jumping = false;

            if let Some(ref mut break_block_timer) = self.break_block_timer {
                *break_block_timer -= dt.as_secs_f64();
                if *break_block_timer <= 0.0 {
                    self.break_block();
                }
            }

            if let Some(ref mut place_block_timer) = self.place_block_timer {
                *place_block_timer -= dt.as_secs_f64();
                if *place_block_timer <= 0.0 {
                    self.place_block();
                }
            }

            self.handle_breaking_or_placing_blocks(false);
            self.handle_breaking_or_placing_blocks(true);
        }
    }

    fn handle_breaking_or_placing_blocks(&mut self, placing: bool) {
        if placing {
            if self.placing_block {
                self.placing_block = false;
            } else {
                return;
            }
        } else if !placing {
            if self.breaking_block {
                self.breaking_block = false;
            } else {
                return;
            }
        }

        let dimension = self.params.as_ref().unwrap().chunk_size;

        let mut ray_tracing_result = RayTracingResult::new(0.5);
        if !graph_ray_tracer::trace_ray(
            &self.graph,
            self.params.as_ref().unwrap().chunk_size as usize,
            &PointChunkRayTracer {},
            self.position_node,
            &(self.position_local * na::Vector4::w()),
            &(self.position_local
                * self.get_orientation().cast().to_homogeneous()
                * -na::Vector4::z()),
            &mut RayTracingResultHandle::new(
                &mut ray_tracing_result,
                self.position_node,
                common::dodeca::Vertex::A,
                self.position_local.try_inverse().unwrap(),
            ),
        ) {
            return;
        }

        if let Some(intersection) = ray_tracing_result.intersection {
            let Some(block_pos) = (if placing {
                self.get_block_neighbor(
                    intersection.node,
                    intersection.vertex,
                    intersection.voxel_coords,
                    intersection.coord_axis,
                    intersection.coord_direction,
                )
            } else {
                Some((
                    intersection.node,
                    intersection.vertex,
                    intersection.voxel_coords,
                ))
            }) else {
                return;
            };

            let conflict =
                placing && self.placing_has_conflict(block_pos.0, block_pos.1, block_pos.2);

            let mut must_fix_neighboring_chunks = false;

            if let Some(node) = self.graph.get_mut(block_pos.0) {
                if let Chunk::Populated {
                    voxels,
                    surface,
                    old_surface,
                } = &mut node.chunks[block_pos.1]
                {
                    let data = voxels.data_mut(dimension);
                    let lwm = dimension as usize + 2;
                    let array_entry = (block_pos.2[0] + 1)
                        + (block_pos.2[1] + 1) * lwm
                        + (block_pos.2[2] + 1) * lwm * lwm;
                    if placing {
                        if data[array_entry] == Material::Void && !conflict {
                            data[array_entry] = Material::WoodPlanks;
                            must_fix_neighboring_chunks = true;
                        }
                    } else {
                        data[array_entry] = Material::Void;
                        must_fix_neighboring_chunks = true;
                    }

                    *old_surface = *surface;
                    *surface = None;
                }
            }

            if must_fix_neighboring_chunks {
                self.turn_neighboring_solid_to_dense(block_pos.0, block_pos.1, block_pos.2, 0, 1);
                self.turn_neighboring_solid_to_dense(block_pos.0, block_pos.1, block_pos.2, 0, -1);
                self.turn_neighboring_solid_to_dense(block_pos.0, block_pos.1, block_pos.2, 1, 1);
                self.turn_neighboring_solid_to_dense(block_pos.0, block_pos.1, block_pos.2, 1, -1);
                self.turn_neighboring_solid_to_dense(block_pos.0, block_pos.1, block_pos.2, 2, 1);
                self.turn_neighboring_solid_to_dense(block_pos.0, block_pos.1, block_pos.2, 2, -1);
            }
        }
    }

    fn turn_neighboring_solid_to_dense(
        &mut self,
        node: NodeId,
        vertex: Vertex,
        coords: [usize; 3],
        coord_axis: usize,
        coord_direction: isize,
    ) {
        let dimension = self.params.as_ref().unwrap().chunk_size;

        if let Some((neighbor_node, neighbor_vertex, _neighbor_coords)) =
            self.get_block_neighbor(node, vertex, coords, coord_axis, coord_direction)
        {
            if let Some(neighbor_node_data) = self.graph.get_mut(neighbor_node) {
                if let Chunk::Populated {
                    voxels,
                    surface,
                    old_surface,
                } = &mut neighbor_node_data.chunks[neighbor_vertex]
                {
                    if matches!(voxels, VoxelData::Solid(..)) {
                        // This function has the side effect of turning solid chunks into dense chunks,
                        // which is what we want for them to render properly.
                        voxels.data_mut(dimension);
                        *old_surface = *surface;
                        *surface = None;
                    }
                }
            }
        }
    }

    fn get_block_neighbor(
        &self,
        mut node: NodeId,
        mut vertex: Vertex,
        mut coords: [usize; 3],
        coord_axis: usize,
        coord_direction: isize,
    ) -> Option<(NodeId, Vertex, [usize; 3])> {
        let dimension = self.params.as_ref().unwrap().chunk_size as usize;
        if coords[coord_axis] == dimension - 1 && coord_direction == 1 {
            let new_vertex = vertex.adjacent_vertices()[coord_axis];
            let coord_plane0 = (coord_axis + 1) % 3;
            let coord_plane1 = (coord_axis + 2) % 3;
            let mut new_coords: [usize; 3] = [0; 3];
            for (i, new_coord) in new_coords.iter_mut().enumerate() {
                if new_vertex.canonical_sides()[i] == vertex.canonical_sides()[coord_plane0] {
                    *new_coord = coords[coord_plane0];
                } else if new_vertex.canonical_sides()[i] == vertex.canonical_sides()[coord_plane1]
                {
                    *new_coord = coords[coord_plane1];
                } else {
                    *new_coord = coords[coord_axis];
                }
            }
            coords = new_coords;
            vertex = new_vertex;
        } else if coords[coord_axis] == 0 && coord_direction == -1 {
            node = self
                .graph
                .neighbor(node, vertex.canonical_sides()[coord_axis])?;
        } else {
            coords[coord_axis] = (coords[coord_axis] as isize + coord_direction) as usize;
        }

        Some((node, vertex, coords))
    }

    fn placing_has_conflict(&self, node: NodeId, vertex: Vertex, coords: [usize; 3]) -> bool {
        const EPSILON: f64 = 1e-7;

        let mut ray_tracing_result = RayTracingResult::new(0.0);
        if !graph_ray_tracer::trace_ray(
            &self.graph,
            self.params.as_ref().unwrap().chunk_size as usize,
            &SingleBlockSphereCollisionChecker {
                node,
                vertex,
                coords,
                radius: self.radius + EPSILON,
            },
            self.position_node,
            &(self.position_local * na::Vector4::w()),
            &(self.position_local
                * self.get_orientation().cast().to_homogeneous()
                * -na::Vector4::z()),
            &mut RayTracingResultHandle::new(
                &mut ray_tracing_result,
                self.position_node,
                common::dodeca::Vertex::A,
                self.position_local.try_inverse().unwrap(),
            ),
        ) {
            return true; // Unsafe to place blocks when collision check is inconclusive
        };

        ray_tracing_result.intersection.is_some()
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
        if self.sim.jumping {
            self.attempt_jump();
        }

        if let Some(ground_normal) = self.sim.ground_normal {
            self.apply_ground_controls(&ground_normal);
        } else {
            self.apply_air_controls();
            self.apply_gravity();
        }

        self.apply_velocity();
        self.align_with_gravity();

        if self.sim.ground_normal.is_some() {
            self.clamp_to_ground_or_start_falling();
        }

        self.renormalize_transform();
    }

    fn attempt_jump(&mut self) {
        if self.sim.ground_normal.is_some() {
            let relative_up = self.get_relative_up();
            let horizontal_vel = math::project_ortho(&self.sim.vel, &relative_up);
            self.sim.vel = horizontal_vel + relative_up * self.sim.jump_speed;
            self.sim.ground_normal = None;
        }
    }

    fn apply_ground_controls(&mut self, ground_normal: &na::Vector4<f64>) {
        let mut target_unit_vel =
            (na::Rotation::from_axis_angle(&na::Vector3::y_axis(), -self.sim.yaw)
                * self.sim.instantaneous_velocity)
                .to_homogeneous()
                .cast();
        if target_unit_vel.norm_squared() > 1. {
            target_unit_vel.normalize_mut();
        }
        target_unit_vel = math::translate2(&na::Vector4::y(), ground_normal) * target_unit_vel;

        let target_dvel = target_unit_vel * self.sim.max_ground_speed - self.sim.vel;
        let ground_acceleration_impulse = self.sim.ground_acceleration * self.dt.as_secs_f64();
        if target_dvel.norm_squared() > ground_acceleration_impulse.powi(2) {
            self.sim.vel += target_dvel.normalize() * ground_acceleration_impulse;
        } else {
            self.sim.vel += target_dvel;
        }
    }

    fn apply_air_controls(&mut self) {
        self.sim.vel += (na::Rotation::from_axis_angle(&na::Vector3::y_axis(), -self.sim.yaw)
            * self.sim.instantaneous_velocity)
            .to_homogeneous()
            .cast()
            * self.dt.as_secs_f64()
            * self.sim.air_acceleration;
    }

    fn apply_gravity(&mut self) {
        const GRAVITY: f64 = 1.0;
        self.sim.vel -= self.get_relative_up() * GRAVITY * self.dt.as_secs_f64();
    }

    fn apply_velocity(&mut self) {
        let initial_velocity_norm = self.sim.vel.norm();

        let mut active_normals = Vec::<na::Vector4<f64>>::with_capacity(2);
        let mut remaining_dt = self.dt.as_secs_f64();
        for _ in 0..Self::MAX_COLLISION_ITERATIONS {
            let (ray_tracing_result, ray_tracing_transform) =
                self.trace_ray(&(self.sim.vel * remaining_dt));

            self.sim.position_local *= ray_tracing_transform;

            if let Some(intersection) = ray_tracing_result.intersection {
                if math::mip(&intersection.normal, &self.get_relative_up()) > self.sim.max_cos_slope
                {
                    self.update_ground_normal(&intersection.normal);

                    // The update of the ground normal is not a simple projection, so some normals
                    // will need to be deactivated to avoid the player getting stuck in certain edge
                    // cases. For this, we only retain normals the player is moving towards.
                    active_normals.retain(|n| n.dot(&self.sim.vel) < 0.0);
                } else {
                    active_normals.retain(|n| n.dot(&intersection.normal) < 0.0);
                    active_normals.push(intersection.normal);
                }

                self.sim.vel = self.apply_normals(
                    active_normals
                        .clone()
                        .into_iter()
                        .chain(self.sim.ground_normal)
                        .collect(),
                    self.sim.vel,
                );

                remaining_dt -= ray_tracing_result.t.atanh() / initial_velocity_norm;
            } else {
                break;
            }
        }
    }

    fn apply_normals(
        &self,
        mut normals: Vec<na::Vector4<f64>>,
        mut subject: na::Vector4<f64>,
    ) -> na::Vector4<f64> {
        // In this method, the w-coordinate is assumed to be 0 for all vectors passed in.

        if normals.len() >= 3 {
            // The normals are assumed to be linearly independent with w coordinate 0,
            // so applying all of them will zero out the subject.
            return na::Vector4::zeros();
        }

        for i in 0..normals.len() {
            for j in i + 1..normals.len() {
                normals[j] = math::project_ortho(&normals[j], &normals[i]).normalize();
            }
            subject = math::project_ortho(&subject, &normals[i]);
        }
        subject
    }

    fn align_with_gravity(&mut self) {
        let transformation = math::translate2(&na::Vector4::y(), &self.get_relative_up());
        self.sim.position_local *= transformation;
        let transformation_inverse = transformation.try_inverse().unwrap();
        self.sim.vel = transformation_inverse * self.sim.vel;
        self.sim.ground_normal = self.sim.ground_normal.map(|n| transformation_inverse * n);
    }

    fn clamp_to_ground_or_start_falling(&mut self) {
        let mut clamp_vector = -na::Vector4::y() * 0.01;
        let mut active_normals = Vec::<na::Vector4<f64>>::with_capacity(2);
        for _ in 0..Self::MAX_COLLISION_ITERATIONS {
            let (ray_tracing_result, ray_tracing_transform) = self.trace_ray(&clamp_vector);

            if let Some(intersection) = ray_tracing_result.intersection {
                let potential_transform = self.sim.position_local * ray_tracing_transform;
                if math::mip(&intersection.normal, &self.get_relative_up()) > self.sim.max_cos_slope
                {
                    self.sim.position_local = potential_transform;
                    self.update_ground_normal(&intersection.normal);
                    return;
                } else {
                    active_normals.retain(|n| n.dot(&intersection.normal) < 0.0);
                    active_normals.push(intersection.normal);

                    // Shrink clamp vector based on travel distance. This is an approximation based on clamp_vector being small.
                    // More accurate shrinkage can be found at apply_velocity_iteration.
                    clamp_vector -= ray_tracing_transform.column(3);
                    clamp_vector.w = 0.0;

                    // Adjust clamp vector to be perpendicular to the normal vector.
                    clamp_vector = self.apply_normals(active_normals.clone(), clamp_vector);
                }
            } else {
                break;
            }
        }
        self.sim.ground_normal = None;
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

    fn update_ground_normal(&mut self, new_ground_normal: &na::Vector4<f64>) {
        if let Some(ground_normal) = self.sim.ground_normal {
            // Move vel from ground_normal to new_ground_normal
            self.sim.vel = math::translate2(&ground_normal, new_ground_normal) * self.sim.vel;
        } else {
            // To avoid undesirable sliding down slopes on every jump, project to a level ground plane
            // before projecting to the actual group plane.
            self.sim.vel = math::project_ortho(
                &math::project_ortho(&self.sim.vel, &self.get_relative_up()),
                new_ground_normal,
            );
        }
        self.sim.ground_normal = Some(*new_ground_normal);
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
        if !graph_ray_tracer::trace_ray(
            &self.sim.graph,
            self.sim.params.as_ref().unwrap().chunk_size as usize,
            &SphereChunkRayTracer {
                radius: self.sim.radius,
            },
            self.sim.position_node,
            &(self.sim.position_local * na::Vector4::w()),
            &(self.sim.position_local * displacement_normalized),
            &mut RayTracingResultHandle::new(
                &mut ray_tracing_result,
                self.sim.position_node,
                common::dodeca::Vertex::A,
                self.sim.position_local.try_inverse().unwrap(),
            ),
        ) {
            return (RayTracingResult::new(0.0), na::Matrix4::identity());
        }

        // TODO: A more robust and complex margin system will likely be needed once the
        // overall algorithm settles more.
        let t_with_epsilon = (ray_tracing_result.t - EPSILON).max(0.0);

        let ray_tracing_transform = math::translate(
            &na::Vector4::w(),
            &math::lorentz_normalize(
                &(na::Vector4::w() + displacement_normalized * t_with_epsilon),
            ),
        );

        ray_tracing_result.t = t_with_epsilon;
        if let Some(intersection) = ray_tracing_result.intersection.as_mut() {
            intersection.normal =
                ray_tracing_transform.try_inverse().unwrap() * intersection.normal;
            intersection.normal.w = 0.0;
            intersection.normal.normalize_mut();
        }

        (ray_tracing_result, ray_tracing_transform)
    }
}
