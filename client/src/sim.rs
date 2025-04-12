use std::time::Duration;

use fxhash::FxHashMap;
use hecs::Entity;
use tracing::{debug, error, trace};

use crate::{
    local_character_controller::LocalCharacterController, metrics, prediction::PredictedMotion,
    worldgen_driver::WorldgenDriver,
};
use common::{
    EntityId, GraphEntities, SimConfig, Step, character_controller,
    collision_math::Ray,
    graph::{Graph, NodeId},
    graph_ray_casting,
    math::{MDirection, MIsometry, MPoint},
    node::{VoxelData, populate_fresh_nodes},
    proto::{
        self, BlockUpdate, Character, CharacterInput, CharacterState, Command, Component,
        Inventory, Position,
    },
    sanitize_motion_input,
    world::Material,
};

const MATERIAL_PALETTE: [Material; 10] = [
    Material::WoodPlanks,
    Material::Grass,
    Material::Dirt,
    Material::Sand,
    Material::Snow,
    Material::WhiteBrick,
    Material::GreyBrick,
    Material::Basalt,
    Material::Water,
    Material::Lava,
];

/// Game state
pub struct Sim {
    // World state
    pub graph: Graph,
    /// Drives chunk generation
    worldgen_driver: WorldgenDriver,
    pub graph_entities: GraphEntities,
    entity_ids: FxHashMap<EntityId, Entity>,
    pub world: hecs::World,
    pub cfg: SimConfig,
    pub local_character_id: EntityId,
    pub local_character: Option<Entity>,
    step: Option<Step>,

    // Input state
    since_input_sent: Duration,
    /// Most recent input
    ///
    /// Units are relative to movement speed.
    movement_input: na::Vector3<f32>,
    /// Average input over the current time step. The portion of the timestep which has not yet
    /// elapsed is considered to have zero input.
    ///
    /// Units are relative to movement speed.
    average_movement_input: na::Vector3<f32>,
    no_clip: bool,
    /// Whether no_clip will be toggled next step
    toggle_no_clip: bool,
    /// Whether the current step starts with a jump
    is_jumping: bool,
    /// Whether the jump button has been pressed since the last step
    jump_pressed: bool,
    /// Whether the jump button is currently held down
    jump_held: bool,
    /// Whether the place-block button has been pressed since the last step
    place_block_pressed: bool,
    /// Whether the break-block button has been pressed since the last step
    break_block_pressed: bool,

    selected_material: Material,

    prediction: PredictedMotion,
    local_character_controller: LocalCharacterController,
}

impl Sim {
    pub fn new(
        cfg: SimConfig,
        chunk_load_parallelism: usize,
        local_character_id: EntityId,
    ) -> Self {
        let mut graph = Graph::new(cfg.chunk_size);
        populate_fresh_nodes(&mut graph);
        Self {
            graph,
            worldgen_driver: WorldgenDriver::new(chunk_load_parallelism),
            graph_entities: GraphEntities::new(),
            entity_ids: FxHashMap::default(),
            world: hecs::World::new(),
            cfg,
            local_character_id,
            local_character: None,
            step: None,

            since_input_sent: Duration::new(0, 0),
            movement_input: na::zero(),
            average_movement_input: na::zero(),
            no_clip: true,
            toggle_no_clip: false,
            is_jumping: false,
            jump_pressed: false,
            jump_held: false,
            place_block_pressed: false,
            break_block_pressed: false,
            selected_material: Material::WoodPlanks,
            prediction: PredictedMotion::new(proto::Position {
                node: NodeId::ROOT,
                local: MIsometry::identity(),
            }),
            local_character_controller: LocalCharacterController::new(),
        }
    }

    /// Rotates the camera's view in a context-dependent manner based on the desired yaw and pitch angles.
    pub fn look(&mut self, delta_yaw: f32, delta_pitch: f32, delta_roll: f32) {
        if self.no_clip {
            self.local_character_controller
                .look_free(delta_yaw, delta_pitch, delta_roll);
        } else {
            self.local_character_controller
                .look_level(delta_yaw, delta_pitch);
        }
    }

    pub fn set_movement_input(&mut self, mut raw_movement_input: na::Vector3<f32>) {
        if !self.no_clip {
            // Vertical movement keys shouldn't do anything unless no-clip is on.
            raw_movement_input.y = 0.0;
        }
        if raw_movement_input.norm_squared() >= 1.0 {
            // Cap movement input at 1
            raw_movement_input.normalize_mut();
        }
        self.movement_input = raw_movement_input;
    }

    pub fn toggle_no_clip(&mut self) {
        // We prepare to toggle no_clip after the next step instead of immediately, as otherwise,
        // there would be a discontinuity when predicting the player's position within a given step,
        // causing an undesirable jolt.
        self.toggle_no_clip = true;
    }

    pub fn set_jump_held(&mut self, jump_held: bool) {
        self.jump_held = jump_held;
        self.jump_pressed = jump_held || self.jump_pressed;
    }

    pub fn set_jump_pressed_true(&mut self) {
        self.jump_pressed = true;
    }

    pub fn set_place_block_pressed_true(&mut self) {
        self.place_block_pressed = true;
    }

    pub fn select_material(&mut self, idx: usize) {
        self.selected_material = *MATERIAL_PALETTE.get(idx).unwrap_or(&MATERIAL_PALETTE[0]);
    }

    pub fn selected_material(&self) -> Material {
        self.selected_material
    }

    /// Returns an EntityId in the inventory with the given material
    pub fn get_any_inventory_entity_matching_material(
        &self,
        material: Material,
    ) -> Option<EntityId> {
        self.world
            .get::<&Inventory>(self.local_character?)
            .ok()?
            .contents
            .iter()
            .copied()
            .find(|e| {
                self.entity_ids.get(e).is_some_and(|&entity| {
                    self.world
                        .get::<&Material>(entity)
                        .is_ok_and(|m| *m == material)
                })
            })
    }

    /// Returns the number of entities in the inventory with the given material
    pub fn count_inventory_entities_matching_material(&self, material: Material) -> usize {
        let Some(local_character) = self.local_character else {
            return 0;
        };
        let Ok(inventory) = self.world.get::<&Inventory>(local_character) else {
            return 0;
        };
        inventory
            .contents
            .iter()
            .copied()
            .filter(|e| {
                self.entity_ids.get(e).is_some_and(|&entity| {
                    self.world
                        .get::<&Material>(entity)
                        .is_ok_and(|m| *m == material)
                })
            })
            .count()
    }

    pub fn set_break_block_pressed_true(&mut self) {
        self.break_block_pressed = true;
    }

    pub fn cfg(&self) -> &SimConfig {
        &self.cfg
    }

    pub fn step(&mut self, dt: Duration, net: &mut server::Handle) {
        self.local_character_controller.renormalize_orientation();
        self.worldgen_driver
            .drive(self.view(), self.cfg.view_distance, &mut self.graph);

        let step_interval = self.cfg.step_interval;
        self.since_input_sent += dt;
        if let Some(overflow) = self.since_input_sent.checked_sub(step_interval) {
            // At least one step interval has passed since we last sent input, so it's time to
            // send again.

            // Update average movement input for the time between the last input sample and the end of
            // the previous step. dt > overflow because we check whether a step has elapsed
            // after each increment.
            self.average_movement_input +=
                self.movement_input * (dt - overflow).as_secs_f32() / step_interval.as_secs_f32();

            // Send fresh input
            self.send_input(net);
            self.place_block_pressed = false;
            self.break_block_pressed = false;

            // Toggle no clip at the start of a new step
            if self.toggle_no_clip {
                self.no_clip = !self.no_clip;
                self.toggle_no_clip = false;
            }

            self.is_jumping = self.jump_held || self.jump_pressed;
            self.jump_pressed = false;

            // Reset state for the next step
            if overflow > step_interval {
                // If it's been more than two timesteps since we last sent input, skip ahead
                // rather than spamming the server.
                self.average_movement_input = na::zero();
                self.since_input_sent = Duration::new(0, 0);
            } else {
                self.average_movement_input =
                    self.movement_input * overflow.as_secs_f32() / step_interval.as_secs_f32();
                // Send the next input a little sooner if necessary to stay in sync
                self.since_input_sent = overflow;
            }
        } else {
            // Update average movement input for the time within the current step
            self.average_movement_input +=
                self.movement_input * dt.as_secs_f32() / step_interval.as_secs_f32();
        }
        self.update_view_position();
        if !self.no_clip {
            self.local_character_controller.align_to_gravity();
        }
    }

    pub fn handle_net(&mut self, msg: server::Message) {
        use server::Message::*;
        match msg {
            ConnectionLost(_) | Hello(_) => {
                unreachable!("Case already handled by caller");
            }
            Spawns(msg) => self.handle_spawns(msg),
            StateDelta(msg) => {
                // Discard out-of-order messages, taking care to account for step counter wrapping.
                if self.step.is_some_and(|x| x.wrapping_sub(msg.step) >= 0) {
                    return;
                }
                self.step = Some(msg.step);
                for &(id, ref new_pos) in &msg.positions {
                    self.update_position(id, new_pos);
                }
                for &(id, ref new_state) in &msg.character_states {
                    self.update_character_state(id, new_state);
                }
                self.reconcile_prediction(msg.latest_input);
            }
        }
    }

    fn update_position(&mut self, id: EntityId, new_pos: &Position) {
        match self.entity_ids.get(&id) {
            None => debug!(%id, "position update for unknown entity"),
            Some(&entity) => match self.world.get::<&mut Position>(entity) {
                Ok(mut pos) => {
                    if pos.node != new_pos.node {
                        self.graph_entities.remove(pos.node, entity);
                        self.graph_entities.insert(new_pos.node, entity);
                    }
                    *pos = *new_pos;
                }
                Err(e) => error!(%id, "position update error: {}", e),
            },
        }
    }

    fn update_character_state(&mut self, id: EntityId, new_character_state: &CharacterState) {
        match self.entity_ids.get(&id) {
            None => debug!(%id, "character state update for unknown entity"),
            Some(&entity) => match self.world.get::<&mut Character>(entity) {
                Ok(mut ch) => {
                    ch.state = new_character_state.clone();
                }
                Err(e) => {
                    error!(%id, "character state update error: {}", e)
                }
            },
        }
    }

    fn reconcile_prediction(&mut self, latest_input: u16) {
        let id = self.local_character_id;
        let Some(&entity) = self.entity_ids.get(&id) else {
            debug!(%id, "reconciliation attempted for unknown entity");
            return;
        };
        let pos = match self.world.get::<&Position>(entity) {
            Ok(pos) => pos,
            Err(e) => {
                error!(%id, "reconciliation error: {}", e);
                return;
            }
        };
        let ch = match self.world.get::<&Character>(entity) {
            Ok(ch) => ch,
            Err(e) => {
                error!(%id, "reconciliation error: {}", e);
                return;
            }
        };
        self.prediction.reconcile(
            &self.cfg,
            &self.graph,
            latest_input,
            *pos,
            ch.state.velocity,
            ch.state.on_ground,
        );
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
            // The first "Spawns" message from the server populates the graph and allows CPU/GPU metrics
            // to be accurate instead of measuring thousands of no-op frames
            metrics::declare_ready_for_profiling();
        }
        for node in &msg.nodes {
            self.graph.insert_neighbor(node.parent, node.side);
        }
        populate_fresh_nodes(&mut self.graph);
        for block_update in msg.block_updates.into_iter() {
            self.worldgen_driver
                .apply_block_update(&mut self.graph, block_update);
        }
        for (chunk_id, voxel_data) in msg.voxel_data {
            let Some(voxel_data) = VoxelData::deserialize(&voxel_data, self.cfg.chunk_size) else {
                tracing::error!("Voxel data received from server is of incorrect dimension");
                continue;
            };
            self.worldgen_driver
                .apply_voxel_data(&mut self.graph, chunk_id, voxel_data);
        }
        for (subject, new_entity) in msg.inventory_additions {
            self.world
                .get::<&mut Inventory>(*self.entity_ids.get(&subject).unwrap())
                .unwrap()
                .contents
                .push(new_entity);
        }
        for (subject, removed_entity) in msg.inventory_removals {
            self.world
                .get::<&mut Inventory>(*self.entity_ids.get(&subject).unwrap())
                .unwrap()
                .contents
                .retain(|&id| id != removed_entity);
        }
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
                Inventory(x) => {
                    builder.add(x);
                }
                Material(x) => {
                    builder.add(x);
                }
            };
        }
        let entity = self.world.spawn(builder.build());
        if let Some(node) = node {
            self.graph_entities.insert(node, entity);
        }
        if id == self.local_character_id {
            self.local_character = Some(entity);
        }
        if let Some(x) = self.entity_ids.insert(id, entity) {
            self.destroy_idless(x);
            error!(%id, "id collision");
        }
    }

    fn send_input(&mut self, net: &mut server::Handle) {
        let orientation = if self.no_clip {
            self.local_character_controller.orientation()
        } else {
            self.local_character_controller.horizontal_orientation()
        };
        let character_input = CharacterInput {
            movement: sanitize_motion_input(orientation * self.average_movement_input),
            jump: self.is_jumping,
            no_clip: self.no_clip,
            block_update: self.get_local_character_block_update(),
        };
        let generation = self
            .prediction
            .push(&self.cfg, &self.graph, &character_input);

        // Any failure here will be better handled in handle_net's ConnectionLost case
        let _ = net.outgoing.send(Command {
            generation,
            character_input,
            orientation: self.local_character_controller.orientation(),
        });
    }

    fn update_view_position(&mut self) {
        let mut view_position = *self.prediction.predicted_position();
        let mut view_velocity = *self.prediction.predicted_velocity();
        let mut view_on_ground = *self.prediction.predicted_on_ground();
        let orientation = if self.no_clip {
            self.local_character_controller.orientation()
        } else {
            self.local_character_controller.horizontal_orientation()
        };
        // Apply input that hasn't been sent yet
        let predicted_input = CharacterInput {
            // We divide by how far we are through the timestep because self.average_movement_input
            // is always over the entire timestep, filling in zeroes for the future, and we
            // want to use the average over what we have so far. Dividing by zero is handled
            // by the character_controller sanitizing this input.
            movement: orientation * self.average_movement_input
                / (self.since_input_sent.as_secs_f32() / self.cfg.step_interval.as_secs_f32()),
            jump: self.is_jumping,
            no_clip: self.no_clip,
            block_update: None,
        };
        character_controller::run_character_step(
            &self.cfg,
            &self.graph,
            &mut view_position,
            &mut view_velocity,
            &mut view_on_ground,
            &predicted_input,
            self.since_input_sent.as_secs_f32(),
        );

        self.local_character_controller.update_position(
            view_position,
            self.graph.get_relative_up(&view_position).unwrap(),
            !self.no_clip,
        )
    }

    pub fn view(&self) -> Position {
        let mut pos = self.local_character_controller.oriented_position();
        let up = self.graph.get_relative_up(&pos).unwrap();
        pos.local *= MIsometry::translation_along(
            &(up.as_ref() * (self.cfg.character.character_radius - 1e-3)),
        );
        pos
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

    /// Provides the logic for the player to be able to place and break blocks at will
    fn get_local_character_block_update(&self) -> Option<BlockUpdate> {
        let placing = if self.place_block_pressed {
            true
        } else if self.break_block_pressed {
            false
        } else {
            return None;
        };

        let view_position = self.view();
        let ray_casing_result = graph_ray_casting::ray_cast(
            &self.graph,
            &view_position,
            &Ray::new(MPoint::w(), -MDirection::z()),
            self.cfg.character.block_reach,
        );

        let Ok(ray_casting_result) = ray_casing_result else {
            tracing::warn!("Tried to run a raycast beyond generated terrain.");
            return None;
        };

        let hit = ray_casting_result?;

        let block_pos = if placing {
            self.graph.get_block_neighbor(
                hit.chunk,
                hit.voxel_coords,
                hit.face_axis,
                hit.face_sign,
            )?
        } else {
            (hit.chunk, hit.voxel_coords)
        };

        let material = if placing {
            self.selected_material
        } else {
            Material::Void
        };

        let consumed_entity = if placing && self.cfg.gameplay_enabled {
            Some(self.get_any_inventory_entity_matching_material(material)?)
        } else {
            None
        };

        Some(BlockUpdate {
            chunk_id: block_pos.0,
            coords: block_pos.1,
            new_material: material,
            consumed_entity,
        })
    }
}
