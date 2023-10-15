use std::time::Duration;

use fxhash::FxHashMap;
use hecs::Entity;
use tracing::{debug, error, trace};

use crate::{net, prediction::PredictedMotion, Net};
use common::{
    character_controller,
    graph::NodeId,
    node::{populate_fresh_nodes, DualGraph},
    proto::{self, Character, CharacterInput, CharacterState, Command, Component, Position},
    sanitize_motion_input, EntityId, GraphEntities, SimConfig, Step,
};

/// Game state
pub struct Sim {
    // World state
    pub graph: DualGraph,
    pub graph_entities: GraphEntities,
    entity_ids: FxHashMap<EntityId, Entity>,
    pub world: hecs::World,
    pub cfg: SimConfig,
    pub local_character_id: EntityId,
    pub local_character: Option<Entity>,
    orientation: na::UnitQuaternion<f32>,
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
    prediction: PredictedMotion,
}

impl Sim {
    pub fn new(cfg: SimConfig, local_character_id: EntityId) -> Self {
        let mut graph = DualGraph::new();
        populate_fresh_nodes(&mut graph);
        Self {
            graph,
            graph_entities: GraphEntities::new(),
            entity_ids: FxHashMap::default(),
            world: hecs::World::new(),
            cfg,
            local_character_id,
            local_character: None,
            orientation: na::one(),
            step: None,

            since_input_sent: Duration::new(0, 0),
            movement_input: na::zero(),
            average_movement_input: na::zero(),
            no_clip: true,
            toggle_no_clip: false,
            prediction: PredictedMotion::new(proto::Position {
                node: NodeId::ROOT,
                local: na::one(),
            }),
        }
    }

    pub fn rotate(&mut self, delta: &na::UnitQuaternion<f32>) {
        self.orientation *= delta;
    }

    pub fn set_movement_input(&mut self, movement_input: na::Vector3<f32>) {
        self.movement_input = movement_input;
    }

    pub fn toggle_no_clip(&mut self) {
        // We prepare to toggle no_clip after the next step instead of immediately, as otherwise,
        // there would be a discontinuity when predicting the player's position within a given step,
        // causing an undesirable jolt.
        self.toggle_no_clip = true;
    }

    pub fn cfg(&self) -> &SimConfig {
        &self.cfg
    }

    pub fn step(&mut self, dt: Duration, net: &mut Net) {
        self.orientation.renormalize_fast();

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

            // Toggle no clip at the start of a new step
            if self.toggle_no_clip {
                self.no_clip = !self.no_clip;
                self.toggle_no_clip = false;
            }

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
    }

    pub fn handle_net(&mut self, msg: net::Message) {
        use net::Message::*;
        match msg {
            ConnectionLost(_) | Hello(_) => {
                unreachable!("Case already handled by caller");
            }
            Spawns(msg) => self.handle_spawns(msg),
            StateDelta(msg) => {
                // Discard out-of-order messages, taking care to account for step counter wrapping.
                if self.step.map_or(false, |x| x.wrapping_sub(msg.step) >= 0) {
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
        if id == self.local_character_id {
            self.local_character = Some(entity);
        }
        if let Some(x) = self.entity_ids.insert(id, entity) {
            self.destroy_idless(x);
            error!(%id, "id collision");
        }
    }

    fn send_input(&mut self, net: &mut Net) {
        let character_input = CharacterInput {
            movement: sanitize_motion_input(self.orientation * self.average_movement_input),
            no_clip: self.no_clip,
        };
        let generation = self
            .prediction
            .push(&self.cfg, &self.graph, &character_input);

        // Any failure here will be better handled in handle_net's ConnectionLost case
        let _ = net.outgoing.send(Command {
            generation,
            character_input,
            orientation: self.orientation,
        });
    }

    pub fn view(&self) -> Position {
        let mut result = *self.prediction.predicted_position();
        let mut predicted_velocity = *self.prediction.predicted_velocity();
        // Apply input that hasn't been sent yet
        let predicted_input = CharacterInput {
            // We divide by how far we are through the timestep because self.average_movement_input
            // is always over the entire timestep, filling in zeroes for the future, and we
            // want to use the average over what we have so far. Dividing by zero is handled
            // by the character_controller sanitizing this input.
            movement: self.orientation * self.average_movement_input
                / (self.since_input_sent.as_secs_f32() / self.cfg.step_interval.as_secs_f32()),
            no_clip: self.no_clip,
        };
        character_controller::run_character_step(
            &self.cfg,
            &self.graph,
            &mut result,
            &mut predicted_velocity,
            &predicted_input,
            self.since_input_sent.as_secs_f32(),
        );
        result.local *= self.orientation.to_homogeneous();
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
