use std::{mem, sync::Arc};

use fxhash::FxHashMap;
use hecs::Entity;
use rand::rngs::SmallRng;
use rand::{Rng, SeedableRng};
use tracing::{debug, error_span, info, trace};

use crate::SimConfig;
use common::{
    graph::{Graph, NodeId},
    math,
    proto::{self, ClientHello, Command, Component, FreshNode, Position, Spawns, StateDelta},
    sanitize_motion_input, EntityId, Step,
};

pub struct Sim {
    cfg: Arc<SimConfig>,
    rng: SmallRng,
    step: Step,
    entity_ids: FxHashMap<EntityId, Entity>,
    world: hecs::World,
    graph: Graph<(), ()>,
    spawns: Vec<Entity>,
    despawns: Vec<EntityId>,
}

impl Sim {
    pub fn new(cfg: Arc<SimConfig>) -> Self {
        let mut result = Self {
            cfg,
            rng: SmallRng::from_entropy(),
            step: 0,
            entity_ids: FxHashMap::default(),
            world: hecs::World::new(),
            graph: Graph::new(),
            spawns: Vec::new(),
            despawns: Vec::new(),
        };
        result
            .graph
            .ensure_nearby(NodeId::ROOT, result.cfg.view_distance);
        result
    }

    pub fn spawn_character(&mut self, hello: ClientHello) -> (EntityId, Entity) {
        let id = self.new_id();
        info!(%id, name = %hello.name, "spawning character");
        let position = Position {
            node: NodeId::ROOT,
            local: na::one(),
        };
        let character = Character {
            name: hello.name,
            latest_input: 0,
            speed: 0.0,
            direction: -na::Vector3::z_axis(),
            orientation: na::one(),
        };
        let entity = self.world.spawn((id, position, character));
        self.spawns.push(entity);
        (id, entity)
    }

    pub fn command(
        &mut self,
        entity: Entity,
        command: Command,
    ) -> Result<(), hecs::ComponentError> {
        let mut ch = self.world.get_mut::<Character>(entity)?;
        if command.generation.wrapping_sub(ch.latest_input) < u16::max_value() / 2 {
            ch.latest_input = command.generation;
            let (direction, speed) = sanitize_motion_input(command.velocity);
            ch.direction = direction;
            ch.speed = speed;
            ch.orientation = command.orientation;
        }
        Ok(())
    }

    pub fn destroy(&mut self, entity: Entity) {
        let id = *self.world.get::<EntityId>(entity).unwrap();
        self.entity_ids.remove(&id);
        self.world.despawn(entity).unwrap();
        self.despawns.push(id);
    }

    /// Collect information about all entities, for transmission to new clients
    pub fn snapshot(&self) -> Spawns {
        let mut spawns = Spawns {
            step: self.step,
            spawns: Vec::new(),
            despawns: Vec::new(),
            nodes: self
                .graph
                .tree()
                .map(|(side, parent)| FreshNode { side, parent })
                .collect(),
        };
        for (entity, &id) in &mut self.world.query::<&EntityId>() {
            spawns.spawns.push((id, dump_entity(&self.world, entity)));
        }
        spawns
    }

    pub fn step(&mut self) -> (Spawns, StateDelta) {
        let span = error_span!("step", step = self.step);
        let _guard = span.enter();

        // Simulate
        for (_, (&id, ch, pos)) in self
            .world
            .query::<(&EntityId, &Character, &mut Position)>()
            .iter()
        {
            let next_xf =
                pos.local * math::translate_along(&ch.direction, ch.speed / self.cfg.rate as f32);
            pos.local = math::renormalize_isometry(&next_xf);
            let (next_node, transition_xf) = self.graph.normalize_transform(pos.node, &pos.local);
            if next_node != pos.node {
                debug!(%id, node = ?next_node, "transition");
                pos.node = next_node;
                pos.local = transition_xf * pos.local;
                self.graph.ensure_nearby(next_node, self.cfg.view_distance);
            }
        }

        // Capture state changes for broadcast to clients
        let mut spawns = Vec::with_capacity(self.spawns.len());
        for entity in self.spawns.drain(..) {
            let id = *self.world.get::<EntityId>(entity).unwrap();
            spawns.push((id, dump_entity(&self.world, entity)));
        }
        if !self.graph.fresh().is_empty() {
            trace!(count = self.graph.fresh().len(), "broadcasting fresh nodes");
        }
        let spawns = Spawns {
            step: self.step,
            spawns,
            despawns: mem::replace(&mut self.despawns, Vec::new()),
            nodes: self
                .graph
                .fresh()
                .iter()
                .filter_map(|&id| {
                    let side = self.graph.parent(id)?;
                    Some(FreshNode {
                        side,
                        parent: self.graph.neighbor(id, side).unwrap(),
                    })
                })
                .collect(),
        };
        self.graph.clear_fresh();

        // TODO: Omit unchanged (e.g. freshly spawned) entities (dirty flag?)
        let delta = StateDelta {
            latest_input: 0, // To be filled in by the caller
            step: self.step,
            positions: self
                .world
                .query::<(&EntityId, &Position)>()
                .iter()
                .map(|(_, (&id, &position))| (id, position))
                .collect(),
            character_orientations: self
                .world
                .query::<(&EntityId, &Character)>()
                .iter()
                .map(|(_, (&id, ch))| (id, ch.orientation))
                .collect(),
        };

        self.step += 1;
        (spawns, delta)
    }

    fn new_id(&mut self) -> EntityId {
        loop {
            let id = self.rng.gen();
            if !self.entity_ids.contains_key(&id) {
                return id;
            }
        }
    }
}

fn dump_entity(world: &hecs::World, entity: Entity) -> Vec<Component> {
    let mut components = Vec::new();
    if let Ok(x) = world.get::<Position>(entity) {
        components.push(Component::Position(*x));
    }
    if let Ok(x) = world.get::<Character>(entity) {
        components.push(Component::Character(proto::Character {
            name: x.name.clone(),
            orientation: x.orientation,
        }));
    }
    components
}

struct Character {
    name: String,
    orientation: na::UnitQuaternion<f32>,
    direction: na::Unit<na::Vector3<f32>>,
    speed: f32,
    latest_input: u16,
}
