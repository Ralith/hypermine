use std::mem;

use fxhash::FxHashMap;
use hecs::Entity;
use rand::rngs::SmallRng;
use rand::{Rng, SeedableRng};
use tracing::error_span;

use common::{
    math::HPoint,
    proto::{ClientHello, Command, Component, Position, Spawns, StateDelta},
    world::TileId,
    EntityId, Step,
};

pub struct Sim {
    rng: SmallRng,
    step: Step,
    entity_ids: FxHashMap<EntityId, Entity>,
    world: hecs::World,
    spawns: Vec<Entity>,
    despawns: Vec<EntityId>,
}

impl Sim {
    pub fn new() -> Self {
        Self {
            rng: SmallRng::from_entropy(),
            step: 0,
            entity_ids: FxHashMap::default(),
            world: hecs::World::new(),
            spawns: Vec::new(),
            despawns: Vec::new(),
        }
    }

    pub fn spawn_character(&mut self, _hello: ClientHello) -> (EntityId, Entity) {
        let id = self.new_id();
        let position = Position {
            tile: TileId::ROOT,
            local: HPoint::origin(),
        };
        let entity = self.world.spawn((id, position));
        self.spawns.push(entity);
        (id, entity)
    }

    pub fn command(&mut self, _entity: Entity, _command: Command) {
        // TODO
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
        };
        for (entity, &id) in &mut self.world.query::<&EntityId>() {
            spawns.spawns.push((id, dump_entity(&self.world, entity)));
        }
        spawns
    }

    pub fn step(&mut self) -> (Spawns, StateDelta) {
        let span = error_span!("step", step = self.step);
        let _guard = span.enter();

        // TODO: simulate

        // Capture state changes for broadcast to clients
        let mut spawns = Vec::with_capacity(self.spawns.len());
        for entity in self.spawns.drain(..) {
            let id = *self.world.get::<EntityId>(entity).unwrap();
            spawns.push((id, dump_entity(&self.world, entity)));
        }
        let spawns = Spawns {
            step: self.step,
            spawns,
            despawns: mem::replace(&mut self.despawns, Vec::new()),
        };

        // TODO: Omit unchanged (e.g. freshly spawned) entities
        let delta = StateDelta {
            step: self.step,
            positions: self
                .world
                .query::<(&EntityId, &Position)>()
                .iter()
                .map(|(_, (&id, &position))| (id, position))
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
    components
}
