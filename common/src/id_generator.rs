use crate::EntityId;
use fxhash::FxHashMap;
use hecs::Entity;
use rand::rngs::SmallRng;
use rand::{Rng, SeedableRng};

pub struct IDGenerator {
    pub rng: SmallRng,
    pub entity_ids: FxHashMap<EntityId, Entity>,
}

impl IDGenerator {
    pub fn new() -> Self {
        Self {
            rng: SmallRng::from_entropy(),
            entity_ids: FxHashMap::default(),
        }
    }
    pub fn new_id(&mut self) -> EntityId {
        loop {
            let id = self.rng.gen();
            if !self.entity_ids.contains_key(&id) {
                return id;
            }
        }
    }
}

impl Default for IDGenerator {
    fn default() -> Self {
        IDGenerator::new()
    }
}
