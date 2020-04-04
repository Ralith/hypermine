use fxhash::FxHashMap;
use hecs::Entity;

use crate::graph::NodeId;

#[derive(Default)]
pub struct GraphEntities {
    map: FxHashMap<NodeId, Vec<Entity>>,
}

impl GraphEntities {
    pub fn new() -> Self {
        Self {
            map: FxHashMap::default(),
        }
    }

    pub fn get(&self, node: NodeId) -> &[Entity] {
        self.map.get(&node).map_or(&[], |x| &x[..])
    }

    pub fn insert(&mut self, node: NodeId, entity: Entity) {
        let vec = self.map.entry(node).or_insert_with(Vec::new);
        debug_assert!(!vec.contains(&entity), "redundant insert");
        vec.push(entity);
    }

    pub fn remove(&mut self, node: NodeId, entity: Entity) {
        let vec = self.map.get_mut(&node).expect("remove from empty node");
        let pos = vec
            .iter()
            .position(|&e| e == entity)
            .expect("no such entity at this node");
        vec.swap_remove(pos);
        if vec.is_empty() {
            self.map.remove(&node);
        }
    }
}
