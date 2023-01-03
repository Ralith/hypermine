use std::sync::Arc;

use fxhash::FxHashMap;
use hecs::Entity;
use rand::rngs::SmallRng;
use rand::{Rng, SeedableRng};
use tracing::{error_span, info, trace};

use common::{
    character_controller, dodeca,
    graph::{Graph, NodeId},
    math,
    node::{populate_fresh_nodes, Chunk, DualGraph},
    proto::{
        Character, CharacterInput, CharacterState, ClientHello, Command, Component, FreshNode,
        Position, Spawns, StateDelta,
    },
    traversal::{ensure_nearby, nearby_nodes},
    worldgen::ChunkParams,
    EntityId, SimConfig, Step,
};

pub struct Sim {
    cfg: Arc<SimConfig>,
    rng: SmallRng,
    step: Step,
    entity_ids: FxHashMap<EntityId, Entity>,
    world: hecs::World,
    graph: DualGraph,
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

        ensure_nearby(
            &mut result.graph,
            &Position::origin(),
            f64::from(result.cfg.view_distance),
        );
        result
    }

    pub fn spawn_character(&mut self, hello: ClientHello) -> (EntityId, Entity) {
        let id = self.new_id();
        info!(%id, name = %hello.name, "spawning character");
        let position = Position {
            node: NodeId::ROOT,
            local: math::translate_along(&(na::Vector3::y() * 1.1)),
        };
        let character = Character {
            name: hello.name,
            state: CharacterState {
                orientation: na::one(),
                velocity: na::Vector3::zeros(),
            },
        };
        let initial_input = CharacterInput {
            movement: na::Vector3::zeros(),
            no_clip: true,
        };
        let entity = self.world.spawn((id, position, character, initial_input));
        self.entity_ids.insert(id, entity);
        self.spawns.push(entity);
        (id, entity)
    }

    pub fn command(
        &mut self,
        entity: Entity,
        command: Command,
    ) -> Result<(), hecs::ComponentError> {
        let mut input = self.world.get::<&mut CharacterInput>(entity)?;
        *input = command.character_input;
        Ok(())
    }

    pub fn destroy(&mut self, entity: Entity) {
        let id = *self.world.get::<&EntityId>(entity).unwrap();
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
        for (_, (position, character, input)) in self
            .world
            .query::<(&mut Position, &mut Character, &CharacterInput)>()
            .iter()
        {
            character_controller::run_character_step(
                &self.cfg,
                &self.graph,
                position,
                &mut character.state.velocity,
                input,
                self.cfg.step_interval.as_secs_f32(),
            );
            ensure_nearby(&mut self.graph, position, f64::from(self.cfg.view_distance));
        }

        // Capture state changes for broadcast to clients
        let mut spawns = Vec::with_capacity(self.spawns.len());
        for entity in self.spawns.drain(..) {
            let id = *self.world.get::<&EntityId>(entity).unwrap();
            spawns.push((id, dump_entity(&self.world, entity)));
        }
        if !self.graph.fresh().is_empty() {
            trace!(count = self.graph.fresh().len(), "broadcasting fresh nodes");
        }
        let spawns = Spawns {
            step: self.step,
            spawns,
            despawns: std::mem::take(&mut self.despawns),
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
        populate_fresh_nodes(&mut self.graph);

        // We want to load all chunks that a player can interact with in a single step, so chunk_generation_distance
        // is set up to cover that distance.
        // TODO: Use actual max speed instead of max ground speed.
        // TODO: Account for the radius of the player's collision sphere
        let chunk_generation_distance = dodeca::BOUNDING_SPHERE_RADIUS
            + self.cfg.max_ground_speed as f64 * self.cfg.step_interval.as_secs_f64()
            + 0.001;

        // Load all chunks around entities corresponding to clients, which correspond to entities
        // with a "Character" component.
        for (_, (position, _)) in self.world.query::<(&Position, &Character)>().iter() {
            let nodes = nearby_nodes(&self.graph, position, chunk_generation_distance);
            for &(node, _) in &nodes {
                for chunk in dodeca::Vertex::iter() {
                    if let Chunk::Fresh = self
                        .graph
                        .get(node)
                        .as_ref()
                        .expect("all nodes must be populated before loading their chunks")
                        .chunks[chunk]
                    {
                        if let Some(params) =
                            ChunkParams::new(self.cfg.chunk_size, &self.graph, node, chunk)
                        {
                            self.graph.get_mut(node).as_mut().unwrap().chunks[chunk] =
                                Chunk::Populated {
                                    voxels: params.generate_voxels(),
                                    surface: None,
                                };
                        }
                    }
                }
            }
        }

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
            character_states: self
                .world
                .query::<(&EntityId, &Character)>()
                .iter()
                .map(|(_, (&id, ch))| (id, ch.state.clone()))
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
    if let Ok(x) = world.get::<&Position>(entity) {
        components.push(Component::Position(*x));
    }
    if let Ok(x) = world.get::<&Character>(entity) {
        components.push(Component::Character((*x).clone()));
    }
    components
}
