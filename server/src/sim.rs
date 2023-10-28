use std::sync::Arc;

use common::{node::ChunkId, GraphEntities};
use fxhash::{FxHashMap, FxHashSet};
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

use crate::postcard_helpers;

pub struct Sim {
    cfg: Arc<SimConfig>,
    rng: SmallRng,
    step: Step,
    entity_ids: FxHashMap<EntityId, Entity>,
    world: hecs::World,
    graph: DualGraph,
    spawns: Vec<Entity>,
    despawns: Vec<EntityId>,
    graph_entities: GraphEntities,
    dirty_nodes: FxHashSet<NodeId>,
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
            graph_entities: GraphEntities::new(),
            dirty_nodes: FxHashSet::default(),
        };

        ensure_nearby(
            &mut result.graph,
            &Position::origin(),
            f64::from(result.cfg.view_distance),
        );
        result
    }

    pub fn save(&mut self, save: &mut save::Save) -> Result<(), save::DbError> {
        fn path_from_origin(graph: &DualGraph, mut node: NodeId) -> Vec<u32> {
            let mut result = Vec::new();
            while let Some(parent) = graph.parent(node) {
                result.push(parent as u32);
                node = graph.neighbor(node, parent).unwrap();
            }
            result.reverse();
            result
        }

        let mut tx = save.write()?;
        let mut writer = tx.get()?;
        for (_, (pos, ch)) in self.world.query::<(&Position, &Character)>().iter() {
            writer.put_character(
                &ch.name,
                &save::Character {
                    path: path_from_origin(&self.graph, pos.node),
                },
            )?;
        }

        let dirty_nodes = self.dirty_nodes.drain().collect::<Vec<_>>();
        for node in dirty_nodes {
            let entities = self.snapshot_node(node);
            writer.put_entity_node(self.graph.hash_of(node), &entities)?;
        }

        drop(writer);
        tx.commit()?;
        Ok(())
    }

    fn snapshot_node(&self, node: NodeId) -> save::EntityNode {
        let mut ids = Vec::new();
        let mut character_transforms = Vec::new();
        let mut character_names = Vec::new();
        let entities = self.graph_entities.get(node);

        for &entity in entities {
            // TODO: Handle entities other than characters
            let mut q = self
                .world
                .query_one::<(&EntityId, &Position, &Character)>(entity)
                .unwrap();
            let Some((id, pos, ch)) = q.get() else {
                continue;
            };
            ids.push(id.to_bits());
            postcard_helpers::serialize(pos.local.as_ref(), &mut character_transforms).unwrap();
            postcard_helpers::serialize(&ch.name, &mut character_names).unwrap();
        }

        save::EntityNode {
            archetypes: vec![save::Archetype {
                entities: ids,
                component_types: vec![
                    save::ComponentType::Position.into(),
                    save::ComponentType::Name.into(),
                ],
                component_data: vec![character_transforms, character_names],
            }],
        }
    }

    pub fn spawn_character(&mut self, hello: ClientHello) -> (EntityId, Entity) {
        let id = self.new_id();
        info!(%id, name = %hello.name, "spawning character");
        let position = Position {
            node: NodeId::ROOT,
            local: math::translate_along(&(na::Vector3::y() * 1.4)),
        };
        let character = Character {
            name: hello.name,
            state: CharacterState {
                orientation: na::one(),
                velocity: na::Vector3::zeros(),
                on_ground: false,
            },
        };
        let initial_input = CharacterInput {
            movement: na::Vector3::zeros(),
            jump: false,
            no_clip: true,
        };
        let entity = self.world.spawn((id, position, character, initial_input));
        self.graph_entities.insert(position.node, entity);
        self.entity_ids.insert(id, entity);
        self.spawns.push(entity);
        self.dirty_nodes.insert(position.node);
        (id, entity)
    }

    pub fn command(
        &mut self,
        entity: Entity,
        command: Command,
    ) -> Result<(), hecs::ComponentError> {
        let mut input = self.world.get::<&mut CharacterInput>(entity)?;
        *input = command.character_input;
        let mut ch = self.world.get::<&mut Character>(entity)?;
        ch.state.orientation = command.orientation;
        Ok(())
    }

    pub fn destroy(&mut self, entity: Entity) {
        let id = *self.world.get::<&EntityId>(entity).unwrap();
        self.entity_ids.remove(&id);
        if let Ok(position) = self.world.get::<&Position>(entity) {
            self.graph_entities.remove(position.node, entity);
        }
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
        for (entity, (position, character, input)) in self
            .world
            .query::<(&mut Position, &mut Character, &CharacterInput)>()
            .iter()
        {
            let prev_node = position.node;
            character_controller::run_character_step(
                &self.cfg,
                &self.graph,
                position,
                &mut character.state.velocity,
                &mut character.state.on_ground,
                input,
                self.cfg.step_interval.as_secs_f32(),
            );
            if prev_node != position.node {
                self.dirty_nodes.insert(prev_node);
                self.graph_entities.remove(prev_node, entity);
                self.graph_entities.insert(position.node, entity);
            }
            self.dirty_nodes.insert(position.node);
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
        let chunk_generation_distance = dodeca::BOUNDING_SPHERE_RADIUS
            + self.cfg.character.character_radius as f64
            + self.cfg.character.speed_cap as f64 * self.cfg.step_interval.as_secs_f64()
            + self.cfg.character.ground_distance_tolerance as f64
            + 0.001;

        // Load all chunks around entities corresponding to clients, which correspond to entities
        // with a "Character" component.
        for (_, (position, _)) in self.world.query::<(&Position, &Character)>().iter() {
            let nodes = nearby_nodes(&self.graph, position, chunk_generation_distance);
            for &(node, _) in &nodes {
                for vertex in dodeca::Vertex::iter() {
                    let chunk = ChunkId::new(node, vertex);
                    if let Chunk::Fresh = self
                        .graph
                        .get_chunk(chunk)
                        .expect("all nodes must be populated before loading their chunks")
                    {
                        if let Some(params) =
                            ChunkParams::new(self.cfg.chunk_size, &self.graph, chunk)
                        {
                            self.graph[chunk] = Chunk::Populated {
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
