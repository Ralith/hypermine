use std::sync::Arc;

use anyhow::Context;
use common::dodeca::Vertex;
use common::node::VoxelData;
use common::proto::{BlockUpdate, SerializedVoxelData};
use common::{node::ChunkId, GraphEntities};
use fxhash::{FxHashMap, FxHashSet};
use hecs::Entity;
use save::ComponentType;
use tracing::{error, error_span, info, trace};

use common::{
    character_controller, dodeca,
    graph::{Graph, NodeId},
    id_generator::IDGenerator,
    math,
    node::{populate_fresh_nodes, Chunk},
    proto::{
        Character, CharacterInput, CharacterState, ClientHello, Command, Component, FreshNode,
        Position, Spawns, StateDelta,
    },
    ticker::{Ticker, TickerEntity},
    traversal::{ensure_nearby, nearby_nodes},
    worldgen::ChunkParams,
    EntityId, SimConfig, Step,
};

use crate::postcard_helpers::{self, SaveEntity};

pub struct Sim {
    cfg: Arc<SimConfig>,
    step: Step,
    id_generator: IDGenerator,
    world: hecs::World,
    graph: Graph,
    /// Voxel data that has been fetched from a savefile but not yet introduced to the graph
    preloaded_voxel_data: FxHashMap<ChunkId, VoxelData>,
    spawns: Vec<Entity>,
    pending_ticker_spawns: Vec<(Position, TickerEntity)>,
    despawns: Vec<EntityId>,
    graph_entities: GraphEntities,
    /// All nodes that have entity-related information yet to be saved
    dirty_nodes: FxHashSet<NodeId>,
    /// All nodes that have voxel-related information yet to be saved
    dirty_voxel_nodes: FxHashSet<NodeId>,
    /// All chunks that have ever had any block updates applied to them and can no longer be regenerated with worldgen
    modified_chunks: FxHashSet<ChunkId>,
}

impl Sim {
    pub fn new(cfg: Arc<SimConfig>, save: &save::Save) -> Self {
        let mut result = Self {
            step: 0,
            id_generator: IDGenerator::new(),
            world: hecs::World::new(),
            graph: Graph::new(cfg.chunk_size),
            preloaded_voxel_data: FxHashMap::default(),
            spawns: Vec::new(),
            pending_ticker_spawns: Vec::new(),
            despawns: Vec::new(),
            graph_entities: GraphEntities::new(),
            dirty_nodes: FxHashSet::default(),
            dirty_voxel_nodes: FxHashSet::default(),
            modified_chunks: FxHashSet::default(),
            cfg,
        };

        result
            .load_all_voxels(save)
            .expect("save file must be of a valid format");
        result
    }

    pub fn save(&mut self, save: &mut save::Save) -> Result<(), save::DbError> {
        fn path_from_origin(graph: &Graph, mut node: NodeId) -> Vec<u32> {
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
        let dirty_voxel_nodes = self.dirty_voxel_nodes.drain().collect::<Vec<_>>();
        for node in dirty_nodes {
            let entities = self.snapshot_node(node);
            writer.put_entity_node(self.graph.hash_of(node), &entities)?;
        }
        for node in dirty_voxel_nodes {
            let voxels = self.snapshot_voxel_node(node);
            writer.put_voxel_node(self.graph.hash_of(node), &voxels)?;
        }

        drop(writer);
        tx.commit()?;
        Ok(())
    }

    fn load_all_voxels(&mut self, save: &save::Save) -> anyhow::Result<()> {
        let mut read = save.read()?;
        for node_hash in read.get_all_voxel_node_ids()? {
            let Some(voxel_node) = read.get_voxel_node(node_hash)? else {
                continue;
            };
            for chunk in voxel_node.chunks {
                let voxels = SerializedVoxelData {
                    inner: chunk.voxels,
                };
                let vertex = Vertex::iter()
                    .nth(chunk.vertex as usize)
                    .context("deserializing vertex ID")?;
                self.preloaded_voxel_data.insert(
                    ChunkId::new(self.graph.from_hash(node_hash), vertex),
                    VoxelData::deserialize(&voxels, self.cfg.chunk_size)
                        .context("deserializing voxel data")?,
                );
            }
        }
        Ok(())
    }

    fn snapshot_node(&self, node: NodeId) -> save::EntityNode {
        let mut entities = Vec::new();
        for &entity in self.graph_entities.get(node) {
            let Ok(entity) = self.world.entity(entity) else {
                error!("stale graph entity {:?}", entity);
                continue;
            };
            let Some(id) = entity.get::<&EntityId>() else {
                continue;
            };
            let mut components = Vec::new();
            if let Some(pos) = entity.get::<&Position>() {
                components.push((
                    ComponentType::Position as u64,
                    postcard::to_stdvec(pos.local.as_ref()).unwrap(),
                ));
            }
            if let Some(ch) = entity.get::<&Character>() {
                components.push((ComponentType::Name as u64, ch.name.as_bytes().into()));
            }
            let mut repr = Vec::new();
            postcard_helpers::serialize(
                &SaveEntity {
                    entity: id.to_bits().to_le_bytes(),
                    components,
                },
                &mut repr,
            )
            .unwrap();
            entities.push(repr);
        }

        save::EntityNode { entities }
    }

    fn snapshot_voxel_node(&self, node: NodeId) -> save::VoxelNode {
        let mut chunks = vec![];
        let node_data = self.graph.get(node).as_ref().unwrap();
        for vertex in Vertex::iter() {
            if !self.modified_chunks.contains(&ChunkId::new(node, vertex)) {
                continue;
            }
            let Chunk::Populated { ref voxels, .. } = node_data.chunks[vertex] else {
                panic!("Unknown chunk listed as modified");
            };
            chunks.push(save::Chunk {
                vertex: vertex as u32,
                voxels: voxels.serialize(self.cfg.chunk_size).inner,
            })
        }
        save::VoxelNode { chunks }
    }

    pub fn spawn_character(&mut self, hello: ClientHello) -> (EntityId, Entity) {
        let id = self.id_generator.new_id();
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
            debug_spawn_blinker: false,
            block_update: None,
        };
        let entity = self.world.spawn((id, position, character, initial_input));
        self.graph_entities.insert(position.node, entity);
        self.id_generator.entity_ids.insert(id, entity);
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
        self.id_generator.entity_ids.remove(&id);
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
            block_updates: Vec::new(),
            voxel_data: Vec::new(),
        };
        for (entity, &id) in &mut self.world.query::<&EntityId>() {
            spawns.spawns.push((id, dump_entity(&self.world, entity)));
        }
        for &chunk_id in self.modified_chunks.iter() {
            let voxels =
                match self.graph.get(chunk_id.node).as_ref().unwrap().chunks[chunk_id.vertex] {
                    Chunk::Populated { ref voxels, .. } => voxels,
                    _ => panic!("ungenerated chunk is marked as modified"),
                };

            spawns
                .voxel_data
                .push((chunk_id, voxels.serialize(self.cfg.chunk_size)));
        }
        spawns
    }

    pub fn step(&mut self) -> (Spawns, StateDelta) {
        let span = error_span!("step", step = self.step);
        let _guard = span.enter();

        {
            let mut ticker_query = self.world.query::<&mut TickerEntity>();
            let ticker_iter = ticker_query.iter();
            for (_entity, ticker_entity) in ticker_iter {
                ticker_entity.ticker.tick(self.step);
            }
        }

        let mut pending_block_updates: Vec<BlockUpdate> = vec![];


        // Extend graph structure
        for (_, (position, _)) in self.world.query::<(&mut Position, &mut Character)>().iter() {
            ensure_nearby(&mut self.graph, position, self.cfg.view_distance);
        }

        let fresh_nodes = self.graph.fresh().to_vec();
        populate_fresh_nodes(&mut self.graph);

        let mut fresh_voxel_data = vec![];
        for fresh_node in fresh_nodes.iter().cloned() {
            for vertex in Vertex::iter() {
                let chunk = ChunkId::new(fresh_node, vertex);
                if let Some(voxel_data) = self.preloaded_voxel_data.remove(&chunk) {
                    fresh_voxel_data.push((chunk, voxel_data.serialize(self.cfg.chunk_size)));
                    self.modified_chunks.insert(chunk);
                    self.graph.populate_chunk(chunk, voxel_data)
                }
            }
        }

        // We want to load all chunks that a player can interact with in a single step, so chunk_generation_distance
        // is set up to cover that distance.
        let chunk_generation_distance = dodeca::BOUNDING_SPHERE_RADIUS
            + self.cfg.character.character_radius
            + self.cfg.character.speed_cap * self.cfg.step_interval.as_secs_f32()
            + self.cfg.character.ground_distance_tolerance
            + self.cfg.character.block_reach
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
                            self.graph.populate_chunk(chunk, params.generate_voxels());
                        }
                    }
                }
            }
        }

        // Simulate
        for (entity, (position, character, input)) in self
            .world
            .query::<(&mut Position, &mut Character, &CharacterInput)>()
            .iter()
        {
            let prev_node = position.node;
            if input.debug_spawn_blinker {
                let ticker: TickerEntity = TickerEntity {
                    last_ticked: 0,
                    ticker: Ticker::new(),
                };
                self.pending_ticker_spawns.push((*position, ticker));
            }
            character_controller::run_character_step(
                &self.cfg,
                &self.graph,
                position,
                &mut character.state.velocity,
                &mut character.state.on_ground,
                input,
                self.cfg.step_interval.as_secs_f32(),
            );
            pending_block_updates.extend(input.block_update.iter().cloned());
            if prev_node != position.node {
                self.dirty_nodes.insert(prev_node);
                self.graph_entities.remove(prev_node, entity);
                self.graph_entities.insert(position.node, entity);
            }
            self.dirty_nodes.insert(position.node);
        }

        let pending_ticker_spawns = std::mem::take(&mut self.pending_ticker_spawns);
        for (position, ticker) in pending_ticker_spawns {
            let id = self.id_generator.new_id();
            let entity = self.world.spawn((id, position, ticker));
            self.graph_entities.insert(position.node, entity);
            self.id_generator.entity_ids.insert(id, entity);
            self.spawns.push(entity);
            self.dirty_nodes.insert(position.node);
        }

        let mut accepted_block_updates: Vec<BlockUpdate> = vec![];

        for block_update in pending_block_updates.into_iter() {
            if !self.graph.update_block(&block_update) {
                tracing::warn!("Block update received from ungenerated chunk");
            }
            self.modified_chunks.insert(block_update.chunk_id);
            self.dirty_voxel_nodes.insert(block_update.chunk_id.node);
            accepted_block_updates.push(block_update);
        }

        // Capture state changes for broadcast to clients
        let mut spawns = Vec::with_capacity(self.spawns.len());
        for entity in self.spawns.drain(..) {
            let id = *self.world.get::<&EntityId>(entity).unwrap();
            spawns.push((id, dump_entity(&self.world, entity)));
        }

        if !fresh_nodes.is_empty() {
            trace!(count = self.graph.fresh().len(), "broadcasting fresh nodes");
        }

        let spawns = Spawns {
            step: self.step,
            spawns,
            despawns: std::mem::take(&mut self.despawns),
            nodes: fresh_nodes
                .iter()
                .filter_map(|&id| {
                    let side = self.graph.parent(id)?;
                    Some(FreshNode {
                        side,
                        parent: self.graph.neighbor(id, side).unwrap(),
                    })
                })
                .collect(),
            block_updates: accepted_block_updates,
            voxel_data: fresh_voxel_data,
        };

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
