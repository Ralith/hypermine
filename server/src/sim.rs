use std::sync::Arc;

use anyhow::Context;
use common::dodeca::{Side, Vertex};
use common::math::MIsometry;
use common::node::VoxelData;
use common::proto::{BlockUpdate, Inventory, SerializedVoxelData};
use common::world::Material;
use common::{node::ChunkId, GraphEntities};
use fxhash::{FxHashMap, FxHashSet};
use hecs::{DynamicBundle, Entity, EntityBuilder};
use rand::rngs::SmallRng;
use rand::{Rng, SeedableRng};
use save::ComponentType;
use serde::{Deserialize, Serialize};
use tracing::{error, error_span, info, trace};

use common::{
    character_controller, dodeca,
    graph::{Graph, NodeId},
    node::{populate_fresh_nodes, Chunk},
    proto::{
        Character, CharacterInput, CharacterState, ClientHello, Command, Component, FreshNode,
        Position, Spawns, StateDelta,
    },
    traversal::{ensure_nearby, nearby_nodes},
    worldgen::ChunkParams,
    EntityId, SimConfig, Step,
};

use crate::postcard_helpers::{self, SaveEntity};

pub struct Sim {
    cfg: Arc<SimConfig>,
    rng: SmallRng,
    step: Step,
    entity_ids: FxHashMap<EntityId, Entity>,
    world: hecs::World,
    graph: Graph,
    /// Voxel data that has been fetched from a savefile but not yet introduced to the graph
    preloaded_voxel_data: FxHashMap<ChunkId, VoxelData>,
    accumulated_changes: AccumulatedChanges,
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
            rng: SmallRng::from_os_rng(),
            step: 0,
            entity_ids: FxHashMap::default(),
            world: hecs::World::new(),
            graph: Graph::new(cfg.chunk_size),
            preloaded_voxel_data: FxHashMap::default(),
            accumulated_changes: AccumulatedChanges::default(),
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
            .load_all_entities(save)
            .expect("save file must be of a valid format");
        // Loading entities can cause graph nodes to also be created, so we should populate them before returning.
        result.populate_fresh_graph_nodes();
        // As no players have logged in yet, and `snapshot` may be called before the first call of `step`,
        // make sure that `accumulated_changes` is empty to avoid accidental double-spawns of anything.
        result.accumulated_changes = AccumulatedChanges::default();
        result
    }

    pub fn save(&mut self, save: &mut save::Save) -> Result<(), save::DbError> {
        fn path_from_origin(graph: &Graph, mut node: NodeId) -> Vec<u8> {
            let mut result = Vec::new();
            while let Some(parent) = graph.parent(node) {
                result.push(parent as u8);
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

    /// Loads all entities from the given save file. Note that this must be called before any players
    /// log in, as `accumulated_changes` will not properly reflect the entities that were loaded in.
    /// It is also important to call `populate_fresh_graph_nodes` after calling this function to keep
    /// `Sim` in a consistent state, as this function can expand the graph without populating the graph nodes.
    fn load_all_entities(&mut self, save: &save::Save) -> anyhow::Result<()> {
        let mut read = save.read()?;
        for node_hash in read.get_all_entity_node_ids()? {
            let Some(entity_node) = read.get_entity_node(node_hash)? else {
                continue;
            };
            let node_id = self.graph.from_hash(node_hash);
            for entity_bytes in entity_node.entities {
                let save_entity: SaveEntity = postcard::from_bytes(&entity_bytes)?;
                self.load_entity(&mut read, node_id, save_entity)?;
            }
        }
        Ok(())
    }

    fn load_entity(
        &mut self,
        read: &mut save::Reader,
        node: NodeId,
        save_entity: SaveEntity,
    ) -> anyhow::Result<()> {
        let entity_id = EntityId::from_bits(u64::from_le_bytes(save_entity.entity));
        let mut entity_builder = EntityBuilder::new();
        entity_builder.add(entity_id);
        entity_builder.add(node);
        for (component_type, component_bytes) in save_entity.components {
            self.load_component(
                read,
                &mut entity_builder,
                node,
                ComponentType::try_from(component_type as i32).unwrap(),
                component_bytes,
            )?;
        }
        let entity = self.world.spawn(entity_builder.build());
        self.graph_entities.insert(node, entity);
        self.entity_ids.insert(entity_id, entity);
        Ok(())
    }

    fn load_component(
        &mut self,
        read: &mut save::Reader,
        entity_builder: &mut EntityBuilder,
        node: NodeId,
        component_type: ComponentType,
        component_bytes: Vec<u8>,
    ) -> anyhow::Result<()> {
        match component_type {
            ComponentType::Position => {
                let column_slice: [f32; 16] = postcard::from_bytes(&component_bytes)?;
                entity_builder.add(Position {
                    node,
                    local: MIsometry::from_column_slice_unchecked(&column_slice),
                });
            }
            ComponentType::Name => {
                let name = String::from_utf8(component_bytes)?;
                // Ensure that every node occupied by a character is generated.
                let Some(character) = read.get_character(&name)? else {
                    // Skip loading named entities that lack path information.
                    error!("Entity {} will not be loaded because their node path information is missing.", name);
                    return Ok(());
                };
                let mut current_node = NodeId::ROOT;
                for side in character
                    .path
                    .into_iter()
                    .map(|side| Side::VALUES[side as usize])
                {
                    current_node = self.graph.ensure_neighbor(current_node, side);
                }
                if current_node != node {
                    // Skip loading named entities that are in the wrong place. This can happen
                    // when there are multiple entities with the same name, which has been possible
                    // in previous versions of Hypermine.
                    error!("Entity {} will not be loaded because their node path information is incorrect.", name);
                    return Ok(());
                }
                // Prepare all relevant components that are needed to support ComponentType::Name
                entity_builder.add(InactiveCharacter(Character {
                    name,
                    state: CharacterState {
                        velocity: na::Vector3::zeros(),
                        on_ground: false,
                        orientation: na::UnitQuaternion::identity(),
                    },
                }));
            }
            ComponentType::Material => {
                let material: u16 =
                    u16::from_le_bytes(component_bytes.try_into().map_err(|_| {
                        anyhow::anyhow!("Expected Material component in save file to be 2 bytes")
                    })?);
                entity_builder.add(Material::try_from(material)?);
            }
            ComponentType::Inventory => {
                let mut contents = vec![];
                for chunk in component_bytes.chunks(8) {
                    contents.push(EntityId::from_bits(u64::from_le_bytes(
                        chunk.try_into().unwrap(),
                    )));
                }
                entity_builder.add(Inventory { contents });
            }
        }
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
                    postcard::to_stdvec(&pos.local.as_ref()).unwrap(),
                ));
            }
            if let Some(ch) = entity.get::<&Character>().or_else(|| {
                entity
                    .get::<&InactiveCharacter>()
                    .map(|ich| hecs::Ref::map(ich, |ich| &ich.0)) // Extract Ref<Character> from Ref<InactiveCharacter>
            }) {
                components.push((ComponentType::Name as u64, ch.name.as_bytes().into()));
            }
            if let Some(material) = entity.get::<&Material>() {
                components.push((
                    ComponentType::Material as u64,
                    (*material as u16).to_le_bytes().into(),
                ));
            }
            if let Some(inventory) = entity.get::<&Inventory>() {
                let mut serialized_inventory_contents = vec![];
                for entity_id in &inventory.contents {
                    serialized_inventory_contents
                        .extend_from_slice(&entity_id.to_bits().to_le_bytes());
                }
                components.push((
                    ComponentType::Inventory as u64,
                    serialized_inventory_contents,
                ));
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

    /// Activates or spawns a character with a given name, or returns None if there is already an active
    /// character with that name
    pub fn activate_or_spawn_character(
        &mut self,
        hello: &ClientHello,
    ) -> Option<(EntityId, Entity)> {
        // Check for conflicting characters
        if self
            .world
            .query::<&Character>()
            .iter()
            .any(|(_, character)| character.name == hello.name)
        {
            return None;
        }

        // Check for matching characters
        let matching_character = self
            .world
            .query::<(&EntityId, &InactiveCharacter)>()
            .iter()
            .find(|(_, (_, inactive_character))| inactive_character.0.name == hello.name)
            .map(|(entity, (entity_id, _))| (*entity_id, entity));
        if let Some((entity_id, entity)) = matching_character {
            info!(id = %entity_id, name = %hello.name, "activating character");
            let inactive_character = self.world.remove_one::<InactiveCharacter>(entity).unwrap();
            self.world
                .insert(entity, (inactive_character.0, CharacterInput::default()))
                .unwrap();
            self.accumulated_changes.spawns.push(entity);
            return Some((entity_id, entity));
        }

        // Spawn entirely new character
        let position = Position {
            node: NodeId::ROOT,
            local: MIsometry::translation_along(&(na::Vector3::y() * 1.4)),
        };
        let character = Character {
            name: hello.name.clone(),
            state: CharacterState {
                orientation: na::one(),
                velocity: na::Vector3::zeros(),
                on_ground: false,
            },
        };
        let inventory = Inventory { contents: vec![] };
        let initial_input = CharacterInput::default();
        Some(self.spawn((position.node, position, character, inventory, initial_input)))
    }

    pub fn deactivate_character(&mut self, entity: Entity) {
        let entity_id = *self.world.get::<&EntityId>(entity).unwrap();
        let (character, _) = self
            .world
            .remove::<(Character, CharacterInput)>(entity)
            .unwrap();
        self.world
            .insert_one(entity, InactiveCharacter(character))
            .unwrap();
        self.accumulated_changes.despawns.push(entity_id);
    }

    fn spawn(&mut self, bundle: impl DynamicBundle) -> (EntityId, Entity) {
        let id = self.new_id();
        let mut entity_builder = EntityBuilder::new();
        entity_builder.add(id);
        entity_builder.add_bundle(bundle);
        let entity = self.world.spawn(entity_builder.build());

        if let Ok(node) = self.world.get::<&NodeId>(entity) {
            self.graph_entities.insert(*node, entity);
            self.dirty_nodes.insert(*node);
        }

        if let Ok(character) = self.world.get::<&Character>(entity) {
            info!(%id, name = %character.name, "spawning character");
        }

        self.entity_ids.insert(id, entity);

        if !self.world.satisfies::<&InactiveCharacter>(entity).unwrap() {
            self.accumulated_changes.spawns.push(entity);
        }

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
        if let Ok(node) = self.world.get::<&NodeId>(entity) {
            self.graph_entities.remove(*node, entity);
        }
        if !self.world.satisfies::<&InactiveCharacter>(entity).unwrap() {
            self.accumulated_changes.despawns.push(id);
        }
        self.world.despawn(entity).unwrap();
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
            inventory_additions: Vec::new(),
            inventory_removals: Vec::new(),
        };
        for (entity, &id) in &mut self
            .world
            .query::<hecs::Without<&EntityId, &InactiveCharacter>>()
        {
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

    pub fn step(&mut self) -> (Option<Spawns>, StateDelta) {
        let span = error_span!("step", step = self.step);
        let _guard = span.enter();

        // Extend graph structure
        for (_, (position, _)) in self.world.query::<(&mut Position, &mut Character)>().iter() {
            ensure_nearby(&mut self.graph, position, self.cfg.view_distance);
        }

        self.populate_fresh_graph_nodes();

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

        let mut pending_block_updates: Vec<(Entity, BlockUpdate)> = vec![];

        // Simulate
        for (entity, (node, position, character, input)) in self
            .world
            .query::<(&NodeId, &mut Position, &mut Character, &CharacterInput)>()
            .iter()
        {
            character_controller::run_character_step(
                &self.cfg,
                &self.graph,
                position,
                &mut character.state.velocity,
                &mut character.state.on_ground,
                input,
                self.cfg.step_interval.as_secs_f32(),
            );
            if let Some(block_update) = input.block_update.clone() {
                pending_block_updates.push((entity, block_update));
            }
            self.dirty_nodes.insert(*node);
        }

        for (entity, block_update) in pending_block_updates {
            let id = *self.world.get::<&EntityId>(entity).unwrap();
            self.attempt_block_update(id, block_update);
        }

        self.update_entity_node_ids();

        let spawns = std::mem::take(&mut self.accumulated_changes).into_spawns(
            self.step,
            &self.world,
            &self.graph,
        );

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

    /// Ensure that the NodeId component of every entity is set to what it should be to ensure consistency. Any entity
    /// with a position should have a NodeId that matches that position, and all entities with inventories should propagate
    /// their NodeId to their inventory items.
    fn update_entity_node_ids(&mut self) {
        // Helper function for properly changing the NodeId of a given node without leaving any
        // of the supporting structures out of date.
        let mut update_node_id = |entity: Entity, node_id: &mut NodeId, new_node_id: NodeId| {
            if *node_id != new_node_id {
                self.dirty_nodes.insert(*node_id);
                self.graph_entities.remove(*node_id, entity);

                *node_id = new_node_id;
                self.dirty_nodes.insert(*node_id);
                self.graph_entities.insert(*node_id, entity);
            }
        };

        // Synchronize NodeId and Position
        for (entity, (node_id, position)) in self.world.query::<(&mut NodeId, &Position)>().iter() {
            update_node_id(entity, node_id, position.node);
        }

        // Synchronize NodeId for all inventory items.
        // TODO: Note that the order in which inventory items are updated is arbitrary, so
        // if inventory items can themselves have inventories, their respective NodeIds
        // may be out of date by a few steps, which could cause bugs. This can be solved with
        // a more complete entity hierarchy system
        for (_, (&inventory_node_id, inventory)) in
            self.world.query::<(&NodeId, &Inventory)>().iter()
        {
            for inventory_entity_id in &inventory.contents {
                let inventory_entity = *self.entity_ids.get(inventory_entity_id).unwrap();

                let mut inventory_entity_node_id =
                    self.world.get::<&mut NodeId>(inventory_entity).unwrap();

                update_node_id(
                    inventory_entity,
                    &mut inventory_entity_node_id,
                    inventory_node_id,
                );
            }
        }
    }

    /// Should be called after any set of changes is made to the graph to ensure that the server
    /// does not have any partially-initialized graph nodes.
    fn populate_fresh_graph_nodes(&mut self) {
        let fresh_nodes = self.graph.fresh().to_vec();
        populate_fresh_nodes(&mut self.graph);

        self.accumulated_changes
            .fresh_nodes
            .extend_from_slice(&fresh_nodes);
        for fresh_node in fresh_nodes.iter().copied() {
            for vertex in Vertex::iter() {
                let chunk = ChunkId::new(fresh_node, vertex);
                if let Some(voxel_data) = self.preloaded_voxel_data.remove(&chunk) {
                    self.accumulated_changes
                        .fresh_voxel_data
                        .push((chunk, voxel_data.serialize(self.cfg.chunk_size)));
                    self.modified_chunks.insert(chunk);
                    self.graph.populate_chunk(chunk, voxel_data)
                }
            }
        }
    }

    fn new_id(&mut self) -> EntityId {
        loop {
            let id = self.rng.random();
            if !self.entity_ids.contains_key(&id) {
                return id;
            }
        }
    }

    /// Add the given entity to the given inventory
    fn add_to_inventory(&mut self, inventory_id: EntityId, entity_id: EntityId) {
        let mut inventory = self
            .world
            .get::<&mut Inventory>(*self.entity_ids.get(&inventory_id).unwrap())
            .unwrap();
        inventory.contents.push(entity_id);
        self.accumulated_changes
            .inventory_additions
            .push((inventory_id, entity_id));
    }

    /// Remove the given entity from the given inventory. Note that this does not destroy the entity.
    /// Returns whether the item was in the inventory to begin with.
    fn remove_from_inventory(&mut self, inventory_id: EntityId, entity_id: EntityId) -> bool {
        let mut inventory = self
            .world
            .get::<&mut Inventory>(*self.entity_ids.get(&inventory_id).unwrap())
            .unwrap();
        let Some(position) = inventory.contents.iter().position(|&e| e == entity_id) else {
            return false;
        };
        inventory.contents.remove(position);
        self.accumulated_changes
            .inventory_removals
            .push((inventory_id, entity_id));
        true
    }

    /// Executes the requested block update if the subject is able to do so and
    /// leaves the state of the world unchanged otherwise
    fn attempt_block_update(&mut self, subject: EntityId, block_update: BlockUpdate) {
        let subject_node = *self
            .world
            .get::<&NodeId>(*self.entity_ids.get(&subject).unwrap())
            .unwrap();
        let Some(old_material) = self
            .graph
            .get_material(block_update.chunk_id, block_update.coords)
        else {
            tracing::warn!("Block update received from ungenerated chunk");
            return;
        };
        if self.cfg.gameplay_enabled {
            if block_update.new_material != Material::Void {
                let Some(consumed_entity_id) = block_update.consumed_entity else {
                    tracing::warn!("Tried to place block without consuming any entities");
                    return;
                };
                let Some(&consumed_entity) = self.entity_ids.get(&consumed_entity_id) else {
                    tracing::warn!("Tried to consume an unknown entity ID");
                    return;
                };
                if !self
                    .world
                    .get::<&Material>(consumed_entity)
                    .is_ok_and(|m| *m == block_update.new_material)
                {
                    tracing::warn!("Tried to consume wrong material");
                    return;
                }
                if !self.remove_from_inventory(subject, consumed_entity_id) {
                    tracing::warn!("Tried to consume entity not in player inventory");
                    return;
                }
                self.destroy(consumed_entity);
            }
            if old_material != Material::Void {
                let (produced_entity, _) = self.spawn((subject_node, old_material));
                self.add_to_inventory(subject, produced_entity);
            }
        }
        assert!(self.graph.update_block(&block_update));
        self.modified_chunks.insert(block_update.chunk_id);
        self.dirty_voxel_nodes.insert(block_update.chunk_id.node);
        self.accumulated_changes.block_updates.push(block_update);
    }
}

/// Collect all information about a particular entity for transmission to clients.
fn dump_entity(world: &hecs::World, entity: Entity) -> Vec<Component> {
    assert!(
        !world.satisfies::<&InactiveCharacter>(entity).unwrap(),
        "Inactive characters should not be sent to clients"
    );
    let mut components = Vec::new();
    if let Ok(x) = world.get::<&Position>(entity) {
        components.push(Component::Position(*x));
    }
    if let Ok(x) = world.get::<&Character>(entity) {
        components.push(Component::Character((*x).clone()));
    }
    if let Ok(x) = world.get::<&Inventory>(entity) {
        components.push(Component::Inventory((*x).clone()));
    }
    if let Ok(x) = world.get::<&Material>(entity) {
        components.push(Component::Material(*x));
    }
    components
}

#[derive(Debug, Serialize, Deserialize, Clone)]
struct InactiveCharacter(pub Character);

/// Stores changes that the server has canonically done but hasn't yet broadcast to clients
#[derive(Default)]
struct AccumulatedChanges {
    /// Entities that have been spawned since the last broadcast
    spawns: Vec<Entity>,

    /// Entities that have been despawned since the last broadcast
    despawns: Vec<EntityId>,

    /// Block updates that have been applied to the world since the last broadcast
    block_updates: Vec<BlockUpdate>,

    /// Entities that have been added to an inventory since the last broadcast, where `(a, b)`` represents
    /// entity `b`` being added to inventory `a``
    inventory_additions: Vec<(EntityId, EntityId)>,

    /// Entities that have been removed from an inventory since the last broadcast, where `(a, b)`` represents
    /// entity `b`` being removed from inventory `a``
    inventory_removals: Vec<(EntityId, EntityId)>,

    /// Nodes that have been added to the graph since the last broadcast
    fresh_nodes: Vec<NodeId>,

    /// Voxel data from `fresh_nodes` that needs to be broadcast to clients due to not exactly matching what
    /// world generation would return. This is needed to support `preloaded_voxel_data`
    fresh_voxel_data: Vec<(ChunkId, SerializedVoxelData)>,
}

impl AccumulatedChanges {
    fn is_empty(&self) -> bool {
        self.spawns.is_empty()
            && self.despawns.is_empty()
            && self.block_updates.is_empty()
            && self.inventory_additions.is_empty()
            && self.inventory_removals.is_empty()
            && self.fresh_nodes.is_empty()
            && self.fresh_voxel_data.is_empty()
    }

    /// Convert state changes for broadcast to clients
    fn into_spawns(self, step: Step, world: &hecs::World, graph: &Graph) -> Option<Spawns> {
        if self.is_empty() {
            return None;
        }

        let mut spawns = Vec::with_capacity(self.spawns.len());
        for entity in self.spawns {
            let id = *world.get::<&EntityId>(entity).unwrap();
            spawns.push((id, dump_entity(world, entity)));
        }

        if !self.fresh_nodes.is_empty() {
            trace!(count = self.fresh_nodes.len(), "broadcasting fresh nodes");
        }

        Some(Spawns {
            step,
            spawns,
            despawns: self.despawns,
            nodes: self
                .fresh_nodes
                .iter()
                .filter_map(|&id| {
                    let side = graph.parent(id)?;
                    Some(FreshNode {
                        side,
                        parent: graph.neighbor(id, side).unwrap(),
                    })
                })
                .collect(),
            block_updates: self.block_updates,
            voxel_data: self.fresh_voxel_data,
            inventory_additions: self.inventory_additions,
            inventory_removals: self.inventory_removals,
        })
    }
}
