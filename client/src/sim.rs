use std::{sync::Arc, time::Duration};

use fxhash::FxHashMap;
use hecs::Entity;
use tracing::{error, trace};

use crate::{graphics::lru_table::SlotId, net, worldgen::NodeState, Config, Net};
use common::{
    dodeca,
    graph::{Graph, NodeId},
    proto::{self, Command, Position},
    world::{Material, SUBDIVISION_FACTOR},
    EntityId, Step,
};

type DualGraph = Graph<NodeState, Cube>;

/// Game state
pub struct Sim {
    cfg: Arc<Config>,
    net: Net,

    // World state
    entity_ids: FxHashMap<EntityId, Entity>,
    world: hecs::World,
    pub graph: DualGraph,
    local_character: Option<EntityId>,
    orientation: na::UnitQuaternion<f32>,
    step: Option<Step>,

    // Input state
    since_input_sent: Duration,
    velocity: na::Vector3<f32>,
}

impl Sim {
    pub fn new(net: Net, cfg: Arc<Config>) -> Self {
        Self {
            cfg,
            net,

            graph: {
                let mut g = Graph::new();
                populate_node(&mut g, NodeId::ROOT);
                for v in dodeca::Vertex::iter() {
                    populate_cube(&mut g, NodeId::ROOT, v);
                }
                g
            },
            entity_ids: FxHashMap::default(),
            world: hecs::World::new(),
            local_character: None,
            orientation: na::one(),
            step: None,

            since_input_sent: Duration::new(0, 0),
            velocity: na::zero(),
        }
    }

    pub fn rotate(&mut self, delta: &na::UnitQuaternion<f32>) {
        self.orientation *= delta;
    }

    pub fn velocity(&mut self, v: na::Vector3<f32>) {
        self.velocity = v;
    }

    pub fn step(&mut self, dt: Duration) {
        while let Ok(msg) = self.net.incoming.try_recv() {
            self.handle_net(msg);
        }

        self.since_input_sent += dt;
        if self.since_input_sent > Duration::from_secs(1) / (self.cfg.input_send_rate as u32) {
            self.send_input();
            self.since_input_sent = Duration::new(0, 0);
        }
    }

    fn handle_net(&mut self, msg: net::Message) {
        use net::Message::*;
        match msg {
            ConnectionLost(e) => {
                error!("connection lost: {}", e);
            }
            Hello(msg) => {
                self.local_character = Some(msg.character);
            }
            Spawns(msg) => self.handle_spawns(msg),
            StateDelta(msg) => {
                self.step = self.step.max(Some(msg.step));
                for &(id, new_pos) in &msg.positions {
                    match self.entity_ids.get(&id) {
                        None => error!(%id, "position update for unknown entity"),
                        Some(&entity) => match self.world.get_mut::<Position>(entity) {
                            Ok(mut pos) => {
                                *pos = new_pos;
                            }
                            Err(e) => error!(%id, "position update for unpositioned entity: {}", e),
                        },
                    }
                }
            }
        }
    }

    fn handle_spawns(&mut self, msg: proto::Spawns) {
        self.step = self.step.max(Some(msg.step));
        let mut builder = hecs::EntityBuilder::new();
        for &(id, ref components) in &msg.spawns {
            trace!(%id, "spawning entity");
            builder.add(id);
            for component in components {
                use common::proto::Component::*;
                match *component {
                    Character(_) => {}
                    Position(x) => {
                        builder.add(x);
                    }
                }
            }
            let entity = self.world.spawn(builder.build());
            if let Some(x) = self.entity_ids.insert(id, entity) {
                let _ = self.world.despawn(x);
                error!(%id, "id collision");
            }
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

    fn send_input(&mut self) {
        if let Some(&entity) = self.local_character.and_then(|id| self.entity_ids.get(&id)) {
            let pos = *self.world.get::<Position>(entity).unwrap();
            // Any failure here will be better handled in ConnectionLost above on the next call
            let _ = self.net.outgoing.send(Command {
                step: self.step.unwrap(),
                node: pos.node,
                orientation: self.orientation,
                velocity: self.orientation * self.velocity,
            });
        }
    }

    pub fn view(&self) -> Position {
        if let Some(&entity) = self.local_character.and_then(|id| self.entity_ids.get(&id)) {
            let mut pos = *self.world.get::<Position>(entity).unwrap();
            pos.local *= self.orientation.to_homogeneous();
            pos
        } else {
            Position {
                node: NodeId::ROOT,
                local: na::Matrix4::identity(),
            }
        }
    }

    fn destroy(&mut self, entity: Entity) {
        let id = *self.world.get::<EntityId>(entity).unwrap();
        self.entity_ids.remove(&id);
        self.world
            .despawn(entity)
            .expect("destroyed nonexistent entity");
    }
}

pub struct Cube {
    pub surface: Option<SlotId>,
    pub voxels: VoxelData,
}

pub enum VoxelData {
    Dense(Box<[Material]>),
    Solid(Material),
}
impl VoxelData {
    pub fn data_mut(&mut self) -> &mut [Material] {
        match self {
            VoxelData::Dense(d) => d,
            VoxelData::Solid(_) => {
                *self = VoxelData::Dense(self.data());
                self.data_mut()
            }
        }
    }
    pub fn data(&self) -> Box<[Material]> {
        match self {
            VoxelData::Dense(d) => Box::clone(d),
            VoxelData::Solid(mat) => {
                (0..(SUBDIVISION_FACTOR + 2).pow(3))
                    .map(|_| *mat)
                    .collect::<Vec<_>>()
                    .into_boxed_slice()
            }
        }
    }
}

fn populate_fresh_nodes(graph: &mut DualGraph) {
    let fresh = graph.fresh().to_vec();
    graph.clear_fresh();
    for &node in &fresh {
        populate_node(graph, node);
    }
    for &node in &fresh {
        for cube in graph.cubes_at(node) {
            populate_cube(graph, node, cube);
        }
    }
    graph.clear_fresh();
}

fn populate_node(graph: &mut DualGraph, node: NodeId) {
    *graph.get_mut(node) = graph
        .parent(node)
        .and_then(|i| {
            let parent_state = (*graph.get(graph.neighbor(node, i)?))?;
            Some(parent_state.child(i))
        })
        .or(Some(NodeState::ROOT));
}

fn populate_cube(graph: &mut DualGraph, node: NodeId, cube: dodeca::Vertex) {
    // find the state of all nodes incident to this cube
    let node_state = graph.get(node).unwrap();
    let node_length = graph.length(node);
    let mut voxels = VoxelData::Solid(Material::Void);
    cube.dual_vertices()
        // this'll give us 7/8 nodes that touch the cube
        .map(|paths| {
            paths.fold((Some(node), node_state), |(acc_id, acc_state), x| {
                let x_id = acc_id.and_then(|i| graph.neighbor(i, x));
                (
                    x_id,
                    x_id.and_then(|i| *graph.get(i))
                        .unwrap_or_else(|| acc_state.child(x)),
                )
            })
        })
        .map(|(node, state)| {
            (
                na::Vector3::from_vec(
                    cube.canonical_sides()
                        .iter()
                        .map(|&s| {
                            node.and_then(|n| graph.neighbor(n, s))
                                .map(|n| if graph.length(n) > node_length { 0 } else { 1 })
                                .unwrap_or(1)
                        })
                        .collect::<Vec<usize>>(),
                ),
                state,
            )
        })
        .for_each(|(subchunk_offset, state)| state.fill_subchunk(&mut voxels, subchunk_offset));
    *graph.get_cube_mut(node, cube) = Some(Cube {
        surface: None,
        voxels,
    });
}
