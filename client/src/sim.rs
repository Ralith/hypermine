use std::time::Duration;

use tracing::{debug, error};

use crate::{graphics::lru_table::SlotId, Net};
use common::{
    dodeca,
    graph::{Graph, NodeId},
    math,
    world::{Material, SUBDIVISION_FACTOR},
};

/// Game state
pub struct Sim {
    view_reference: NodeId,
    view: na::Matrix4<f64>,
    velocity: na::Vector3<f64>,
    net: Net,
    pub graph: Graph<bool, Node>,
}

impl Sim {
    pub fn new(net: Net) -> Self {
        let mut result = Self {
            view_reference: NodeId::ROOT,
            view: na::Matrix4::identity(),
            velocity: na::zero(),
            net,
            graph: Graph::new(),
        };

        result.populate_node(NodeId::ROOT);
        result.graph.ensure_nearby(NodeId::ROOT, 3);
        result.populate_fresh_nodes();

        result
    }

    pub fn rotate(&mut self, delta: na::Vector3<f64>) {
        self.view = self.view
            * na::Matrix4::from_axis_angle(&na::Vector3::y_axis(), -delta.x)
            * na::Matrix4::from_axis_angle(&na::Vector3::x_axis(), -delta.y)
            * na::Matrix4::from_axis_angle(&na::Vector3::z_axis(), -delta.z);
    }

    pub fn velocity(&mut self, v: na::Vector3<f64>) {
        self.velocity = v;
    }

    pub fn step(&mut self, dt: Duration) {
        while let Ok(msg) = self.net.incoming.try_recv() {
            use crate::net::Message::*;
            match msg {
                ConnectionLost(e) => {
                    error!("connection lost: {}", e);
                }
                _ => {
                    debug!("unimplemented: {:?}", msg);
                }
            }
        }

        let (dir, rate) = na::Unit::new_and_get(na::convert::<_, na::Vector3<f64>>(self.velocity));
        let distance = rate * dt.as_secs_f64() * 0.5;
        let view = math::renormalize_isometry(&self.view) * math::translate_along(&dir, distance);
        let (reference, view) = self.graph.normalize_transform(self.view_reference, view);
        if reference != self.view_reference {
            debug!("moved to node {:?}", reference);
        }
        self.view_reference = reference;
        self.view = view;

        self.graph.ensure_nearby(self.view_reference, 2);
        self.populate_fresh_nodes();
    }

    pub fn view_reference(&self) -> NodeId {
        self.view_reference
    }

    pub fn view(&self) -> &na::Matrix4<f64> {
        &self.view
    }

    fn populate_fresh_nodes(&mut self) {
        let fresh = self.graph.fresh().to_vec();
        self.graph.clear_fresh();
        for &node in &fresh {
            self.populate_node(node);
        }
        for &node in &fresh {
            for cube in self.graph.cubes_at(node) {
                self.populate_cube(node, cube);
            }
        }
        self.graph.clear_fresh();
    }

    fn populate_node(&mut self, node: NodeId) {
        use common::dodeca::Side;

        *self.graph.get_mut(node) = Some(match self.graph.parent(node) {
            None => true,
            Some(x) => {
                let parent_solid = self
                    .graph
                    .get(self.graph.neighbor(node, x).unwrap())
                    .unwrap();
                if x == Side::A {
                    !parent_solid
                } else {
                    parent_solid
                }
            }
        });
    }

    fn populate_cube(&mut self, node: NodeId, cube: dodeca::Vertex) {
        let contains_border = cube.canonical_sides().iter().any(|&x| x == dodeca::Side::A);

        let voxels = if contains_border {
            let mut data = (0..(SUBDIVISION_FACTOR + 2).pow(3))
                .map(|_| Material::Void)
                .collect::<Vec<_>>()
                .into_boxed_slice();

            let mut rd = ((cube as u32) + 20 * u32::from(node)) % 1000081; // Pseudorandom value to fill chunk with
            const GAP: usize = 0;
            const XGAP: usize = (SUBDIVISION_FACTOR - 1) / 2; // dodeca::Side::A will always correspond to the x coordinate, so let`s flatten it in this direction
            for z in GAP..(SUBDIVISION_FACTOR - GAP) {
                for y in GAP..(SUBDIVISION_FACTOR - GAP) {
                    for x in XGAP..(SUBDIVISION_FACTOR - XGAP) {
                        rd = (37 * rd + 1) % 1000081;
                        data[(x + 1)
                            + (y + 1) * (SUBDIVISION_FACTOR + 2)
                            + (z + 1) * (SUBDIVISION_FACTOR + 2).pow(2)] = if rd % 4 == 1 {
                            Material::Stone
                        } else if rd % 4 == 2 {
                            Material::Dirt
                        } else if rd % 4 == 3 {
                            Material::Sand
                        } else {
                            Material::Void
                        };
                    }
                }
            }
            VoxelData::Dense(data)
        } else {
            VoxelData::Empty
        };
        *self.graph.get_cube_mut(node, cube) = Some(Node {
            surface: None,
            voxels: voxels,
        });
    }
}

pub struct Node {
    pub surface: Option<SlotId>,
    pub voxels: VoxelData,
}

pub enum VoxelData {
    Empty,
    Dense(Box<[Material]>),
}
