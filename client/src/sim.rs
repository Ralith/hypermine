use std::time::Duration;

use tracing::{debug, error};

use crate::{graphics::Chunk, Net};
use common::{
    graph::{Graph, NodeId},
    math,
};

/// Game state
pub struct Sim {
    view_reference: NodeId,
    view: na::Matrix4<f64>,
    velocity: na::Vector3<f64>,
    net: Net,
    pub graph: Graph<Node>,
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
        for node in result.graph.fresh().to_vec() {
            result.populate_node(node);
        }
        result.graph.clear_fresh();

        result
    }

    pub fn rotate(&mut self, delta: na::Vector2<f64>) {
        self.view = self.view
            * na::Matrix4::from_axis_angle(&na::Vector3::y_axis(), -delta.x)
            * na::Matrix4::from_axis_angle(&na::Vector3::x_axis(), -delta.y);
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
    }

    pub fn view_reference(&self) -> NodeId {
        self.view_reference
    }

    pub fn view(&self) -> &na::Matrix4<f64> {
        &self.view
    }

    fn populate_node(&mut self, node: NodeId) {
        for cube in self.graph.cubes_at(node) {
            *self.graph.get_mut(node, cube) = Some(Node { surface: None });
        }
    }
}

#[derive(Default)]
pub struct Node {
    pub surface: Option<Chunk>,
}
