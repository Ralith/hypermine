use std::time::Duration;

use tracing::{debug, error};

use crate::Net;
use common::math;

/// Game state
pub struct Sim {
    view: na::Matrix4<f32>,
    velocity: na::Vector3<f32>,
    net: Net,
}

impl Sim {
    pub fn new(net: Net) -> Self {
        Self {
            view: na::Matrix4::identity(),
            velocity: na::zero(),
            net,
        }
    }

    pub fn rotate(&mut self, delta: na::Vector2<f32>) {
        self.view = self.view
            * na::Matrix4::from_axis_angle(&na::Vector3::y_axis(), -delta.x)
            * na::Matrix4::from_axis_angle(&na::Vector3::x_axis(), -delta.y);
    }

    pub fn velocity(&mut self, v: na::Vector3<f32>) {
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

        let dx = self.velocity * dt.as_secs_f32();
        self.view *= math::translate(
            &na::Vector4::new(0.0, 0.0, 0.0, 1.0),
            &na::Vector4::new(dx.x, dx.y, dx.z, 1.0),
        );
    }

    pub fn view(&self) -> na::Matrix4<f32> {
        self.view
    }
}
