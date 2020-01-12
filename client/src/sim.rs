use std::time::Duration;

use tracing::{debug, error};

use crate::Net;

/// Game state
pub struct Sim {
    view: na::Isometry3<f32>,
    velocity: na::Vector3<f32>,
    net: Net,
}

impl Sim {
    pub fn new(net: Net) -> Self {
        Self {
            view: na::Isometry3::translation(0.0, 0.0, 32.0),
            velocity: na::zero(),
            net,
        }
    }

    pub fn rotate(&mut self, delta: na::Vector2<f32>) {
        self.view *= na::UnitQuaternion::from_axis_angle(&na::Vector3::y_axis(), -delta.x)
            * na::UnitQuaternion::from_axis_angle(&na::Vector3::x_axis(), -delta.y);
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

        self.view *= na::Translation3::from(self.velocity * dt.as_secs_f32() * 10.0);
    }

    pub fn view(&self) -> na::Isometry3<f32> {
        self.view
    }
}
