use std::time::Duration;

use tracing::{debug, error};

use crate::Net;
use common::math;

/// Game state
pub struct Sim {
    view: na::Matrix4<f64>,
    velocity: na::Vector3<f64>,
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
        self.view = math::renormalize_isometry(&self.view) * math::translate_along(&dir, distance);
    }

    pub fn view(&self) -> na::Matrix4<f32> {
        na::convert(self.view)
    }
}
