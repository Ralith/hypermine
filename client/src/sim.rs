use std::time::Duration;

/// Game state
pub struct Sim {
    view: na::Isometry3<f32>,
    velocity: na::Vector3<f32>,
}

impl Sim {
    pub fn new() -> Self {
        Self {
            view: na::Isometry3::translation(0.0, 0.0, 32.0),
            velocity: na::zero(),
        }
    }

    pub fn rotate(&mut self, delta: na::Vector2<f32>) {
        self.view *= na::UnitQuaternion::from_axis_angle(&na::Vector3::y_axis(), -delta.x)
            * na::UnitQuaternion::from_axis_angle(&na::Vector3::x_axis(), -delta.y);
    }

    pub fn velocity(&mut self, v: na::Vector3<f32>) {
        self.velocity = v;
    }

    pub fn advance(&mut self, dt: Duration) {
        self.view *= na::Translation3::from(self.velocity * dt.as_secs_f32() * 10.0);
    }

    pub fn view(&self) -> na::Isometry3<f32> {
        self.view
    }
}
