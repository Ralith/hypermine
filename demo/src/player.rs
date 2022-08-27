use crate::{math, tessellation::{NodeHandle, Tessellation}, collision::is_colliding};

pub struct Player {
    pos: na::Matrix3<f64>,
    node: NodeHandle,
    vel: na::Vector3<f64>,
    max_ground_speed: f64,
    ground_acceleration: f64,
    rotation_speed: f64,
}

pub struct PlayerInput<'a> {
    dt: f64,
    tessellation: &'a Tessellation,
    x_axis: f64,
    y_axis: f64,
    rotation_axis: f64,
}

impl<'a> PlayerInput<'a> {
    pub fn new(ctx: &ggez::Context, tessellation: &'a Tessellation, dt: f64) -> PlayerInput<'a> {
        let left_pressed = ggez::input::keyboard::is_key_pressed(ctx, ggez::event::KeyCode::A);
        let right_pressed = ggez::input::keyboard::is_key_pressed(ctx, ggez::event::KeyCode::D);
        let down_pressed = ggez::input::keyboard::is_key_pressed(ctx, ggez::event::KeyCode::S);
        let up_pressed = ggez::input::keyboard::is_key_pressed(ctx, ggez::event::KeyCode::W);
        let cw_pressed = ggez::input::keyboard::is_key_pressed(ctx, ggez::event::KeyCode::E);
        let ccw_pressed = ggez::input::keyboard::is_key_pressed(ctx, ggez::event::KeyCode::Q);

        PlayerInput {
            dt,
            tessellation,
            x_axis: if left_pressed { -1. } else { 0. } + if right_pressed { 1. } else { 0. },
            y_axis: if down_pressed { -1. } else { 0. } + if up_pressed { 1. } else { 0. },
            rotation_axis: if cw_pressed { -1. } else { 0. } + if ccw_pressed { 1. } else { 0. },
        }
    }
}

impl Player {
    pub fn new(node: NodeHandle) -> Player {
        Player {
            pos: na::Matrix3::identity(),
            node,
            vel: na::Vector3::zeros(),
            max_ground_speed: 0.5,
            ground_acceleration: 2.5,
            rotation_speed: 1.0,
        }
    }

    pub fn step(&mut self, input: &PlayerInput) {
        // Apply rotation input
        self.pos *= math::rotation(input.rotation_axis * self.rotation_speed * input.dt);

        // Apply input to velocity
        let mut target_unit_vel = na::Vector3::new(input.x_axis, input.y_axis, 0.);
        if target_unit_vel.norm_squared() > 1. {
            target_unit_vel.normalize_mut();
        }

        let target_dvel = target_unit_vel * self.max_ground_speed - self.vel;
        let ground_acceleration_impulse = self.ground_acceleration * input.dt;
        if target_dvel.norm_squared() > ground_acceleration_impulse.powi(2) {
            self.vel += target_dvel.normalize() * ground_acceleration_impulse;
        } else {
            self.vel += target_dvel;
        }

        // Apply velocity to position
        let candidate_pos = self.pos * math::displacement(&(self.vel * input.dt));
        if !is_colliding(input.tessellation, self.node, &(candidate_pos * na::Vector3::new(0.0, 0.0, 1.0))) {
            self.pos *= math::displacement(&(self.vel * input.dt));
        }

        // Prevent errors from building up
        math::qr_normalize(&mut self.pos);
    }

    pub fn pos(&self) -> &na::Matrix3<f64> {
        &self.pos
    }
}
