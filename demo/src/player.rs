use crate::{
    collision::collision_point,
    math::{HyperboloidMatrix, HyperboloidVector},
    penta::Side,
    tessellation::{NodeHandle, Tessellation},
};

pub struct Player {
    radius: f64,
    transform: na::Matrix3<f64>,
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
            radius: 0.02,
            transform: na::Matrix3::identity(),
            node,
            vel: na::Vector3::zeros(),
            max_ground_speed: 0.5,
            ground_acceleration: 2.5,
            rotation_speed: 1.0,
        }
    }

    pub fn step(&mut self, input: &PlayerInput) {
        // Apply rotation input
        self.transform *=
            na::Matrix3::new_rotation(input.rotation_axis * self.rotation_speed * input.dt);

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
        let mut remaining_dt = input.dt;
        for _ in 0..5 {
            if remaining_dt == 0.0 {
                break;
            }
            remaining_dt = self.apply_velocity_iteration(input.tessellation, remaining_dt);
        }
        self.hop_node(input.tessellation);

        // Prevent errors from building up
        self.transform.qr_normalize();
    }

    // Returns dt remaining
    fn apply_velocity_iteration(&mut self, tessellation: &Tessellation, dt: f64) -> f64 {
        let current_pos = self.transform * na::Vector3::z();
        let candidate_displacement = (self.vel * dt).tangent_displacement_vec();
        let collision = collision_point(
            tessellation,
            self.radius,
            self.node,
            &current_pos,
            &(self.transform * candidate_displacement),
        );
        // TODO: This version of epsilon makes the error margin highly dependent on speed,
        // causing semi-frequent clipping through walls. A more robust and complex margin
        // system is needed once the overall algorithm settles more.
        let t_with_epsilon = (collision.t - 1e-5).max(0.0);

        self.transform *= (na::Vector3::z() + candidate_displacement * t_with_epsilon)
            .m_normalized_point()
            .translation();

        if let Some(normal) = collision.normal {
            let expected_displacement_norm = self.vel.norm() * dt;
            let actual_displacement_norm = (candidate_displacement * t_with_epsilon).norm().atanh();
            let local_normal = (self.transform.iso_inverse() * normal)
                .project_z()
                .m_normalized_vector();
            self.vel = self.vel.project(&local_normal);
            dt * (1.0 - actual_displacement_norm / expected_displacement_norm)
        } else {
            0.0
        }
    }

    fn hop_node(&mut self, tessellation: &Tessellation) -> bool {
        let current_pos = self.transform * na::Vector3::z();
        for side in Side::iter() {
            if current_pos.mip(side.normal()) > 0.1 {
                if let Some(node) = tessellation.get_neighbor(self.node, side) {
                    self.transform = side.reflection() * self.transform;
                    self.node = node;
                }
                return true;
            }
        }

        false
    }

    pub fn pos(&self) -> &na::Matrix3<f64> {
        &self.transform
    }

    pub fn node(&self) -> NodeHandle {
        self.node
    }

    pub fn radius(&self) -> f64 {
        self.radius
    }
}
