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
    ground_normal: Option<na::Vector3<f64>>,
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
            ground_normal: None,
        }
    }

    pub fn step(&mut self, input: &PlayerInput) {
        // Apply rotation input
        self.transform *=
            na::Matrix3::new_rotation(input.rotation_axis * self.rotation_speed * input.dt);

        // Jumping
        if self.ground_normal.is_some() && input.y_axis > 0.0 {
            // TODO: Use a set-speed like jumping instead of an add-speed like jumping.
            self.vel -= self.get_relative_down(input.tessellation);
            self.ground_normal = None;
        }

        // Apply input to velocity
        if let Some(ground_normal) = self.ground_normal {
            let mut target_unit_vel =
                na::Vector3::new(ground_normal.y, -ground_normal.x, 0.) * input.x_axis;
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
        } else {
            let relative_down = self.get_relative_down(input.tessellation);
            self.vel -=
                na::Vector3::new(relative_down.y, -relative_down.x, 0.) * input.x_axis * input.dt;
            self.apply_gravity(&relative_down, input.dt);
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
        self.align_with_gravity(&self.get_relative_down(input.tessellation));

        // Consider whether to stay grounded
        if self.ground_normal.is_some() {
            self.clamp_to_ground_or_leave(input.tessellation);
        }

        // Prevent errors from building up
        self.transform.qr_normalize();
    }

    fn update_ground_normal(&mut self, new_ground_normal: &na::Vector3<f64>) {
        if let Some(ground_normal) = self.ground_normal {
            // Use two reflections to move vel from ground_normal to new_ground_normal
            let mid_ground_normal = (ground_normal + new_ground_normal).m_normalized_vector();
            let reflected_vel = ground_normal * self.vel.mip(&ground_normal) * 2.0 - self.vel;
            self.vel =
                mid_ground_normal * reflected_vel.mip(&mid_ground_normal) * 2.0 - reflected_vel;
            self.ground_normal = Some(*new_ground_normal);
        } else {
            self.vel = self.vel.project(new_ground_normal);
            self.ground_normal = Some(*new_ground_normal);
        }
    }

    fn get_relative_down(&self, tessellation: &Tessellation) -> na::Vector3<f64> {
        let mut relative_down = self.transform.iso_inverse() * tessellation.down(self.node);
        relative_down.z = 0.0;
        relative_down.normalize_mut();
        relative_down
    }

    fn apply_gravity(&mut self, relative_down: &na::Vector3<f64>, dt: f64) {
        const GRAVITY: f64 = 1.0;
        self.vel += relative_down * GRAVITY * dt;
    }

    // Returns dt remaining
    fn apply_velocity_iteration(&mut self, tessellation: &Tessellation, dt: f64) -> f64 {
        const EPSILON: f64 = 1e-5;

        let current_pos = self.transform * na::Vector3::z();
        let candidate_displacement = self.vel * dt;
        let candidate_displacement_sqr = candidate_displacement.sqr();
        if candidate_displacement_sqr < 1e-16 {
            return 0.0;
        }

        let candidate_displacement_norm = candidate_displacement_sqr.sqrt();
        let candidate_displacement_normalized =
            candidate_displacement / candidate_displacement_norm;
        let collision = collision_point(
            tessellation,
            self.radius,
            self.node,
            &current_pos,
            &(self.transform * candidate_displacement_normalized),
            candidate_displacement_norm.tanh() + EPSILON,
        );

        // TODO: A more robust and complex margin system will likely be needed once the
        // overall algorithm settles more.
        let t_with_epsilon = (collision.t - EPSILON).max(0.0);

        self.transform *= (na::Vector3::z() + candidate_displacement_normalized * t_with_epsilon)
            .m_normalized_point()
            .translation();

        // TODO: Will need to allow two collision normals to act at once, especially in 3D
        if let Some(normal) = collision.normal {
            let expected_displacement_norm = self.vel.norm() * dt;
            let actual_displacement_norm = (candidate_displacement_normalized * t_with_epsilon)
                .norm()
                .atanh();
            let local_normal = (self.transform.iso_inverse() * normal)
                .project_z()
                .m_normalized_vector();

            if local_normal.mip(&self.get_relative_down(tessellation)) < -0.5 {
                self.update_ground_normal(&local_normal);
            }

            self.vel = self.vel.project(&local_normal);
            dt * (1.0 - actual_displacement_norm / expected_displacement_norm)
        } else {
            0.0
        }
    }

    fn clamp_to_ground_or_leave(&mut self, tessellation: &Tessellation) {
        const EPSILON: f64 = 1e-5;

        let collision = collision_point(
            tessellation,
            self.radius,
            self.node,
            &(self.transform * na::Vector3::z()),
            &(self.transform * -na::Vector3::y()),
            0.01,
        );

        if let Some(normal) = collision.normal {
            let t_with_epsilon = (collision.t - EPSILON).max(0.0);

            let potential_transform = self.transform
                * (na::Vector3::z() - na::Vector3::y() * t_with_epsilon)
                    .m_normalized_point()
                    .translation();

            let local_normal = (potential_transform.iso_inverse() * normal)
                .project_z()
                .m_normalized_vector();

            if local_normal.mip(&self.get_relative_down(tessellation)) < -0.5 {
                self.transform = potential_transform;
                self.update_ground_normal(&local_normal);
            } else {
                self.ground_normal = None;
            }
        } else {
            self.ground_normal = None;
        }
    }

    fn hop_node(&mut self, tessellation: &Tessellation) -> bool {
        let current_pos = self.transform * na::Vector3::z();
        for side in Side::iter() {
            if current_pos.mip(side.normal()) > 0.1 {
                if let Some(node) = tessellation.get_neighbor(self.node, side) {
                    self.node = node;
                    self.transform = side.reflection() * self.transform;
                }
                return true;
            }
        }

        false
    }

    fn align_with_gravity(&mut self, relative_down: &na::Vector3<f64>) {
        let theta = relative_down[0].atan2(-relative_down[1]);
        self.transform *= na::Matrix3::new_rotation(theta);
        self.vel = na::Matrix3::new_rotation(-theta) * self.vel;
        self.ground_normal = self
            .ground_normal
            .map(|n| na::Matrix3::new_rotation(-theta) * n);
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
