use crate::{
    collision::{collision_point, Collision},
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
            max_ground_speed: 0.2,
            ground_acceleration: 1.0,
            rotation_speed: 1.0,
            ground_normal: None,
        }
    }

    pub fn step(&mut self, input: &PlayerInput) {
        PlayerPhysicsPass {
            player: self,
            input,
        }
        .step();
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

struct PlayerPhysicsPass<'a> {
    player: &'a mut Player,
    input: &'a PlayerInput<'a>,
}

impl<'a> PlayerPhysicsPass<'a> {
    fn step(&mut self) {
        // Apply rotation input
        self.player.transform *= na::Matrix3::new_rotation(
            self.input.rotation_axis * self.player.rotation_speed * self.input.dt,
        );

        // Jumping
        if self.player.ground_normal.is_some() && self.input.y_axis > 0.0 {
            let relative_down = self.get_relative_down();
            let horizontal_vel = self.player.vel.project(&relative_down);
            self.player.vel = horizontal_vel - relative_down * 0.4;
            self.player.ground_normal = None;
        }

        // Apply input to velocity
        if let Some(ground_normal) = self.player.ground_normal {
            let mut target_unit_vel =
                na::Vector3::new(ground_normal.y, -ground_normal.x, 0.) * self.input.x_axis;
            if target_unit_vel.norm_squared() > 1. {
                target_unit_vel.normalize_mut();
            }

            let target_dvel = target_unit_vel * self.player.max_ground_speed - self.player.vel;
            let ground_acceleration_impulse = self.player.ground_acceleration * self.input.dt;
            if target_dvel.norm_squared() > ground_acceleration_impulse.powi(2) {
                self.player.vel += target_dvel.normalize() * ground_acceleration_impulse;
            } else {
                self.player.vel += target_dvel;
            }
        } else {
            let relative_down = self.get_relative_down();
            self.player.vel -= na::Vector3::new(relative_down.y, -relative_down.x, 0.)
                * self.input.x_axis
                * self.input.dt
                * 0.2;
            self.apply_gravity();
        }

        // Apply velocity to position
        let mut remaining_dt = self.input.dt;
        for _ in 0..5 {
            if remaining_dt == 0.0 {
                break;
            }
            remaining_dt = self.apply_velocity_iteration(remaining_dt);
        }
        self.hop_node();
        self.align_with_gravity();

        // Consider whether to stay grounded
        if self.player.ground_normal.is_some() {
            self.clamp_to_ground_or_leave();
        }

        // Prevent errors from building up
        self.player.transform.qr_normalize();
    }

    fn update_ground_normal(&mut self, new_ground_normal: &na::Vector3<f64>) {
        if let Some(ground_normal) = self.player.ground_normal {
            // Use two reflections to move vel from ground_normal to new_ground_normal
            let mid_ground_normal = (ground_normal + new_ground_normal).m_normalized_vector();
            let reflected_vel =
                ground_normal * self.player.vel.mip(&ground_normal) * 2.0 - self.player.vel;
            self.player.vel =
                mid_ground_normal * reflected_vel.mip(&mid_ground_normal) * 2.0 - reflected_vel;
            self.player.ground_normal = Some(*new_ground_normal);
        } else {
            // To avoid undesirable sliding down slopes on every jump, project to a level ground plane
            // before projecting to the actual group plane.
            self.player.vel = self
                .player
                .vel
                //.project(&self.get_relative_down()) TODO: Uncomment
                .project(new_ground_normal);
            self.player.ground_normal = Some(*new_ground_normal);
        }
    }

    fn get_relative_down(&self) -> na::Vector3<f64> {
        let mut relative_down =
            self.player.transform.iso_inverse() * self.input.tessellation.down(self.player.node);
        relative_down.z = 0.0;
        relative_down.normalize_mut();
        relative_down
    }

    fn apply_gravity(&mut self) {
        const GRAVITY: f64 = 1.0;
        self.player.vel += self.get_relative_down() * GRAVITY * self.input.dt;
    }

    // Returns dt remaining
    fn apply_velocity_iteration(&mut self, dt: f64) -> f64 {
        let (collision, collision_transform) = self.get_collision(&(self.player.vel * dt));

        self.player.transform *= collision_transform;

        // TODO: Will need to allow two collision normals to act at once, especially in 3D
        if let Some(normal) = collision.normal {
            let expected_displacement_norm = self.player.vel.norm() * dt;
            let actual_displacement_norm = collision.t.atanh();

            if normal.mip(&self.get_relative_down()) < -0.5 {
                self.update_ground_normal(&normal);
            }

            self.player.vel = self.player.vel.project(&normal);
            dt * (1.0 - actual_displacement_norm / expected_displacement_norm)
        } else {
            0.0
        }
    }

    fn clamp_to_ground_or_leave(&mut self) {
        let mut clamp_vector = -na::Vector3::y() * 0.01;
        for _ in 0..5 {
            let (collision, collision_transform) = self.get_collision(&clamp_vector);

            // TODO: Will need to allow two collision normals to act at once, especially in 3D
            if let Some(normal) = collision.normal {
                let potential_transform = self.player.transform * collision_transform;
                if normal.mip(&self.get_relative_down()) < -0.5 {
                    self.player.transform = potential_transform;
                    self.update_ground_normal(&normal);
                    return;
                } else {
                    // Shrink clamp vector based on travel distance. This is an approximation based on clamp_vector being small.
                    // More accurate shrinkage can be found at apply_velocity_iteration.
                    clamp_vector = (clamp_vector - collision_transform.column(2)).project_z();
                    // Adjust clamp vector to be perpendicular to the normal vector.
                    clamp_vector = clamp_vector.project(&normal);
                    self.player.ground_normal = None;
                }
            } else {
                break;
            }
        }
        self.player.ground_normal = None;
    }

    fn get_collision(
        &self,
        relative_displacement: &na::Vector3<f64>,
    ) -> (Collision, na::Matrix3<f64>) {
        const EPSILON: f64 = 1e-5;

        let displacement_sqr = relative_displacement.sqr();
        if displacement_sqr < 1e-16 {
            return (
                Collision {
                    t: 0.0,
                    normal: None,
                },
                na::Matrix3::identity(),
            );
        }

        let displacement_norm = displacement_sqr.sqrt();
        let displacement_normalized = relative_displacement / displacement_norm;

        let mut collision = collision_point(
            self.input.tessellation,
            self.player.radius,
            self.player.node,
            &(self.player.transform * na::Vector3::z()),
            &(self.player.transform * displacement_normalized),
            displacement_norm.tanh() + EPSILON,
        );

        // TODO: A more robust and complex margin system will likely be needed once the
        // overall algorithm settles more.
        let t_with_epsilon = (collision.t - EPSILON).max(0.0);
        collision.t = t_with_epsilon;
        collision.normal = collision.normal.map(|n| {
            (self.player.transform.iso_inverse() * n)
                .project_z()
                .m_normalized_vector()
        });
        (
            collision,
            (na::Vector3::z() + displacement_normalized * t_with_epsilon)
                .m_normalized_point()
                .translation(),
        )
    }

    fn hop_node(&mut self) -> bool {
        let current_pos = self.player.transform * na::Vector3::z();
        for side in Side::iter() {
            if current_pos.mip(side.normal()) > 0.1 {
                if let Some(node) = self.input.tessellation.get_neighbor(self.player.node, side) {
                    self.player.node = node;
                    self.player.transform = side.reflection() * self.player.transform;
                }
                return true;
            }
        }

        false
    }

    fn align_with_gravity(&mut self) {
        let relative_down = self.get_relative_down();
        let theta = relative_down[0].atan2(-relative_down[1]);
        self.player.transform *= na::Matrix3::new_rotation(theta);
        self.player.vel = na::Matrix3::new_rotation(-theta) * self.player.vel;
        self.player.ground_normal = self
            .player
            .ground_normal
            .map(|n| na::Matrix3::new_rotation(-theta) * n);
    }
}
