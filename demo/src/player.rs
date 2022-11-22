use crate::{
    chunk_ray_tracer::{ChunkRayTracer, RayTracingResult, RayTracingResultHandle},
    math::{HyperboloidMatrix, HyperboloidVector},
    penta::Side,
    sphere_chunk_ray_tracer::SphereChunkRayTracer,
    tessellation::{NodeHandle, Tessellation},
};

pub struct Player {
    radius: f64,
    transform: na::Matrix3<f64>,
    node: NodeHandle,
    vel: na::Vector3<f64>,
    max_ground_speed: f64,
    ground_acceleration: f64,
    ground_normal: Option<na::Vector3<f64>>,
}

pub struct PlayerInput<'a> {
    dt: f64,
    tessellation: &'a Tessellation,
    x_axis: f64,
    jumping: bool,
}

impl<'a> PlayerInput<'a> {
    pub fn new(ctx: &ggez::Context, tessellation: &'a Tessellation, dt: f64) -> PlayerInput<'a> {
        let left_pressed = ggez::input::keyboard::is_key_pressed(ctx, ggez::event::KeyCode::A);
        let right_pressed = ggez::input::keyboard::is_key_pressed(ctx, ggez::event::KeyCode::D);
        let up_pressed = ggez::input::keyboard::is_key_pressed(ctx, ggez::event::KeyCode::W);

        PlayerInput {
            dt,
            tessellation,
            x_axis: if left_pressed { -1. } else { 0. } + if right_pressed { 1. } else { 0. },
            jumping: up_pressed,
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
        if self.input.jumping {
            self.attempt_jump();
        }

        if let Some(ground_normal) = self.player.ground_normal {
            self.apply_ground_controls(&ground_normal);
        } else {
            self.apply_air_controls();
            self.apply_gravity();
        }

        self.apply_velocity();
        self.align_with_gravity();

        if self.player.ground_normal.is_some() {
            self.clamp_to_ground_or_start_falling();
        }

        self.renormalize_transform();
    }

    fn attempt_jump(&mut self) {
        if self.player.ground_normal.is_some() {
            let relative_down = self.get_relative_down();
            let horizontal_vel = self.player.vel.project(&relative_down);
            self.player.vel = horizontal_vel - relative_down * 0.4;
            self.player.ground_normal = None;
        }
    }

    fn apply_ground_controls(&mut self, ground_normal: &na::Vector3<f64>) {
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
    }

    fn apply_air_controls(&mut self) {
        let relative_down = self.get_relative_down();
        self.player.vel -= na::Vector3::new(relative_down.y, -relative_down.x, 0.)
            * self.input.x_axis
            * self.input.dt
            * 0.2;
    }

    fn apply_gravity(&mut self) {
        const GRAVITY: f64 = 1.0;
        self.player.vel += self.get_relative_down() * GRAVITY * self.input.dt;
    }

    fn apply_velocity(&mut self) {
        let mut remaining_dt = self.input.dt;
        for _ in 0..5 {
            let (ray_tracing_result, ray_tracing_transform) =
                self.trace_ray(&(self.player.vel * remaining_dt));

            self.player.transform *= ray_tracing_transform;

            // TODO: Will need to allow two collision normals to act at once, especially in 3D
            if let Some(intersection) = ray_tracing_result.intersection {
                let expected_displacement_norm = self.player.vel.norm() * remaining_dt;
                let actual_displacement_norm = ray_tracing_result.t.atanh();

                if intersection.normal.mip(&self.get_relative_down()) < -0.5 {
                    self.update_ground_normal(&intersection.normal);
                }

                self.player.vel = self.player.vel.project(&intersection.normal);
                remaining_dt *= 1.0 - actual_displacement_norm / expected_displacement_norm;
            } else {
                break;
            }
        }
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

    fn clamp_to_ground_or_start_falling(&mut self) {
        let mut clamp_vector = -na::Vector3::y() * 0.01;
        for _ in 0..5 {
            let (ray_tracing_result, ray_tracing_transform) = self.trace_ray(&clamp_vector);

            // TODO: Will need to allow two collision normals to act at once, especially in 3D
            if let Some(intersection) = ray_tracing_result.intersection {
                let potential_transform = self.player.transform * ray_tracing_transform;
                if intersection.normal.mip(&self.get_relative_down()) < -0.5 {
                    self.player.transform = potential_transform;
                    self.update_ground_normal(&intersection.normal);
                    return;
                } else {
                    // Shrink clamp vector based on travel distance. This is an approximation based on clamp_vector being small.
                    // More accurate shrinkage can be found at apply_velocity_iteration.
                    clamp_vector = (clamp_vector - ray_tracing_transform.column(2)).project_z();
                    // Adjust clamp vector to be perpendicular to the normal vector.
                    clamp_vector = clamp_vector.project(&intersection.normal);
                    self.player.ground_normal = None;
                }
            } else {
                break;
            }
        }
        self.player.ground_normal = None;
    }

    /// This both switches the player's reference node as well as normalizing the transformation matrix
    fn renormalize_transform(&mut self) {
        'a: loop {
            let current_pos = self.player.transform * na::Vector3::z();
            for side in Side::iter() {
                if current_pos.mip(side.normal()) > 0.1 {
                    if let Some(node) = self.input.tessellation.get_neighbor(self.player.node, side)
                    {
                        self.player.node = node;
                        self.player.transform = side.reflection() * self.player.transform;
                    }
                    continue 'a;
                }
            }
            break;
        }

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
                .project(&self.get_relative_down())
                .project(new_ground_normal);
            self.player.ground_normal = Some(*new_ground_normal);
        }
    }

    /// Returns the down direction relative to the player's transform
    fn get_relative_down(&self) -> na::Vector3<f64> {
        let mut relative_down =
            self.player.transform.iso_inverse() * self.input.tessellation.down(self.player.node);
        relative_down.z = 0.0;
        relative_down.normalize_mut();
        relative_down
    }

    /// Completes a ray-tracing collision check for the player. All arguments and returned vectors/matrices
    /// are relative to the player's transformation matrix. Returns a `RayTracingResult` and a matrix that
    /// translates the player to the location where it would meet the wall.
    fn trace_ray(
        &self,
        relative_displacement: &na::Vector3<f64>,
    ) -> (RayTracingResult, na::Matrix3<f64>) {
        const EPSILON: f64 = 1e-5;

        let displacement_sqr = relative_displacement.sqr();
        if displacement_sqr < 1e-16 {
            return (RayTracingResult::new(0.0), na::Matrix3::identity());
        }

        let displacement_norm = displacement_sqr.sqrt();
        let displacement_normalized = relative_displacement / displacement_norm;

        let mut ray_tracing_result = RayTracingResult::new(displacement_norm.tanh() + EPSILON);
        SphereChunkRayTracer {
            radius: self.player.radius,
        }
        .trace_ray_in_tessellation(
            self.input.tessellation,
            self.player.node,
            &(self.player.transform * na::Vector3::z()),
            &(self.player.transform * displacement_normalized),
            &mut RayTracingResultHandle::new(
                &mut ray_tracing_result,
                self.player.transform.iso_inverse(),
            ),
        );

        // TODO: A more robust and complex margin system will likely be needed once the
        // overall algorithm settles more.
        let t_with_epsilon = (ray_tracing_result.t - EPSILON).max(0.0);

        ray_tracing_result.t = t_with_epsilon;
        if let Some(intersection) = ray_tracing_result.intersection.as_mut() {
            intersection.normal = intersection.normal.project_z().m_normalized_vector();
        }

        (
            ray_tracing_result,
            (na::Vector3::z() + displacement_normalized * t_with_epsilon)
                .m_normalized_point()
                .translation(),
        )
    }
}
