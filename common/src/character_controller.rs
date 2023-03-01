use tracing::{error, info};

use crate::{
    graph_collision, math,
    node::{ChunkLayout, DualGraph},
    proto::{CharacterInput, Position},
    sanitize_motion_input, SimConfig,
};

pub fn run_character_step(
    cfg: &SimConfig,
    graph: &DualGraph,
    position: &mut Position,
    velocity: &mut na::Vector3<f32>,
    input: &CharacterInput,
    dt_seconds: f32,
) {
    CharacterControllerPass {
        cfg,
        graph,
        position,
        velocity,
        input,
        dt_seconds,
    }
    .step();
}

struct CharacterControllerPass<'a> {
    cfg: &'a SimConfig,
    graph: &'a DualGraph,
    position: &'a mut Position,
    velocity: &'a mut na::Vector3<f32>,
    input: &'a CharacterInput,
    dt_seconds: f32,
}

impl CharacterControllerPass<'_> {
    fn step(&mut self) {
        let movement = sanitize_motion_input(self.input.movement);

        if self.input.no_clip {
            // If no-clip is on, the velocity field is useless, and we don't want to accidentally
            // save velocity from when no-clip was off.
            *self.velocity = na::Vector3::zeros();
            self.position.local *= math::translate_along(
                &(movement * self.cfg.no_clip_movement_speed * self.dt_seconds),
            );
        } else {
            let old_velocity = *self.velocity;

            // Update velocity
            let current_to_target_velocity = movement * self.cfg.max_ground_speed - *self.velocity;
            let max_delta_velocity = self.cfg.ground_acceleration * self.dt_seconds;
            if current_to_target_velocity.norm_squared() > math::sqr(max_delta_velocity) {
                *self.velocity += current_to_target_velocity.normalize() * max_delta_velocity;
            } else {
                *self.velocity += current_to_target_velocity;
            }

            // Estimate the average velocity by using the average of the old velocity and new velocity,
            // which has the effect of modeling a velocity that changes linearly over the timestep.
            // This is necessary to avoid the following two issues:
            // 1. Input lag, which would occur if only the old velocity was used
            // 2. Movement artifacts, which would occur if only the new velocity was used. One
            //    example of such an artifact is the character moving backwards slightly when they
            //    stop moving after releasing a direction key.
            let estimated_average_velocity = (*self.velocity + old_velocity) * 0.5;

            // Update position with collision checking
            let expected_displacement = estimated_average_velocity * self.dt_seconds;
            let collision_checking_result = self.check_collision(&expected_displacement);
            self.position.local *= collision_checking_result.displacement_transform;
            if let Some(collision) = collision_checking_result.collision {
                *self.velocity = na::Vector3::zeros();
                // We are not using collision normals yet, so print them to the console to allow
                // sanity checking. Note that the "orientation" quaternion is not used here, so the
                // numbers will only make sense if the character doesn't look around.
                info!("Collision: normal = {:?}", collision.normal);
            }
        }

        // Renormalize
        self.position.local = math::renormalize_isometry(&self.position.local);
        let (next_node, transition_xf) = self
            .graph
            .normalize_transform(self.position.node, &self.position.local);
        if next_node != self.position.node {
            self.position.node = next_node;
            self.position.local = transition_xf * self.position.local;
        }
    }

    /// Checks for collisions when a character moves with a character-relative displacement vector of `relative_displacement`.
    fn check_collision(&self, relative_displacement: &na::Vector3<f32>) -> CollisionCheckingResult {
        // Split relative_displacement into its norm and a unit vector
        let relative_displacement = relative_displacement.to_homogeneous();
        let displacement_sqr = relative_displacement.norm_squared();
        if displacement_sqr < 1e-16 {
            // Fallback for if the displacement vector isn't large enough to reliably be normalized.
            // Any value that is sufficiently large compared to f32::MIN_POSITIVE should work as the cutoff.
            return CollisionCheckingResult::stationary();
        }

        let displacement_norm = displacement_sqr.sqrt();
        let displacement_normalized = relative_displacement / displacement_norm;

        let ray = graph_collision::Ray::new(math::origin(), displacement_normalized);
        let tanh_distance = displacement_norm.tanh();

        let cast_hit = graph_collision::sphere_cast(
            self.cfg.character_radius,
            self.graph,
            &ChunkLayout::new(self.cfg.chunk_size as usize),
            self.position,
            &ray,
            tanh_distance,
        );

        let cast_hit = match cast_hit {
            Ok(r) => r,
            Err(e) => {
                error!("Collision checking returned {:?}", e);
                return CollisionCheckingResult::stationary();
            }
        };

        let distance = cast_hit
            .as_ref()
            .map_or(tanh_distance, |hit| hit.tanh_distance)
            .atanh();

        let displacement_vector = displacement_normalized.xyz() * distance;
        let displacement_transform = math::translate_along(&displacement_vector);

        CollisionCheckingResult {
            displacement_vector,
            displacement_transform,
            collision: cast_hit.map(|hit| Collision {
                // `CastEndpoint` has its `normal` given relative to the character's original position,
                // but we want the normal relative to the character after the character moves to meet the wall.
                // This normal now represents a contact point at the origin, so we omit the w-coordinate
                // to ensure that it's orthogonal to the origin.
                normal: na::UnitVector3::new_normalize(
                    (math::mtranspose(&displacement_transform) * hit.normal).xyz(),
                ),
            }),
        }
    }
}

struct CollisionCheckingResult {
    ///The displacement allowed by the character before hitting a wall. The result of
    /// `math::translate_along(&displacement_vector)` is `displacement_transform`.
    #[allow(dead_code)]
    displacement_vector: na::Vector3<f32>,

    /// Multiplying the character's position by this matrix will move the character as far as it can up to its intended
    /// displacement until it hits the wall.
    displacement_transform: na::Matrix4<f32>,

    collision: Option<Collision>,
}

struct Collision {
    /// This collision normal faces away from the collision surface and is given in the perspective of the character
    /// _after_ it is transformed by `allowed_displacement`. The 4th coordinate of this normal vector is assumed to be
    /// 0.0 and is therefore omitted.
    normal: na::UnitVector3<f32>,
}

impl CollisionCheckingResult {
    /// Return a CollisionCheckingResult with no movement and no collision; useful if the character is not moving
    /// and has nothing to check collision against. Also useful as a last resort fallback if an unexpected error occurs.
    fn stationary() -> CollisionCheckingResult {
        CollisionCheckingResult {
            displacement_vector: na::Vector3::zeros(),
            displacement_transform: na::Matrix4::identity(),
            collision: None,
        }
    }
}
