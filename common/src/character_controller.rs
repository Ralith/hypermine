use tracing::{error, warn};

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

            self.apply_velocity(&estimated_average_velocity);
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

    /// Updates the position based on the given average velocity while handling collisions. Also updates the velocity
    /// based on collisions that occur.
    fn apply_velocity(&mut self, estimated_average_velocity: &na::Vector3<f32>) {
        // To prevent an unbounded runtime, we only allow a limited number of collisions to be processed in
        // a single step. If the player encounters excessively complex geometry, it is possible to hit this limit,
        // in which case further movement processing is delayed until the next time step.
        const MAX_COLLISION_ITERATIONS: u32 = 5;

        let mut expected_displacement = estimated_average_velocity * self.dt_seconds;
        let mut active_normals = Vec::<na::UnitVector3<f32>>::with_capacity(3);

        let mut all_collisions_resolved = false;

        for _ in 0..MAX_COLLISION_ITERATIONS {
            let collision_result = self.check_collision(&expected_displacement);
            self.position.local *= collision_result.displacement_transform;

            if let Some(collision) = collision_result.collision {
                // We maintain a list of surface normals that should restrict player movement. We remove normals for
                // surfaces the player is pushed away from and add the surface normal of the latest collision.
                active_normals.retain(|n| n.dot(&collision.normal) < 0.0);
                active_normals.push(collision.normal);

                // Update the expected displacement to whatever is remaining.
                expected_displacement -= collision_result.displacement_vector;
                apply_normals(&active_normals, &mut expected_displacement);

                // Also update the velocity to ensure that walls kill momentum.
                apply_normals(&active_normals, self.velocity);
            } else {
                all_collisions_resolved = true;
                break;
            }
        }

        if !all_collisions_resolved {
            warn!("A character entity processed too many collisions and collision resolution was cut short.");
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

/// Modifies the `subject` by a linear combination of the `normals` to ensure that it is approximately
/// orthogonal to all the normals. The normals are assumed to be linearly independent, and, assuming the final
/// result is nonzero, a small correction is applied to ensure that the subject is moving away from the surfaces
/// the normals represent even when floating point approximation is involved.
fn apply_normals(normals: &[na::UnitVector3<f32>], subject: &mut na::Vector3<f32>) {
    if normals.len() >= 3 {
        // The normals are assumed to be linearly independent, so applying all of them will zero out the subject.
        // There is no need to do any extra logic to handle precision limitations in this case.
        *subject = na::Vector3::zeros();
    }

    // Corrective term to ensure that normals face away from any potential collision surfaces
    const RELATIVE_EPSILON: f32 = 1e-4;
    apply_normals_internal(normals, subject, subject.magnitude() * RELATIVE_EPSILON);
}

/// Modifies the `subject` by a linear combination of the `normals` so that the dot product with each normal is
/// `distance`. The `normals` must be linearly independent for this function to work as expected.
fn apply_normals_internal(
    normals: &[na::UnitVector3<f32>],
    subject: &mut na::Vector3<f32>,
    distance: f32,
) {
    let mut ortho_normals: Vec<na::Vector3<f32>> = normals.iter().map(|n| n.into_inner()).collect();
    for i in 0..normals.len() {
        // Perform the Gram-Schmidt process on `normals` to produce `ortho_normals`.
        for j in i + 1..normals.len() {
            ortho_normals[j] = (ortho_normals[j]
                - ortho_normals[i] * ortho_normals[j].dot(&ortho_normals[i]))
            .normalize();
        }

        // The following formula ensures that the dot product of `subject` and `normals[i]` is `distance.
        // Because we only move the subject along `ortho_normals[i]`, this adjustment does not affect the
        // subject's dot product with any earlier normals.
        *subject += ortho_normals[i]
            * ((distance - subject.dot(&normals[i])) / ortho_normals[i].dot(&normals[i]));
    }
}

struct CollisionCheckingResult {
    /// The displacement allowed for the character before hitting a wall. The result of
    /// `math::translate_along(&displacement_vector)` is `displacement_transform`.
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

#[cfg(test)]
mod tests {
    use approx::assert_abs_diff_eq;

    use super::*;

    #[test]
    fn apply_normals_internal_examples() {
        // Zero vectors (No-op but should not panic)
        test_apply_normals_internal(&[], [0.60, -0.85, 0.90], 0.2);

        // One vector
        test_apply_normals_internal(&[[-0.48, -0.10, -0.67]], [0.85, -0.53, -0.61], 0.2);

        // Two vectors
        test_apply_normals_internal(
            &[[-0.17, 0.07, -0.38], [-0.85, 0.19, -0.84]],
            [0.19, -0.84, -0.62],
            0.2,
        );

        // Three vectors (Not in use as of the creation of this test but should work anyways)
        test_apply_normals_internal(
            &[
                [-0.24, 0.90, -0.06],
                [-0.91, 0.01, 0.44],
                [0.02, -0.65, -0.12],
            ],
            [0.91, -0.01, -0.61],
            0.2,
        );
    }

    fn test_apply_normals_internal(normals: &[[f32; 3]], subject: [f32; 3], distance: f32) {
        let normals: Vec<na::UnitVector3<f32>> = normals
            .iter()
            .map(|n| na::UnitVector3::new_normalize(na::Vector3::new(n[0], n[1], n[2])))
            .collect();
        let mut subject = na::Vector3::new(subject[0], subject[1], subject[2]);

        apply_normals_internal(&normals, &mut subject, distance);
        for normal in normals {
            assert_abs_diff_eq!(subject.dot(&normal), distance, epsilon = 1.0e-5);
        }
    }
}
