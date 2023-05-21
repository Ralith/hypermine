mod collision;

use tracing::warn;

use crate::{
    character_controller::collision::{check_collision, CollisionContext},
    math,
    node::{ChunkLayout, DualGraph},
    proto::{CharacterInput, Position},
    sanitize_motion_input,
    sim_config::CharacterConfig,
    SimConfig,
};

/// Runs a single step of character movement
pub fn run_character_step(
    sim_config: &SimConfig,
    graph: &DualGraph,
    position: &mut Position,
    velocity: &mut na::Vector3<f32>,
    input: &CharacterInput,
    dt_seconds: f32,
) {
    let ctx = CharacterControllerContext {
        cfg: &sim_config.character,
        collision_context: CollisionContext {
            graph,
            chunk_layout: ChunkLayout::new(sim_config.chunk_size as usize),
            radius: sim_config.character.character_radius,
        },
        dt_seconds,
        movement_input: sanitize_motion_input(input.movement),
    };

    if input.no_clip {
        run_no_clip_character_step(&ctx, position, velocity);
    } else {
        run_standard_character_step(&ctx, position, velocity);
    }

    // Renormalize
    position.local = math::renormalize_isometry(&position.local);
    let (next_node, transition_xf) = graph.normalize_transform(position.node, &position.local);
    if next_node != position.node {
        position.node = next_node;
        position.local = transition_xf * position.local;
    }
}

fn run_standard_character_step(
    ctx: &CharacterControllerContext,
    position: &mut Position,
    velocity: &mut na::Vector3<f32>,
) {
    let old_velocity = *velocity;

    // Update velocity
    let current_to_target_velocity = ctx.movement_input * ctx.cfg.max_ground_speed - *velocity;
    let max_delta_velocity = ctx.cfg.ground_acceleration * ctx.dt_seconds;
    if current_to_target_velocity.norm_squared() > math::sqr(max_delta_velocity) {
        *velocity += current_to_target_velocity.normalize() * max_delta_velocity;
    } else {
        *velocity += current_to_target_velocity;
    }

    // Estimate the average velocity by using the average of the old velocity and new velocity,
    // which has the effect of modeling a velocity that changes linearly over the timestep.
    // This is necessary to avoid the following two issues:
    // 1. Input lag, which would occur if only the old velocity was used
    // 2. Movement artifacts, which would occur if only the new velocity was used. One
    //    example of such an artifact is the character moving backwards slightly when they
    //    stop moving after releasing a direction key.
    let estimated_average_velocity = (*velocity + old_velocity) * 0.5;

    apply_velocity(ctx, estimated_average_velocity, position, velocity);
}

fn run_no_clip_character_step(
    ctx: &CharacterControllerContext,
    position: &mut Position,
    velocity: &mut na::Vector3<f32>,
) {
    // If no-clip is on, the velocity field is useless, and we don't want to accidentally
    // save velocity from when no-clip was off.
    *velocity = na::Vector3::zeros();
    position.local *= math::translate_along(
        &(ctx.movement_input * ctx.cfg.no_clip_movement_speed * ctx.dt_seconds),
    );
}

/// Updates the character's position based on the given average velocity while handling collisions.
/// Also updates the velocity based on collisions that occur.
fn apply_velocity(
    ctx: &CharacterControllerContext,
    estimated_average_velocity: na::Vector3<f32>,
    position: &mut Position,
    velocity: &mut na::Vector3<f32>,
) {
    // To prevent an unbounded runtime, we only allow a limited number of collisions to be processed in
    // a single step. If the character encounters excessively complex geometry, it is possible to hit this limit,
    // in which case further movement processing is delayed until the next time step.
    const MAX_COLLISION_ITERATIONS: u32 = 5;

    let mut expected_displacement = estimated_average_velocity * ctx.dt_seconds;
    let mut active_normals = Vec::<na::UnitVector3<f32>>::with_capacity(3);

    let mut all_collisions_resolved = false;

    for _ in 0..MAX_COLLISION_ITERATIONS {
        let collision_result =
            check_collision(&ctx.collision_context, position, &expected_displacement);
        position.local *= collision_result.displacement_transform;

        if let Some(collision) = collision_result.collision {
            // We maintain a list of surface normals that should restrict player movement. We remove normals for
            // surfaces the player is pushed away from and add the surface normal of the latest collision.
            active_normals.retain(|n| n.dot(&collision.normal) < 0.0);
            active_normals.push(collision.normal);

            // Update the expected displacement to whatever is remaining.
            expected_displacement -= collision_result.displacement_vector;
            apply_normals(&active_normals, &mut expected_displacement);

            // Also update the velocity to ensure that walls kill momentum.
            apply_normals(&active_normals, velocity);
        } else {
            all_collisions_resolved = true;
            break;
        }
    }

    if !all_collisions_resolved {
        warn!("A character entity processed too many collisions and collision resolution was cut short.");
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

/// Contains all information about a character that the character controller doesn't change during
/// one of its simulation steps
struct CharacterControllerContext<'a> {
    collision_context: CollisionContext<'a>,
    cfg: &'a CharacterConfig,
    dt_seconds: f32,
    movement_input: na::Vector3<f32>,
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
