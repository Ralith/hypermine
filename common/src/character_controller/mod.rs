mod collision;
mod vector_bounds;

use tracing::warn;

use crate::{
    character_controller::{
        collision::{check_collision, CollisionContext},
        vector_bounds::{BoundedVectors, VectorBound},
    },
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

    apply_velocity(
        ctx,
        estimated_average_velocity * ctx.dt_seconds,
        position,
        velocity,
    );
}

fn run_no_clip_character_step(
    ctx: &CharacterControllerContext,
    position: &mut Position,
    velocity: &mut na::Vector3<f32>,
) {
    *velocity = ctx.movement_input * ctx.cfg.no_clip_movement_speed;
    position.local *= math::translate_along(&(*velocity * ctx.dt_seconds));
}

/// Updates the character's position based on the given average velocity while handling collisions.
/// Also updates the velocity based on collisions that occur.
fn apply_velocity(
    ctx: &CharacterControllerContext,
    expected_displacement: na::Vector3<f32>,
    position: &mut Position,
    velocity: &mut na::Vector3<f32>,
) {
    // To prevent an unbounded runtime, we only allow a limited number of collisions to be processed in
    // a single step. If the character encounters excessively complex geometry, it is possible to hit this limit,
    // in which case further movement processing is delayed until the next time step.
    const MAX_COLLISION_ITERATIONS: u32 = 5;

    let mut bounded_vectors = BoundedVectors::new(expected_displacement, Some(*velocity));

    let mut all_collisions_resolved = false;
    for _ in 0..MAX_COLLISION_ITERATIONS {
        let collision_result = check_collision(
            &ctx.collision_context,
            position,
            bounded_vectors.displacement(),
        );
        position.local *= collision_result.displacement_transform;

        if let Some(collision) = collision_result.collision {
            // Update the expected displacement to represent a reduction in the remaining dt
            let displacement_reduction_factor = 1.0
                - collision_result.displacement_vector.magnitude()
                    / bounded_vectors.displacement().magnitude();
            bounded_vectors.scale_displacement(displacement_reduction_factor);

            // Block further movement towards the wall.
            bounded_vectors.add_bound(VectorBound::new(collision.normal, collision.normal));
        } else {
            all_collisions_resolved = true;
            break;
        }
    }

    if !all_collisions_resolved {
        warn!("A character entity processed too many collisions and collision resolution was cut short.");
    }

    *velocity = *bounded_vectors.velocity().unwrap();
}

/// Contains all information about a character that the character controller doesn't change during
/// one of its simulation steps
struct CharacterControllerContext<'a> {
    collision_context: CollisionContext<'a>,
    cfg: &'a CharacterConfig,
    dt_seconds: f32,
    movement_input: na::Vector3<f32>,
}
