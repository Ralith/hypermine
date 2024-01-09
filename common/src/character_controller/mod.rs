mod collision;
mod vector_bounds;

use std::mem::replace;

use tracing::warn;

use crate::{
    character_controller::{
        collision::{check_collision, Collision, CollisionContext},
        vector_bounds::{BoundedVectors, VectorBound},
    },
    graph::Graph,
    math,
    proto::{CharacterInput, Position},
    sanitize_motion_input,
    sim_config::CharacterConfig,
    SimConfig,
};

/// Runs a single step of character movement
pub fn run_character_step(
    sim_config: &SimConfig,
    graph: &Graph,
    position: &mut Position,
    velocity: &mut na::Vector3<f32>,
    on_ground: &mut bool,
    input: &CharacterInput,
    dt_seconds: f32,
) {
    let ctx = CharacterControllerContext {
        cfg: &sim_config.character,
        collision_context: CollisionContext {
            graph,
            radius: sim_config.character.character_radius,
        },
        up: graph.get_relative_up(position).unwrap(), // up: graph.get_relative_up(position).unwrap_or(na::Vector3::x_axis()),
        dt_seconds,
        movement_input: sanitize_motion_input(input.movement),
        jump_input: input.jump,
    };

    if input.no_clip {
        run_no_clip_character_step(&ctx, position, velocity, on_ground);
    } else {
        run_standard_character_step(&ctx, position, velocity, on_ground);
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
    on_ground: &mut bool,
) {
    let mut ground_normal = None;
    if *on_ground {
        ground_normal = get_ground_normal(ctx, position);
    }

    // Handle jumping
    if ctx.jump_input && ground_normal.is_some() {
        let horizontal_velocity = *velocity - *ctx.up * ctx.up.dot(velocity);
        *velocity = horizontal_velocity + *ctx.up * ctx.cfg.jump_speed;
        ground_normal = None;
    }

    let old_velocity = *velocity;

    // Update velocity
    if let Some(ground_normal) = ground_normal {
        apply_ground_controls(ctx, &ground_normal, velocity);
    } else {
        apply_air_controls(ctx, velocity);

        // Apply air resistance
        *velocity *= (-ctx.cfg.air_resistance * ctx.dt_seconds).exp();
    }

    // Apply gravity
    *velocity -= *ctx.up * ctx.cfg.gravity_acceleration * ctx.dt_seconds;

    // Apply speed cap
    *velocity = velocity.cap_magnitude(ctx.cfg.speed_cap);

    // Estimate the average velocity by using the average of the old velocity and new velocity,
    // which has the effect of modeling a velocity that changes linearly over the timestep.
    // This is necessary to avoid the following two issues:
    // 1. Input lag, which would occur if only the old velocity was used
    // 2. Movement artifacts, which would occur if only the new velocity was used. One
    //    example of such an artifact is the character moving backwards slightly when they
    //    stop moving after releasing a direction key.
    let average_velocity = (*velocity + old_velocity) * 0.5;

    // Handle actual movement
    apply_velocity(
        ctx,
        average_velocity * ctx.dt_seconds,
        position,
        velocity,
        &mut ground_normal,
    );

    *on_ground = ground_normal.is_some();
}

fn run_no_clip_character_step(
    ctx: &CharacterControllerContext,
    position: &mut Position,
    velocity: &mut na::Vector3<f32>,
    on_ground: &mut bool,
) {
    *velocity = ctx.movement_input * ctx.cfg.no_clip_movement_speed;
    *on_ground = false;
    position.local *= math::translate_along(&(*velocity * ctx.dt_seconds));
}

/// Returns the normal corresponding to the ground below the character, up to the `allowed_distance`. If
/// no such ground exists, returns `None`.
fn get_ground_normal(
    ctx: &CharacterControllerContext,
    position: &Position,
) -> Option<na::UnitVector3<f32>> {
    // Since the character can be at a corner between a slanted wall and the ground, the first collision
    // directly below the character is not guaranteed to be part of the ground regardless of whether the
    // character is on the ground. To handle this, we repeatedly redirect the direction we search to be
    // parallel to walls we collide with to ensure that we find the ground if is indeed below the character.
    const MAX_COLLISION_ITERATIONS: u32 = 6;
    let mut allowed_displacement = BoundedVectors::new(
        -ctx.up.into_inner() * ctx.cfg.ground_distance_tolerance,
        None,
    );

    for _ in 0..MAX_COLLISION_ITERATIONS {
        let collision_result = check_collision(
            &ctx.collision_context,
            position,
            allowed_displacement.displacement(),
        );
        if let Some(collision) = collision_result.collision.as_ref() {
            if is_ground(ctx, &collision.normal) {
                // We found the ground, so return its normal.
                return Some(collision.normal);
            }
            allowed_displacement.add_bound(VectorBound::new(
                collision.normal,
                collision.normal,
                true,
            ));
        } else {
            // Return `None` if we travel the whole `allowed_displacement` and don't find the ground.
            return None;
        }
    }
    // Return `None` if we fail to find the ground after the maximum number of attempts
    None
}

/// Checks whether the given normal is flat enough to be considered part of the ground
fn is_ground(ctx: &CharacterControllerContext, normal: &na::UnitVector3<f32>) -> bool {
    let min_slope_up_component = 1.0 / (ctx.cfg.max_ground_slope.powi(2) + 1.0).sqrt();
    normal.dot(&ctx.up) > min_slope_up_component
}

/// Updates the velocity based on user input assuming the character is on the ground
fn apply_ground_controls(
    ctx: &CharacterControllerContext,
    ground_normal: &na::UnitVector3<f32>,
    velocity: &mut na::Vector3<f32>,
) {
    // Set `target_ground_velocity` to have a consistent magnitude regardless
    // of the movement direction, but ensure that the horizontal direction matches
    // the horizontal direction of the intended movement direction.
    let movement_norm = ctx.movement_input.norm();
    let target_ground_velocity = if movement_norm < 1e-16 {
        na::Vector3::zeros()
    } else {
        let mut unit_movement = ctx.movement_input / movement_norm;
        math::project_to_plane(&mut unit_movement, ground_normal, &ctx.up, 0.0);
        unit_movement.try_normalize_mut(1e-16);
        unit_movement * movement_norm * ctx.cfg.max_ground_speed
    };

    // Set `ground_velocity` to be the current velocity's ground-parallel component,
    // using a basis that contains the up vector to ensure that the result is unaffected
    // by gravity.
    let mut ground_velocity = *velocity;
    math::project_to_plane(&mut ground_velocity, ground_normal, &ctx.up, 0.0);

    // Adjust the ground-parallel component of the velocity vector to be closer to the
    // target velocity.
    let current_to_target_velocity = target_ground_velocity - ground_velocity;
    let max_delta_velocity = ctx.cfg.ground_acceleration * ctx.dt_seconds;
    if current_to_target_velocity.norm_squared() > max_delta_velocity.powi(2) {
        *velocity += current_to_target_velocity.normalize() * max_delta_velocity;
    } else {
        *velocity += current_to_target_velocity;
    }
}

/// Updates the velocity based on user input assuming the character is in the air
fn apply_air_controls(ctx: &CharacterControllerContext, velocity: &mut na::Vector3<f32>) {
    *velocity += ctx.movement_input * ctx.cfg.air_acceleration * ctx.dt_seconds;
}

/// Updates the character's position based on the given average velocity while handling collisions.
/// Also updates the velocity and ground normal based on collisions that occur.
fn apply_velocity(
    ctx: &CharacterControllerContext,
    expected_displacement: na::Vector3<f32>,
    position: &mut Position,
    velocity: &mut na::Vector3<f32>,
    ground_normal: &mut Option<na::UnitVector3<f32>>,
) {
    // To prevent an unbounded runtime, we only allow a limited number of collisions to be processed in
    // a single step. If the character encounters excessively complex geometry, it is possible to hit this limit,
    // in which case further movement processing is delayed until the next time step.
    const MAX_COLLISION_ITERATIONS: u32 = 6;

    let mut bounded_vectors = BoundedVectors::new(expected_displacement, Some(*velocity));
    let mut bounded_vectors_without_collisions = bounded_vectors.clone();

    let mut ground_collision_handled = false;

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
            bounded_vectors_without_collisions.scale_displacement(displacement_reduction_factor);

            handle_collision(
                ctx,
                collision,
                &bounded_vectors_without_collisions,
                &mut bounded_vectors,
                ground_normal,
                &mut ground_collision_handled,
            );
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

/// Updates character information based on the results of a single collision
fn handle_collision(
    ctx: &CharacterControllerContext,
    collision: Collision,
    bounded_vectors_without_collisions: &BoundedVectors,
    bounded_vectors: &mut BoundedVectors,
    ground_normal: &mut Option<na::UnitVector3<f32>>,
    ground_collision_handled: &mut bool,
) {
    // Collisions are divided into two categories: Ground collisions and wall collisions.
    // Ground collisions will only affect vertical movement of the character, while wall collisions will
    // push the character away from the wall in a perpendicular direction. If the character is on the ground,
    // we have extra logic: Using a temporary bound to ensure that slanted wall collisions do not lift the
    // character off the ground.
    if is_ground(ctx, &collision.normal) {
        if !*ground_collision_handled {
            // Wall collisions can turn vertical momentum into unwanted horizontal momentum. This can
            // occur if the character jumps at the corner between the ground and a slanted wall. If the wall
            // collision is handled first, this horizontal momentum will push the character away from the wall.
            // This can also occur if the character is on the ground and walks into a slanted wall. A single frame
            // of downward momentum caused by gravity can turn into unwanted horizontal momentum that pushes
            // the character away from the wall. Neither of these issues can occur if the ground collision is
            // handled first, so when computing how the velocity vectors change, we rewrite history as if
            // the ground collision was first. This is only necessary for the first ground collision, since
            // afterwards, there is no more unexpected vertical momentum.
            let old_bounded_vectors =
                replace(bounded_vectors, bounded_vectors_without_collisions.clone());
            bounded_vectors.add_temp_bound(VectorBound::new(collision.normal, ctx.up, false));
            bounded_vectors.add_bound(VectorBound::new(collision.normal, ctx.up, true));
            for bound in old_bounded_vectors.bounds() {
                bounded_vectors.add_bound(bound.clone());
            }
            bounded_vectors.clear_temp_bounds();

            *ground_collision_handled = true;
        } else {
            bounded_vectors.add_temp_bound(VectorBound::new(collision.normal, ctx.up, false));
            bounded_vectors.add_bound(VectorBound::new(collision.normal, ctx.up, true));
            bounded_vectors.clear_temp_bounds();
        }

        *ground_normal = Some(collision.normal);
    } else {
        if let Some(ground_normal) = ground_normal {
            bounded_vectors.add_temp_bound(VectorBound::new(*ground_normal, ctx.up, false));
        }
        bounded_vectors.add_bound(VectorBound::new(collision.normal, collision.normal, true));
        bounded_vectors.clear_temp_bounds();
    }
}

/// Contains all information about a character that the character controller doesn't change during
/// one of its simulation steps
struct CharacterControllerContext<'a> {
    collision_context: CollisionContext<'a>,
    up: na::UnitVector3<f32>,
    cfg: &'a CharacterConfig,
    dt_seconds: f32,
    movement_input: na::Vector3<f32>,
    jump_input: bool,
}
