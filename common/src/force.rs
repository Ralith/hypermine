// use crate::Plane;

// this module covers forces like gravity and the normal force.

// 1/2 absolute units per second squared of acceleration; it will take two seconds for something at rest to fall the length of one chunk.
const GRAVITY_INTENSITY: f64 = 0.50_f64;

const BUOYANT_INTENSITY: f64 = GRAVITY_INTENSITY * 2.5_f64;

// 1000 inverse absolute units of drag acceleration; The equilibruim speed of something under all three forces will be 0.1 absolute units a second.
const GROUND_DRAG_INTENSITY: f64 = 10000_f64;
const AIR_DRAG_INTENSITY: f64 = GROUND_DRAG_INTENSITY / 15_f64;

// gravity pulls downward
pub fn gravity(down: &na::Vector4<f64>, height: f64) -> na::Vector4<f64> {
    // return down * GRAVITY_INTENSITY;
    // versoin of gravity that conforms to divergence theorem
    return down * GRAVITY_INTENSITY / height.cosh().powi(2);
}

// currently hard collions aren't possible, so you will just "float" to the top of the ground.
pub fn normal_buoyant(down: &na::Vector4<f64>, height: f64, is_collsion: bool) -> na::Vector4<f64> {
    if is_collsion {
        return down * -BUOYANT_INTENSITY / height.cosh().powi(2);
    }
    return na::Vector4::zeros();
}

// when sinking in to the ground, you will expereince drag which acts agaist whichever way you are moving
// and which gains strength proportional to the speed cubed.
pub fn normal_drag(is_collsion: bool, current_velocity: na::Vector4<f64>) -> na::Vector4<f64> {
    let speed_squared = current_velocity.norm().powi(2);

    if is_collsion {
        return -(GROUND_DRAG_INTENSITY * speed_squared).min(1_f64) * current_velocity;
    }
    return -(AIR_DRAG_INTENSITY * speed_squared).min(1_f64) * current_velocity;
}
