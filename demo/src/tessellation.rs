use na::SimdValue;

use crate::math;

// Order 4 pentagonal tiling
pub const NUM_SIDES: usize = 5;
pub const ORDER: usize = 4;

pub struct Tessellation {
    reflections: [na::Vector3<f64>; NUM_SIDES],
    vertices: [na::Vector3<f64>; NUM_SIDES],
}

impl Tessellation {
    pub fn new() -> Self {
        let side_angle = math::TAU / NUM_SIDES as f64;
        let order_angle = math::TAU / ORDER as f64;

        let cos_side_angle = side_angle.cos();
        let cos_order_angle = order_angle.cos();

        let reflection_r = ((1.0 + cos_order_angle) / (1.0 - cos_side_angle)).sqrt();
        let reflection_z = ((cos_side_angle + cos_order_angle) / (1.0 - cos_side_angle)).sqrt();

        let mut reflections = [na::Vector3::default(); NUM_SIDES];
        let mut vertices = [na::Vector3::default(); NUM_SIDES];

        for (i, reflection) in reflections.iter_mut().enumerate() {
            let theta = side_angle * i as f64;
            *reflection = na::Vector3::new(
                reflection_r * theta.cos(),
                reflection_r * theta.sin(),
                reflection_z,
            );
        }

        for (i, vertex) in vertices.iter_mut().enumerate() {
            *vertex = reflections[(i + 1) % NUM_SIDES] + reflections[(i) % NUM_SIDES];
            *vertex /= (-math::sqr(*vertex)).sqrt();
        }

        Tessellation {
            reflections,
            vertices,
        }
    }
}

impl Default for Tessellation {
    fn default() -> Self {
        Tessellation::new()
    }
}
