use crate::{math, node_string::Side};

// Order 4 pentagonal tiling
// Note: Despite being constants, it is undefined behavior to change these values, as certain hardcoded unsafe code
// depends on them being set to their current values, NUM_SIDES = 5 and ORDER = 4
pub const NUM_SIDES: usize = 5;
pub const ORDER: usize = 4;

pub struct Tessellation {
    reflection_vectors: [na::Vector3<f64>; NUM_SIDES],
    reflections: [na::Matrix3<f64>; NUM_SIDES],
    vertices: [na::Vector3<f64>; NUM_SIDES],
    voxel_to_hyperboloid: [na::Matrix3<f64>; NUM_SIDES],
    hyperboloid_to_voxel: [na::Matrix3<f64>; NUM_SIDES],
}

impl Tessellation {
    pub fn new() -> Self {
        let side_angle = math::TAU / NUM_SIDES as f64;
        let order_angle = math::TAU / ORDER as f64;

        let cos_side_angle = side_angle.cos();
        let cos_order_angle = order_angle.cos();

        let reflection_r = ((1.0 + cos_order_angle) / (1.0 - cos_side_angle)).sqrt();
        let reflection_z = ((cos_side_angle + cos_order_angle) / (1.0 - cos_side_angle)).sqrt();

        let mut reflection_vectors = [na::Vector3::default(); NUM_SIDES];
        let mut vertices = [na::Vector3::default(); NUM_SIDES];
        let mut voxel_to_hyperboloid = [na::Matrix3::default(); NUM_SIDES];

        for (i, reflection) in reflection_vectors.iter_mut().enumerate() {
            let theta = side_angle * i as f64;
            *reflection = na::Vector3::new(
                reflection_r * theta.cos(),
                reflection_r * theta.sin(),
                reflection_z,
            );
        }

        for (i, vertex) in vertices.iter_mut().enumerate() {
            *vertex = math::normal(
                &reflection_vectors[(i + 1) % NUM_SIDES],
                &reflection_vectors[(i) % NUM_SIDES],
            );
            *vertex /= (-math::sqr(vertex)).sqrt();
        }

        for (i, mat) in voxel_to_hyperboloid.iter_mut().enumerate() {
            let reflector0 = &reflection_vectors[(i) % NUM_SIDES];
            let reflector1 = &reflection_vectors[(i + 1) % NUM_SIDES];
            let origin = na::Vector3::new(0.0, 0.0, 1.0);
            *mat = na::Matrix3::from_columns(&[
                -reflector0 * reflector0.z,
                -reflector1 * reflector1.z,
                origin + reflector0 * reflector0.z + reflector1 * reflector1.z,
            ]);
        }

        Tessellation {
            reflection_vectors,
            reflections: reflection_vectors.map(|v| math::reflection(&v)),
            vertices,
            voxel_to_hyperboloid,
            hyperboloid_to_voxel: voxel_to_hyperboloid.map(|m| m.try_inverse().unwrap()),
        }
    }

    pub fn reflection(&self, side: Side) -> &na::Matrix3<f64> {
        unsafe { self.reflections.get_unchecked(side as usize) }
    }
}

impl Default for Tessellation {
    fn default() -> Self {
        Tessellation::new()
    }
}
