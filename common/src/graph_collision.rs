use crate::{math, proto::Position};

/// Performs ray casting against a fixed plane.
///
/// This is a temporary implementation, as it is designed to perfrom sphere casting (swept collision query) against
/// the voxels in a `DualGraph` when it is ready for use.
///
/// The `ray` parameter and any resulting hit normals are given in the local coordinate system of `position`.
///
/// The `tanh_distance` is the hyperbolic tangent of the cast_distance, or the distance along the ray to check for hits.
pub fn sphere_cast(position: &Position, ray: &Ray, tanh_distance: f32) -> Option<GraphCastHit> {
    let node_ray = position.local * ray;

    // As a temporary placeholder implementation, check for collision against the z=0 plane going in the -z direction.

    const EPSILON: f32 = 1e-4;
    let normal = math::mtranspose(&position.local) * na::Vector4::z();
    if node_ray.direction.z >= 0.0 {
        // Going the wrong way. Don't report a collision.
        None
    } else if (-EPSILON..=0.0).contains(&node_ray.position.z) {
        // Extra logic to ensure precision issues don't allow a collider to clip through a surface
        Some(GraphCastHit {
            tanh_distance: 0.0,
            normal,
        })
    } else {
        // Compute needed tanh_distance to reach z=0 plane.
        let new_tanh_distance = -node_ray.position.z / node_ray.direction.z;
        if new_tanh_distance >= 0.0 && new_tanh_distance < tanh_distance {
            // Report a collision if the needed tanh_distance is in the right range
            Some(GraphCastHit {
                tanh_distance: new_tanh_distance,
                normal,
            })
        } else {
            None
        }
    }
}

/// Information about the intersection at the end of a ray segment.
#[derive(Debug)]
pub struct GraphCastHit {
    /// The tanh of the distance traveled along the ray to result in this hit.
    pub tanh_distance: f32,

    /// Represents the normal vector of the hit surface in the original coordinate system
    /// of the sphere casting. To get the actual normal vector, project it so that it is orthogonal
    /// to the endpoint in Lorentz space.
    pub normal: na::Vector4<f32>,
}

/// A ray in hyperbolic space. The fields must be lorentz normalized, with `mip(position, position) == -1`,
/// `mip(direction, direction) == 1`, and `mip(position, direction) == 0`.
#[derive(Debug)]
pub struct Ray {
    pub position: na::Vector4<f32>,
    pub direction: na::Vector4<f32>,
}

impl Ray {
    pub fn new(position: na::Vector4<f32>, direction: na::Vector4<f32>) -> Ray {
        Ray {
            position,
            direction,
        }
    }

    /// Returns a point along this ray `atanh(tanh_distance)` units away from the origin. This point
    /// is _not_ lorentz normalized.
    pub fn ray_point(&self, tanh_distance: f32) -> na::Vector4<f32> {
        self.position + self.direction * tanh_distance
    }
}

impl std::ops::Mul<&Ray> for na::Matrix4<f32> {
    type Output = Ray;

    #[inline]
    fn mul(self, rhs: &Ray) -> Self::Output {
        Ray {
            position: self * rhs.position,
            direction: self * rhs.direction,
        }
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_abs_diff_eq;

    use super::*;

    #[test]
    fn temp_shape_cast_example() {
        // Directly hit the z=0 plane with a ray 0.5 units away.
        let ray = math::translate_along(&na::Vector3::new(0.0, 0.0, 0.5))
            * &Ray::new(math::origin(), -na::Vector4::z());
        assert_abs_diff_eq!(
            sphere_cast(&Position::origin(), &ray, 0.9)
                .unwrap()
                .tanh_distance,
            0.5_f32.tanh(),
            epsilon = 1e-4
        );
    }
}
