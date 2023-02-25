use crate::{
    chunk_collision::chunk_sphere_cast,
    dodeca::Vertex,
    math,
    node::{Chunk, ChunkId, ChunkLayout, DualGraph},
    proto::Position,
};

/// Performs sphere casting (swept collision query) against the voxels in the `DualGraph`
///
/// The `ray` parameter and any resulting hit normals are given in the local coordinate system of `position`.
///
/// The `tanh_distance` is the hyperbolic tangent of the cast_distance, or the distance along the ray to check for hits.
///
/// This implementaion is incomplete, and it will only detect intersection with voxels in the "A" chunk of the position's node.
pub fn sphere_cast(
    collider_radius: f32,
    graph: &DualGraph,
    layout: &ChunkLayout,
    position: &Position,
    ray: &Ray,
    tanh_distance: f32,
) -> Option<GraphCastHit> {
    let chunk = ChunkId::new(position.node, Vertex::A);
    let node_transform = position.local;
    let Chunk::Populated {
            voxels: ref voxel_data,
            ..
        } = graph[chunk] else {
            // Collision checking on unpopulated chunk
            panic!("Collision checking on unpopulated chunk");
        };
    let local_ray = chunk.vertex.node_to_dual().cast::<f32>() * node_transform * ray;

    // Check collision within a single chunk
    chunk_sphere_cast(
        collider_radius,
        voxel_data,
        layout,
        &local_ray,
        tanh_distance,
    )
    .map(|hit| GraphCastHit {
        tanh_distance: hit.tanh_distance,
        chunk,
        normal: math::mtranspose(&node_transform) * chunk.vertex.dual_to_node().cast() * hit.normal,
    })
}

/// Information about the intersection at the end of a ray segment.
#[derive(Debug)]
pub struct GraphCastHit {
    /// The tanh of the distance traveled along the ray to result in this hit.
    pub tanh_distance: f32,

    /// Which chunk in the graph the hit occurred in
    pub chunk: ChunkId,

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
