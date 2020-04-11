use rand::{
    distributions::{Distribution, Standard},
    Rng,
};

#[macro_use]
mod id;

mod chunks;
pub mod codec;
pub mod cursor;
pub mod dodeca;
pub mod graph;
mod graph_entities;
pub mod math;
pub mod proto;
pub mod world;

pub use chunks::Chunks;
pub use graph_entities::GraphEntities;

// Stable IDs made of 8 random bytes for easy persistent references
mkid!(EntityId: u64);

impl std::fmt::Display for EntityId {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "{:016x}", self.0)
    }
}

impl Distribution<EntityId> for Standard {
    fn sample<R: Rng + ?Sized>(&self, rng: &mut R) -> EntityId {
        EntityId(rng.gen())
    }
}

pub type Step = i32;

pub fn defer<F: FnOnce()>(f: F) -> Defer<F> {
    Defer::new(f)
}

pub struct Defer<F: FnOnce()>(Option<F>);

impl<F: FnOnce()> Defer<F> {
    pub fn new(f: F) -> Self {
        Self(Some(f))
    }

    pub fn invoke(self) {}

    pub fn cancel(mut self) {
        self.0 = None;
    }
}

impl<F: FnOnce()> Drop for Defer<F> {
    fn drop(&mut self) {
        if let Some(f) = self.0.take() {
            f()
        }
    }
}

/// Convert a motion input into direction + speed, with speed clamped to 1.0 and graceful zero
/// handling
pub fn sanitize_motion_input(v: na::Vector3<f32>) -> (na::Unit<na::Vector3<f32>>, f32) {
    if !v.iter().all(|x| x.is_finite()) {
        return (-na::Vector3::z_axis(), 0.0);
    }
    let (direction, speed) = na::Unit::new_and_get(v);
    if speed == 0.0 {
        // Return an arbitrary direction rather than NaN
        (-na::Vector3::z_axis(), speed)
    } else {
        (direction, speed.min(1.0))
    }
}

/// Compute the scaling factor from meters to absolute units, given the number of voxels in a chunk
/// and the approximate size of a voxel in meters.
///
/// Curved spaces have a notion of absolute distance, defined with respect to the curvature. We
/// mostly work in those units, but the conversion from meters, used to scale the player and assets
/// and so forth, is configurable. This effectively allows for configurable curvature.
///
/// Note that exact voxel size varies within each chunk. We reference the mean width of the voxels
/// along the X axis through the center of a chunk.
pub fn meters_to_absolute(chunk_size: u8, voxel_size: f32) -> f32 {
    let a = dodeca::Vertex::A.chunk_to_node() * na::Vector4::new(1.0, 0.5, 0.5, 1.0);
    let b = dodeca::Vertex::A.chunk_to_node() * na::Vector4::new(0.0, 0.5, 0.5, 1.0);
    let minimum_chunk_face_separation = math::distance(&a, &b);
    let absolute_voxel_size = minimum_chunk_face_separation / f64::from(chunk_size);
    absolute_voxel_size as f32 / voxel_size
}
