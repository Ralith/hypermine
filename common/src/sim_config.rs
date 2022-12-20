use std::time::Duration;

use serde::{Deserialize, Serialize};

use crate::{dodeca, math};

/// Manually specified simulation config parameters
#[derive(Serialize, Deserialize, Default)]
#[serde(deny_unknown_fields)]
pub struct SimConfigRaw {
    /// Number of steps per second
    pub rate: Option<u16>,
    /// Maximum distance at which anything can be seen in meters
    pub view_distance: Option<f32>,
    pub input_queue_size_ms: Option<u16>,
    /// Number of voxels along the edge of a chunk
    pub chunk_size: Option<u8>,
    /// Approximate length of the edge of a voxel in meters
    ///
    /// Curved spaces have a notion of absolute distance, defined with respect to the curvature. We
    /// mostly work in those units, but the conversion from meters, used to scale the player and assets
    /// and so forth, is configurable. This effectively allows for configurable curvature.
    ///
    /// Note that exact voxel size varies within each chunk. We reference the mean width of the voxels
    /// along the X axis through the center of a chunk.
    pub voxel_size: Option<f32>,
    /// Character movement speed in m/s during no-clip
    pub no_clip_movement_speed: Option<f32>,
    /// Character maximumum movement speed while on the ground in m/s
    pub max_ground_speed: Option<f32>,
    /// Character acceleration while on the ground in m/s^2
    pub ground_acceleration: Option<f32>,
}

/// Complete simulation config parameters
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SimConfig {
    /// Amount of time between each step. Inverse of the rate
    pub step_interval: Duration,
    pub view_distance: f32,
    pub input_queue_size: Duration,
    pub chunk_size: u8,
    pub no_clip_movement_speed: f32,
    pub max_ground_speed: f32,
    pub ground_acceleration: f32,
    /// Scaling factor converting meters to absolute units
    pub meters_to_absolute: f32,
}

impl SimConfig {
    pub fn from_raw(x: &SimConfigRaw) -> Self {
        let chunk_size = x.chunk_size.unwrap_or(12);
        let voxel_size = x.voxel_size.unwrap_or(1.0);
        let meters_to_absolute = meters_to_absolute(chunk_size, voxel_size);
        SimConfig {
            step_interval: Duration::from_secs(1) / x.rate.unwrap_or(10) as u32,
            view_distance: x.view_distance.unwrap_or(90.0) * meters_to_absolute,
            input_queue_size: Duration::from_millis(x.input_queue_size_ms.unwrap_or(50).into()),
            chunk_size,
            no_clip_movement_speed: x.no_clip_movement_speed.unwrap_or(12.0) * meters_to_absolute,
            max_ground_speed: x.max_ground_speed.unwrap_or(6.0) * meters_to_absolute,
            ground_acceleration: x.ground_acceleration.unwrap_or(30.0) * meters_to_absolute,
            meters_to_absolute,
        }
    }
}

/// Compute the scaling factor from meters to absolute units, given the number of voxels in a chunk
/// and the approximate size of a voxel in meters.
fn meters_to_absolute(chunk_size: u8, voxel_size: f32) -> f32 {
    let a = dodeca::Vertex::A.chunk_to_node() * na::Vector4::new(1.0, 0.5, 0.5, 1.0);
    let b = dodeca::Vertex::A.chunk_to_node() * na::Vector4::new(0.0, 0.5, 0.5, 1.0);
    let minimum_chunk_face_separation = math::distance(&a, &b);
    let absolute_voxel_size = minimum_chunk_face_separation / f64::from(chunk_size);
    absolute_voxel_size as f32 / voxel_size
}
