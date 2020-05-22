mod base;
mod core;
mod draw;
mod fog;
mod gltf_mesh;
mod meshes;
mod png_array;
mod voxels;
mod window;

#[cfg(test)]
mod tests;

pub use self::{
    base::Base,
    core::Core,
    draw::Draw,
    fog::Fog,
    gltf_mesh::{GlbFile, GltfScene},
    meshes::{Mesh, Meshes},
    png_array::PngArray,
    voxels::{Chunk, Voxels},
    window::{EarlyWindow, Window},
};

#[rustfmt::skip]
/// Compute right-handed y-up inverse Z perspective projection matrix with far plane at 1.0
///
/// This projection is applied to Beltrami-Klein vertices, which fall within a ball of radius 1
/// around the viewpoint, so a far plane of 1.0 gives us ideal distribution of depth precision.
fn projection(left: f32, right: f32, down: f32, up: f32, znear: f32) -> na::Projective3<f32> {
    // Based on http://dev.theomader.com/depth-precision/ + OpenVR docs
    let zfar = 1.0;
    let left = left.tan();
    let right = right.tan();
    let down = down.tan();
    let up = up.tan();
    let idx = 1.0 / (right - left);
    let idy = 1.0 / (down - up);
    let sx = right + left;
    let sy = down + up;
    // For an infinite far plane instead, za = 0 and zb = znear
    let za = -zfar / (znear - zfar) - 1.0;
    let zb = -(znear * zfar) / (znear - zfar);
    na::Projective3::from_matrix_unchecked(
        na::Matrix4::new(
            2.0 * idx,       0.0, sx * idx, 0.0,
                  0.0, 2.0 * idy, sy * idy, 0.0,
                  0.0,       0.0,      za,  zb,
                  0.0,       0.0,     -1.0, 0.0))
}
