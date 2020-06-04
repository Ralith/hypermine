#![allow(clippy::missing_safety_doc)] // Vulkan wrangling is categorically unsafe

mod base;
mod core;
mod draw;
mod fog;
mod frustum;
mod gltf_mesh;
mod meshes;
mod png_array;
pub mod voxels;
mod window;

#[cfg(test)]
mod tests;

pub use self::{
    base::Base,
    core::Core,
    draw::Draw,
    fog::Fog,
    frustum::Frustum,
    gltf_mesh::{GlbFile, GltfScene},
    meshes::{Mesh, Meshes},
    png_array::PngArray,
    voxels::{Chunk, Voxels},
    window::{EarlyWindow, Window},
};

unsafe fn as_bytes<T: Copy>(x: &T) -> &[u8] {
    std::slice::from_raw_parts(x as *const T as *const u8, std::mem::size_of::<T>())
}
