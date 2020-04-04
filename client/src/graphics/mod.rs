macro_rules! cstr {
    ($x:literal) => {{
        #[allow(unused_unsafe)]
        unsafe {
            std::ffi::CStr::from_bytes_with_nul_unchecked(concat!($x, "\0").as_bytes())
        }
    }};
}

mod base;
mod core;
mod draw;
mod gltf_mesh;
mod loader;
pub mod lru_table;
mod meshes;
mod png_array;
mod sky;
mod voxels;
mod window;

#[cfg(test)]
mod tests;

pub use self::{
    base::Base,
    core::Core,
    draw::Draw,
    gltf_mesh::GltfMesh,
    loader::{Asset, LoadCtx, LoadFuture, Loadable, Loader},
    lru_table::LruTable,
    meshes::{Mesh, Meshes},
    png_array::PngArray,
    sky::Sky,
    voxels::Voxels,
    window::{EarlyWindow, Window},
};
