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
mod fog;
mod gltf_mesh;
mod loader;
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
    loader::{Asset, LoadCtx, LoadFuture, Loadable, Loader},
    meshes::{Mesh, Meshes},
    png_array::PngArray,
    voxels::Voxels,
    window::{EarlyWindow, Window},
};
