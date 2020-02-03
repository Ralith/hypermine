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
mod loader;
mod png_array;
mod sky;
mod surface_extraction;
mod voxels;
mod window;

#[cfg(test)]
mod tests;

pub use self::{
    base::Base,
    core::Core,
    draw::Draw,
    loader::{Asset, LoadCtx, LoadFuture, Loadable, Loader},
    png_array::PngArray,
    sky::Sky,
    surface_extraction::{Chunk, SurfaceExtraction},
    voxels::Voxels,
    window::{EarlyWindow, Window},
};
