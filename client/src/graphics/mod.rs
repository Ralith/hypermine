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
mod sky;
mod surface_extraction;
mod voxels;
mod window;

pub use self::{
    base::Base,
    core::Core,
    draw::Draw,
    sky::Sky,
    surface_extraction::SurfaceExtraction,
    voxels::Voxels,
    window::{EarlyWindow, Window},
};

use ash::vk;

const NOOP_STENCIL_STATE: vk::StencilOpState = vk::StencilOpState {
    fail_op: vk::StencilOp::KEEP,
    pass_op: vk::StencilOp::KEEP,
    depth_fail_op: vk::StencilOp::KEEP,
    compare_op: vk::CompareOp::ALWAYS,
    compare_mask: 0,
    write_mask: 0,
    reference: 0,
};
