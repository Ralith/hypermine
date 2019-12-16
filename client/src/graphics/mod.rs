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
mod window;

pub use self::base::Base;
pub use self::core::Core;
pub use self::draw::Draw;
pub use self::window::{EarlyWindow, Window};
