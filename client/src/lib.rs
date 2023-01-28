#![allow(clippy::new_without_default)]
#![allow(clippy::needless_borrowed_reference)]

macro_rules! cstr {
    ($x:literal) => {{
        #[allow(unused_unsafe)]
        unsafe {
            std::ffi::CStr::from_bytes_with_nul_unchecked(concat!($x, "\0").as_bytes())
        }
    }};
}

extern crate nalgebra as na;
mod config;
pub mod graphics;
mod lahar_deprecated;
mod loader;
pub mod metrics;
pub mod net;
mod prediction;
pub mod sim;

pub use config::Config;
pub use sim::Sim;

use loader::{Asset, Loader};
use net::Net;
