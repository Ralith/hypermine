mod graphics;

use std::fs;
use std::sync::Arc;

use ash::extensions::khr;

fn main() {
    tracing_subscriber::fmt::init();

    let dirs = directories::ProjectDirs::from("", "", "hypermine").unwrap();
    let pipeline_cache_path = dirs.cache_dir().join("pipeline_cache");
    let pipeline_cache_data = fs::read(&pipeline_cache_path).unwrap_or_else(|_| vec![]);

    let window = graphics::EarlyWindow::new();
    let core = Arc::new(graphics::Core::new(&[
        khr::Surface::name(),
        window.required_extension(),
    ]));
    let window = graphics::Window::new(window, core.clone());

    let gfx = Arc::new(
        graphics::Base::new(
            core,
            &pipeline_cache_data,
            &[khr::Swapchain::name()],
            |physical, queue_family| window.supports(physical, queue_family),
        )
        .unwrap(),
    );

    window.run(gfx);
}
