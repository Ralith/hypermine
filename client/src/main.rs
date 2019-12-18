mod graphics;

use std::fs;
use std::sync::Arc;

use ash::extensions::khr;

fn main() {
    // Set up logging
    tracing_subscriber::fmt::init();

    // Read GPU driver caches
    let dirs = directories::ProjectDirs::from("", "", "hypermine").unwrap();
    let pipeline_cache_path = dirs.cache_dir().join("pipeline_cache");
    let pipeline_cache_data = fs::read(&pipeline_cache_path).unwrap_or_else(|_| vec![]);

    // Create the OS window
    let window = graphics::EarlyWindow::new();
    // Initialize Vulkan with the extensions needed to render to the window
    let core = Arc::new(graphics::Core::new(&[
        khr::Surface::name(),
        window.required_extension(),
    ]));
    // Finish creating the window, including the Vulkan resources used to render to it
    let window = graphics::Window::new(window, core.clone());

    // Initialize widely-shared graphics resources
    let gfx = Arc::new(
        graphics::Base::new(
            core,
            &pipeline_cache_data,
            &[khr::Swapchain::name()],
            |physical, queue_family| window.supports(physical, queue_family),
        )
        .unwrap(),
    );

    // Run the window's event loop
    window.run(gfx);
}
