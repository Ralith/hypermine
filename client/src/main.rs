mod config;
mod graphics;
mod worldgen;
mod net;
mod sim;

use config::Config;
use net::Net;
use sim::Sim;

use std::sync::Arc;

use ash::extensions::khr;

fn main() {
    // Set up logging
    tracing_subscriber::fmt::init();

    // spawn the server in a new thread
    std::thread::spawn(|| {
        if let Err(e) = server::run() {
            eprintln!("{:#}", e);
            std::process::exit(1);
        }
    });

    let dirs = directories::ProjectDirs::from("", "", "hypermine").unwrap();
    let config = Arc::new(Config::load(&dirs));

    // Create the OS window
    let window = graphics::EarlyWindow::new();
    // Initialize Vulkan with the extensions needed to render to the window
    let core = Arc::new(graphics::Core::new(&[
        khr::Surface::name(),
        window.required_extension(),
    ]));

    // Kick off networking
    let net = net::spawn(config.clone());
    let sim = Sim::new(net, config.clone());

    // Finish creating the window, including the Vulkan resources used to render to it
    let window = graphics::Window::new(window, core.clone(), config, sim);

    // Initialize widely-shared graphics resources
    let gfx = Arc::new(
        graphics::Base::new(
            core,
            Some(dirs.cache_dir().join("pipeline_cache")),
            &[khr::Swapchain::name()],
            |physical, queue_family| window.supports(physical, queue_family),
        )
        .unwrap(),
    );

    // Run the window's event loop
    window.run(gfx);
}
