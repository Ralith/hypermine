mod config;
mod graphics;
mod metrics;
mod net;
mod prediction;
mod sim;
mod worldgen;

use std::{
    net::{SocketAddr, UdpSocket},
    sync::Arc,
};

use config::Config;
use net::Net;
use sim::Sim;

use ash::extensions::khr;

fn main() {
    // Set up logging
    tracing_subscriber::fmt::init();
    let metrics = crate::metrics::init();

    let dirs = directories::ProjectDirs::from("", "", "hypermine").unwrap();
    let mut config = Config::load(&dirs);

    if config.server.is_none() {
        // spawn an in-process server
        let socket =
            UdpSocket::bind(&"[::1]:0".parse::<SocketAddr>().unwrap()).expect("binding socket");
        config.server = Some(socket.local_addr().unwrap());

        let cert = rcgen::generate_simple_self_signed(vec!["localhost".into()]).unwrap();
        let key = cert.serialize_private_key_der();
        let cert = cert.serialize_der().unwrap();
        let sim_cfg = config.local_simulation.clone();

        std::thread::spawn(move || {
            if let Err(e) = server::run(
                server::NetParams {
                    certificate_chain: quinn::CertificateChain::from_certs(
                        quinn::Certificate::from_der(&cert),
                    ),
                    private_key: quinn::PrivateKey::from_der(&key).unwrap(),
                    socket,
                },
                sim_cfg,
            ) {
                eprintln!("{:#}", e);
                std::process::exit(1);
            }
        });
    }
    let config = Arc::new(config);

    // Create the OS window
    let window = graphics::EarlyWindow::new();
    // Initialize Vulkan with the extensions needed to render to the window
    let core = Arc::new(graphics::Core::new(&[
        khr::Surface::name(),
        window.required_extension(),
    ]));

    // Kick off networking
    let net = net::spawn(config.clone());
    let sim = Sim::new(net);

    // Finish creating the window, including the Vulkan resources used to render to it
    let window = graphics::Window::new(window, core.clone(), config, metrics, sim);

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
