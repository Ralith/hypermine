use std::{
    net::{SocketAddr, UdpSocket},
    sync::Arc,
};

use client::{graphics, metrics, net, Config};
use save::Save;

use ash::khr;
use tracing::{error, error_span, info};

fn main() {
    // Set up logging
    common::init_tracing();
    let metrics = crate::metrics::init();

    let dirs = directories::ProjectDirs::from("", "", "hypermine").unwrap();
    let mut config = Config::load(&dirs);

    if config.server.is_none() {
        // spawn an in-process server
        let socket =
            UdpSocket::bind("[::1]:0".parse::<SocketAddr>().unwrap()).expect("binding socket");
        config.server = Some(socket.local_addr().unwrap());

        let cert = rcgen::generate_simple_self_signed(vec!["localhost".into()]).unwrap();
        let key = cert.serialize_private_key_der();
        let cert = cert.serialize_der().unwrap();
        let sim_cfg = config.local_simulation.clone();

        let save = dirs.data_local_dir().join("default.save");
        info!("using save file {}", save.display());
        std::fs::create_dir_all(save.parent().unwrap()).unwrap();
        let save = match Save::open(&save, config.local_simulation.chunk_size) {
            Ok(x) => x,
            Err(e) => {
                error!("couldn't open save: {}", e);
                return;
            }
        };

        std::thread::spawn(move || {
            let span = error_span!("server");
            let _guard = span.enter();
            if let Err(e) = server::run(
                server::NetParams {
                    certificate_chain: vec![rustls::Certificate(cert)],
                    private_key: rustls::PrivateKey(key),
                    socket,
                },
                sim_cfg,
                save,
            ) {
                eprintln!("{e:#}");
                std::process::exit(1);
            }
        });
    }
    let config = Arc::new(config);

    // Create the OS window
    let window = graphics::EarlyWindow::new();
    // Initialize Vulkan with the extensions needed to render to the window
    let core = Arc::new(graphics::Core::new(window.required_extensions()));

    // Kick off networking
    let net = net::spawn(config.clone());

    // Finish creating the window, including the Vulkan resources used to render to it
    let window = graphics::Window::new(window, core.clone(), config, metrics, net);

    // Initialize widely-shared graphics resources
    let gfx = Arc::new(
        graphics::Base::new(
            core,
            Some(dirs.cache_dir().join("pipeline_cache")),
            &[khr::swapchain::NAME],
            |physical, queue_family| window.supports(physical, queue_family),
        )
        .unwrap(),
    );

    // Run the window's event loop
    window.run(gfx);
}
