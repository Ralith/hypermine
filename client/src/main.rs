use std::{
    net::{SocketAddr, UdpSocket},
    sync::Arc,
};

use client::{graphics, metrics, net, Config};
use quinn::rustls::pki_types::{CertificateDer, PrivatePkcs8KeyDer};
use save::Save;

use ash::khr;
use tracing::{error, error_span, info};
use winit::{
    application::ApplicationHandler,
    event_loop::{ActiveEventLoop, EventLoop},
};

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

        let certified_key = rcgen::generate_simple_self_signed(vec!["localhost".into()]).unwrap();
        let key = certified_key.key_pair.serialize_der();
        let cert = certified_key.cert.der().to_vec();
        let sim_cfg = config.local_simulation.clone();

        let save = dirs.data_local_dir().join(&config.save);
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
                    certificate_chain: vec![CertificateDer::from(cert)],
                    private_key: PrivatePkcs8KeyDer::from(key).into(),
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
    let mut app = App {
        config: Arc::new(config),
        dirs,
        metrics,
        window: None,
    };

    let event_loop = EventLoop::new().unwrap();
    event_loop.set_control_flow(winit::event_loop::ControlFlow::Poll);
    event_loop.run_app(&mut app).unwrap();
}

struct App {
    config: Arc<Config>,
    dirs: directories::ProjectDirs,
    metrics: Arc<metrics::Recorder>,
    window: Option<graphics::Window>,
}

impl ApplicationHandler for App {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        // Create the OS window
        let window = graphics::EarlyWindow::new(event_loop);
        // Initialize Vulkan with the extensions needed to render to the window
        let core = Arc::new(graphics::Core::new(window.required_extensions()));

        // Kick off networking
        let net = net::spawn(self.config.clone());

        // Finish creating the window, including the Vulkan resources used to render to it
        let mut window = graphics::Window::new(window, core.clone(), self.config.clone(), net);

        // Initialize widely-shared graphics resources
        let gfx = Arc::new(
            graphics::Base::new(
                core,
                Some(self.dirs.cache_dir().join("pipeline_cache")),
                &[khr::swapchain::NAME],
                |physical, queue_family| window.supports(physical, queue_family),
            )
            .unwrap(),
        );
        window.init_rendering(gfx.clone());
        self.window = Some(window);
    }

    fn suspended(&mut self, _event_loop: &ActiveEventLoop) {
        self.window = None;
    }

    fn window_event(
        &mut self,
        event_loop: &ActiveEventLoop,
        _window_id: winit::window::WindowId,
        event: winit::event::WindowEvent,
    ) {
        let Some(window) = self.window.as_mut() else {
            return;
        };
        window.handle_event(event, event_loop);
    }

    fn device_event(
        &mut self,
        _event_loop: &ActiveEventLoop,
        _device_id: winit::event::DeviceId,
        event: winit::event::DeviceEvent,
    ) {
        let Some(window) = self.window.as_mut() else {
            return;
        };
        window.handle_device_event(event);
    }

    fn about_to_wait(&mut self, _event_loop: &ActiveEventLoop) {
        let Some(window) = self.window.as_mut() else {
            return;
        };
        window.window.request_redraw();
    }

    fn exiting(&mut self, _event_loop: &ActiveEventLoop) {
        self.metrics.report();
    }
}
