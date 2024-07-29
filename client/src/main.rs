use std::{sync::Arc, thread};

use client::{graphics, metrics, net, Config};
use common::proto;
use save::Save;

use ash::khr;
use server::Server;
use tracing::{debug, error, error_span, info, Instrument};
use winit::{
    application::ApplicationHandler,
    event_loop::{ActiveEventLoop, EventLoop},
};

fn main() {
    // Set up logging
    common::init_tracing();
    let metrics = crate::metrics::init();

    let dirs = directories::ProjectDirs::from("", "", "hypermine").unwrap();
    let config = Arc::new(Config::load(&dirs));

    let net = match config.server {
        None => {
            // spawn an in-process server
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

            let mut server = match Server::new(None, sim_cfg, save) {
                Ok(server) => server,
                Err(e) => {
                    eprintln!("{e:#}");
                    std::process::exit(1);
                }
            };

            let (handle, backend) = server::Handle::loopback();
            let name = (*config.name).into();

            thread::spawn(move || {
                let runtime = tokio::runtime::Builder::new_current_thread()
                    .enable_time()
                    .build()
                    .unwrap();
                let _guard = runtime.enter();
                server
                    .connect(proto::ClientHello { name }, backend)
                    .unwrap();
                runtime.block_on(server.run().instrument(error_span!("server")));
                debug!("server thread terminated");
            });

            handle
        }
        Some(_) => net::spawn(config.clone()),
    };
    let mut app = App {
        config,
        dirs,
        metrics,
        window: None,
        net: Some(net),
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
    net: Option<server::Handle>,
}

impl ApplicationHandler for App {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        // Create the OS window
        let window = graphics::EarlyWindow::new(event_loop);
        // Initialize Vulkan with the extensions needed to render to the window
        let core = Arc::new(graphics::Core::new(window.required_extensions()));

        // Finish creating the window, including the Vulkan resources used to render to it
        let mut window = graphics::Window::new(
            window,
            core.clone(),
            self.config.clone(),
            self.net.take().unwrap(),
        );

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
