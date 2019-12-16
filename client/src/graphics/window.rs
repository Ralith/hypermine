use std::ffi::CStr;
use std::sync::Arc;

use ash::{extensions::khr, version::DeviceV1_0, vk};
use lahar::DedicatedImage;
use tracing::info;
use winit::{
    event::{Event, WindowEvent},
    event_loop::{ControlFlow, EventLoop},
    window::{Window as WinitWindow, WindowBuilder},
};

use super::{Base, Core, Draw};

pub struct EarlyWindow {
    event_loop: EventLoop<()>,
    window: WinitWindow,
}

impl EarlyWindow {
    pub fn new() -> Self {
        let event_loop = EventLoop::new();
        let window = WindowBuilder::new()
            .with_title("hypermine")
            .build(&event_loop)
            .unwrap();
        Self { event_loop, window }
    }

    pub fn required_extension(&self) -> &'static CStr {
        ash_window::enumerate_required_extension(&self.window)
    }
}

pub struct Window {
    _core: Arc<Core>,
    _window: WinitWindow,
    event_loop: Option<EventLoop<()>>,
    surface_fn: khr::Surface,
    surface: vk::SurfaceKHR,
    swapchain: Option<SwapchainMgr>,
    draw: Option<Draw>,
}

impl Window {
    pub fn new(early: EarlyWindow, core: Arc<Core>) -> Self {
        let surface = unsafe {
            ash_window::create_surface(&core.entry, &core.instance, &early.window, None).unwrap()
        };
        let surface_fn = khr::Surface::new(&core.entry, &core.instance);

        Self {
            _core: core,
            _window: early.window,
            event_loop: Some(early.event_loop),
            surface,
            surface_fn,
            swapchain: None,
            draw: None,
        }
    }

    pub fn supports(&self, physical: vk::PhysicalDevice, queue_family_index: u32) -> bool {
        unsafe {
            self.surface_fn.get_physical_device_surface_support(
                physical,
                queue_family_index,
                self.surface,
            )
        }
    }

    pub fn run(mut self, gfx: Arc<Base>) -> ! {
        self.swapchain = Some(SwapchainMgr::new(&self, gfx.clone()));
        self.draw = Some(Draw::new(gfx));
        self.event_loop
            .take()
            .unwrap()
            .run(move |event, _, control_flow| match event {
                Event::EventsCleared => {
                    self.draw();
                }
                Event::WindowEvent {
                    event: WindowEvent::CloseRequested,
                    ..
                } => {
                    info!("exiting due to closed window");
                    *control_flow = ControlFlow::Exit;
                }
                _ => {}
            });
    }

    fn draw(&mut self) {
        let swapchain = self.swapchain.as_mut().unwrap();
        let draw = self.draw.as_mut().unwrap();
        let mut needs_update;
        unsafe {
            draw.wait();
            let frame_id = loop {
                match swapchain.acquire_next_image(draw.image_acquired()) {
                    Ok((idx, sub)) => {
                        needs_update = sub;
                        break idx;
                    }
                    Err(vk::Result::ERROR_OUT_OF_DATE_KHR) => {
                        draw.wait_idle();
                        swapchain.update(&self.surface_fn, self.surface);
                    }
                    Err(e) => {
                        panic!("acquire_next_image: {}", e);
                    }
                }
            };
            let frame = &swapchain.state.frames[frame_id as usize];
            draw.draw(frame.buffer, swapchain.state.extent, frame.present);
            match swapchain.queue_present(frame_id) {
                Ok(false) => {}
                Ok(true) | Err(vk::Result::ERROR_OUT_OF_DATE_KHR) => {
                    needs_update = true;
                }
                Err(e) => panic!("queue_present {}", e),
            };
            if needs_update {
                draw.wait_idle();
                swapchain.update(&self.surface_fn, self.surface);
            }
        }
    }
}

impl Drop for Window {
    fn drop(&mut self) {
        self.draw.take();
        self.swapchain.take();
        unsafe {
            self.surface_fn.destroy_surface(self.surface, None);
        }
    }
}

struct SwapchainMgr {
    state: SwapchainState,
    format: vk::SurfaceFormatKHR,
}

impl SwapchainMgr {
    fn new(window: &Window, gfx: Arc<Base>) -> Self {
        let device = &*gfx.device;
        let swapchain_fn = khr::Swapchain::new(&gfx.core.instance, &*device);
        let surface_formats = unsafe {
            window
                .surface_fn
                .get_physical_device_surface_formats(gfx.physical, window.surface)
                .unwrap()
        };
        let desired_format = vk::SurfaceFormatKHR {
            format: super::base::COLOR_FORMAT,
            color_space: vk::ColorSpaceKHR::SRGB_NONLINEAR,
        };
        if (surface_formats.len() != 1
            || (surface_formats[0].format != vk::Format::UNDEFINED
                || surface_formats[0].color_space != desired_format.color_space))
            && surface_formats.iter().all(|x| {
                x.format != desired_format.format || x.color_space != desired_format.color_space
            })
        {
            panic!("no suitable surface format: {:?}", surface_formats);
        }

        Self {
            state: unsafe {
                SwapchainState::new(
                    &window.surface_fn,
                    swapchain_fn,
                    gfx,
                    window.surface,
                    desired_format,
                    vk::SwapchainKHR::null(),
                )
            },
            format: desired_format,
        }
    }

    /// Recreate the swapchain based on the window's current capabilities
    ///
    /// # Safety
    /// - There must be no operations scheduled that access the current swapchain
    unsafe fn update(&mut self, surface_fn: &khr::Surface, surface: vk::SurfaceKHR) {
        self.state = SwapchainState::new(
            surface_fn,
            self.state.swapchain_fn.clone(),
            self.state.gfx.clone(),
            surface,
            self.format,
            self.state.handle,
        );
    }

    unsafe fn acquire_next_image(&self, signal: vk::Semaphore) -> Result<(u32, bool), vk::Result> {
        self.state.swapchain_fn.acquire_next_image(
            self.state.handle,
            std::u64::MAX,
            signal,
            vk::Fence::null(),
        )
    }

    unsafe fn queue_present(&self, index: u32) -> Result<bool, vk::Result> {
        self.state.swapchain_fn.queue_present(
            self.state.gfx.queue,
            &vk::PresentInfoKHR::builder()
                .wait_semaphores(&[self.state.frames[index as usize].present])
                .swapchains(&[self.state.handle])
                .image_indices(&[index]),
        )
    }
}

struct SwapchainState {
    gfx: Arc<Base>,
    swapchain_fn: khr::Swapchain,
    extent: vk::Extent2D,
    handle: vk::SwapchainKHR,
    frames: Vec<Frame>,
}

impl SwapchainState {
    unsafe fn new(
        surface_fn: &khr::Surface,
        swapchain_fn: khr::Swapchain,
        gfx: Arc<Base>,
        surface: vk::SurfaceKHR,
        format: vk::SurfaceFormatKHR,
        old: vk::SwapchainKHR,
    ) -> Self {
        let device = &*gfx.device;

        let surface_capabilities = surface_fn
            .get_physical_device_surface_capabilities(gfx.physical, surface)
            .unwrap();
        let extent = match surface_capabilities.current_extent.width {
            std::u32::MAX => vk::Extent2D {
                width: 1280,
                height: 1024,
            },
            _ => surface_capabilities.current_extent,
        };
        let pre_transform = if surface_capabilities
            .supported_transforms
            .contains(vk::SurfaceTransformFlagsKHR::IDENTITY)
        {
            vk::SurfaceTransformFlagsKHR::IDENTITY
        } else {
            surface_capabilities.current_transform
        };
        let present_modes = surface_fn
            .get_physical_device_surface_present_modes(gfx.physical, surface)
            .unwrap();
        let present_mode = present_modes
            .iter()
            .cloned()
            .find(|&mode| mode == vk::PresentModeKHR::MAILBOX)
            .unwrap_or(vk::PresentModeKHR::FIFO);

        let image_count = if surface_capabilities.max_image_count > 0 {
            surface_capabilities
                .max_image_count
                .min(surface_capabilities.min_image_count + 1)
        } else {
            surface_capabilities.min_image_count + 1
        };

        let handle = swapchain_fn
            .create_swapchain(
                &vk::SwapchainCreateInfoKHR::builder()
                    .surface(surface)
                    .min_image_count(image_count)
                    .image_color_space(format.color_space)
                    .image_format(format.format)
                    .image_extent(extent)
                    .image_usage(vk::ImageUsageFlags::COLOR_ATTACHMENT)
                    .image_sharing_mode(vk::SharingMode::EXCLUSIVE)
                    .pre_transform(pre_transform)
                    .composite_alpha(vk::CompositeAlphaFlagsKHR::OPAQUE)
                    .present_mode(present_mode)
                    .clipped(true)
                    .image_array_layers(1)
                    .old_swapchain(old),
                None,
            )
            .unwrap();
        if old != vk::SwapchainKHR::null() {
            swapchain_fn.destroy_swapchain(old, None);
        }

        let frames = swapchain_fn
            .get_swapchain_images(handle)
            .unwrap()
            .into_iter()
            .map(|image| {
                let view = device
                    .create_image_view(
                        &vk::ImageViewCreateInfo::builder()
                            .view_type(vk::ImageViewType::TYPE_2D)
                            .format(format.format)
                            .subresource_range(vk::ImageSubresourceRange {
                                aspect_mask: vk::ImageAspectFlags::COLOR,
                                base_mip_level: 0,
                                level_count: 1,
                                base_array_layer: 0,
                                layer_count: 1,
                            })
                            .image(image),
                        None,
                    )
                    .unwrap();
                let depth = DedicatedImage::new(
                    device,
                    &gfx.memory_properties,
                    &vk::ImageCreateInfo::builder()
                        .image_type(vk::ImageType::TYPE_2D)
                        .format(vk::Format::D32_SFLOAT)
                        .extent(vk::Extent3D {
                            width: extent.width,
                            height: extent.height,
                            depth: 1,
                        })
                        .mip_levels(1)
                        .array_layers(1)
                        .samples(vk::SampleCountFlags::TYPE_1)
                        .usage(vk::ImageUsageFlags::DEPTH_STENCIL_ATTACHMENT),
                );
                let depth_view = device
                    .create_image_view(
                        &vk::ImageViewCreateInfo::builder()
                            .image(depth.handle)
                            .view_type(vk::ImageViewType::TYPE_2D_ARRAY)
                            .format(vk::Format::D32_SFLOAT)
                            .subresource_range(vk::ImageSubresourceRange {
                                aspect_mask: vk::ImageAspectFlags::DEPTH,
                                base_mip_level: 0,
                                level_count: 1,
                                base_array_layer: 0,
                                layer_count: 1,
                            }),
                        None,
                    )
                    .unwrap();
                Frame {
                    view,
                    depth,
                    depth_view,
                    buffer: device
                        .create_framebuffer(
                            &vk::FramebufferCreateInfo::builder()
                                .render_pass(gfx.render_pass)
                                .attachments(&[view, depth_view])
                                .width(extent.width)
                                .height(extent.height)
                                .layers(1),
                            None,
                        )
                        .unwrap(),
                    present: device.create_semaphore(&Default::default(), None).unwrap(),
                }
            })
            .collect();

        Self {
            swapchain_fn,
            gfx,
            extent,
            handle,
            frames,
        }
    }
}

impl Drop for SwapchainState {
    fn drop(&mut self) {
        let device = &*self.gfx.device;
        unsafe {
            for frame in &mut self.frames {
                device.destroy_framebuffer(frame.buffer, None);
                device.destroy_image_view(frame.depth_view, None);
                device.destroy_image_view(frame.view, None);
                frame.depth.destroy(device);
                device.destroy_semaphore(frame.present, None);
            }
            self.swapchain_fn.destroy_swapchain(self.handle, None);
        }
    }
}

struct Frame {
    view: vk::ImageView,
    depth: DedicatedImage,
    depth_view: vk::ImageView,
    buffer: vk::Framebuffer,
    present: vk::Semaphore,
}
