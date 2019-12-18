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

/// OS window
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

    /// Identify the Vulkan extension needed to render to this window
    pub fn required_extension(&self) -> &'static CStr {
        ash_window::enumerate_required_extension(&self.window)
    }
}

/// OS window + rendering handles
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
    /// Finish constructing a window
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

    /// Determine whether this window can be rendered to from a particular device and queue family
    pub fn supports(&self, physical: vk::PhysicalDevice, queue_family_index: u32) -> bool {
        unsafe {
            self.surface_fn.get_physical_device_surface_support(
                physical,
                queue_family_index,
                self.surface,
            )
        }
    }

    /// Run the event loop until process exit
    pub fn run(mut self, gfx: Arc<Base>) -> ! {
        // Allocate the presentable images we'll be rendering to
        self.swapchain = Some(SwapchainMgr::new(&self, gfx.clone()));
        // Construct the core rendering object
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

    /// Draw a new frame
    fn draw(&mut self) {
        let swapchain = self.swapchain.as_mut().unwrap();
        let draw = self.draw.as_mut().unwrap();
        let mut needs_update;
        unsafe {
            // Wait for a frame's worth of rendering resources to become available
            draw.wait();
            // Get the index of the swapchain image we'll render to
            let frame_id = loop {
                match swapchain.acquire_next_image(draw.image_acquired()) {
                    Ok((idx, sub)) => {
                        needs_update = sub;
                        break idx;
                    }
                    // New swapchain needed immediately (usually due to resize)
                    Err(vk::Result::ERROR_OUT_OF_DATE_KHR) => {
                        // Wait for in-flight frames to complete so we don't have a use-after-free
                        draw.wait_idle();
                        // Recreate the swapchain
                        swapchain.update(&self.surface_fn, self.surface);
                    }
                    Err(e) => {
                        panic!("acquire_next_image: {}", e);
                    }
                }
            };
            let frame = &swapchain.state.frames[frame_id as usize];
            // Render the frame
            draw.draw(frame.buffer, swapchain.state.extent, frame.present);
            // Submit the frame to be presented on the window
            match swapchain.queue_present(frame_id) {
                Ok(false) => {}
                Ok(true) | Err(vk::Result::ERROR_OUT_OF_DATE_KHR) => {
                    needs_update = true;
                }
                Err(e) => panic!("queue_present: {}", e),
            };
            // New swapchain needed (usually due to resize)
            if needs_update {
                // Wait for in-flight frames to complete so we don't have a use-after-free
                draw.wait_idle();
                // Recreate the swapchain
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
    /// Construct a swapchain manager for a certain window
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

    /// Get the index of the next frame to use
    unsafe fn acquire_next_image(&self, signal: vk::Semaphore) -> Result<(u32, bool), vk::Result> {
        self.state.swapchain_fn.acquire_next_image(
            self.state.handle,
            std::u64::MAX,
            signal,
            vk::Fence::null(),
        )
    }

    /// Present a frame on the window
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

/// Data that's replaced when the swapchain is updated
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
                gfx.set_name(view, cstr!("swapchain"));
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
                gfx.set_name(depth.handle, cstr!("depth"));
                gfx.set_name(depth.memory, cstr!("depth"));
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
                gfx.set_name(depth_view, cstr!("depth"));
                let present = device.create_semaphore(&Default::default(), None).unwrap();
                gfx.set_name(present, cstr!("present"));
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
                    present,
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
    /// Image view for an entire swapchain image
    view: vk::ImageView,
    /// Depth buffer to use when rendering to this image
    depth: DedicatedImage,
    /// View thereof
    depth_view: vk::ImageView,
    /// Framebuffer referencing `view` and `depth_view`
    buffer: vk::Framebuffer,
    /// Semaphore used to ensure the frame isn't presented until rendering completes
    present: vk::Semaphore,
}
