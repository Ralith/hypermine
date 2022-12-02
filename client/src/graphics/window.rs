use std::sync::Arc;
use std::time::Instant;
use std::{f32, os::raw::c_char};

use ash::{extensions::khr, vk};
use lahar::DedicatedImage;
use raw_window_handle::{HasRawDisplayHandle, HasRawWindowHandle};
use tracing::info;
use winit::{
    dpi::PhysicalSize,
    event::{
        DeviceEvent, ElementState, Event, KeyboardInput, MouseButton, VirtualKeyCode, WindowEvent,
    },
    event_loop::{ControlFlow, EventLoop},
    window::{CursorGrabMode, Window as WinitWindow, WindowBuilder},
};

use super::{Base, Core, Draw, Frustum};
use crate::{Config, Sim};

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
    pub fn required_extensions(&self) -> &'static [*const c_char] {
        ash_window::enumerate_required_extensions(self.event_loop.raw_display_handle())
            .expect("unsupported platform")
    }
}

/// OS window + rendering handles
pub struct Window {
    _core: Arc<Core>,
    window: WinitWindow,
    config: Arc<Config>,
    metrics: Arc<crate::metrics::Recorder>,
    event_loop: Option<EventLoop<()>>,
    surface_fn: khr::Surface,
    surface: vk::SurfaceKHR,
    swapchain: Option<SwapchainMgr>,
    swapchain_needs_update: bool,
    draw: Option<Draw>,
    sim: Sim,
}

impl Window {
    /// Finish constructing a window
    pub fn new(
        early: EarlyWindow,
        core: Arc<Core>,
        config: Arc<Config>,
        metrics: Arc<crate::metrics::Recorder>,
        sim: Sim,
    ) -> Self {
        let surface = unsafe {
            ash_window::create_surface(
                &core.entry,
                &core.instance,
                early.window.raw_display_handle(),
                early.window.raw_window_handle(),
                None,
            )
            .unwrap()
        };
        let surface_fn = khr::Surface::new(&core.entry, &core.instance);

        Self {
            _core: core,
            window: early.window,
            config,
            metrics,
            event_loop: Some(early.event_loop),
            surface,
            surface_fn,
            swapchain: None,
            swapchain_needs_update: false,
            draw: None,
            sim,
        }
    }

    /// Determine whether this window can be rendered to from a particular device and queue family
    pub fn supports(&self, physical: vk::PhysicalDevice, queue_family_index: u32) -> bool {
        unsafe {
            self.surface_fn
                .get_physical_device_surface_support(physical, queue_family_index, self.surface)
                .unwrap()
        }
    }

    /// Run the event loop until process exit
    pub fn run(mut self, gfx: Arc<Base>) -> ! {
        // Allocate the presentable images we'll be rendering to
        self.swapchain = Some(SwapchainMgr::new(
            &self,
            gfx.clone(),
            self.window.inner_size(),
        ));
        // Construct the core rendering object
        self.draw = Some(Draw::new(gfx, self.config.clone()));
        let mut forward = false;
        let mut back = false;
        let mut left = false;
        let mut right = false;
        let mut up = false;
        let mut down = false;
        let mut jump = false;
        let mut jump_queued = false;
        let mut noclip = false;
        let mut break_blocks = false;
        let mut place_blocks = false;
        let mut last_frame = Instant::now();
        let mut mouse_captured = false;
        self.event_loop
            .take()
            .unwrap()
            .run(move |event, _, control_flow| match event {
                Event::MainEventsCleared => {
                    let this_frame = Instant::now();
                    let dt = this_frame - last_frame;
                    let move_direction: na::Vector3<f32> = if noclip {
                        na::Vector3::x() * (right as u8 as f32 - left as u8 as f32)
                            + na::Vector3::y() * (up as u8 as f32 - down as u8 as f32)
                            + na::Vector3::z() * (back as u8 as f32 - forward as u8 as f32)
                    } else {
                        na::Vector3::x() * (right as u8 as f32 - left as u8 as f32)
                            + na::Vector3::z() * (back as u8 as f32 - forward as u8 as f32)
                    };
                    self.sim.velocity(if move_direction.norm_squared() > 1.0 {
                        move_direction.normalize()
                    } else {
                        move_direction
                    });
                    if jump_queued {
                        self.sim.jump();
                        jump_queued = false;
                    }

                    self.sim.set_noclip(noclip);

                    self.sim.keep_breaking_blocks(break_blocks);
                    self.sim.keep_placing_blocks(place_blocks);

                    let had_params = self.sim.params().is_some();

                    self.sim.step(dt);
                    last_frame = this_frame;

                    if !had_params {
                        if let Some(params) = self.sim.params() {
                            self.draw.as_mut().unwrap().configure(params);
                        }
                    }

                    self.draw();
                }
                Event::DeviceEvent { event, .. } => match event {
                    DeviceEvent::MouseMotion { delta } if mouse_captured => {
                        const SENSITIVITY: f32 = 2e-3;
                        self.sim
                            .rotate(delta.0 as f32 * SENSITIVITY, delta.1 as f32 * SENSITIVITY);
                    }
                    _ => {}
                },
                Event::WindowEvent { event, .. } => match event {
                    WindowEvent::Resized(_) => {
                        // Some environments may not emit the vulkan signals that recommend or
                        // require surface reconstruction, so we need to check for messages from the
                        // windowing system too. We defer actually performing the update until
                        // drawing to avoid doing unnecessary work between frames.
                        self.swapchain_needs_update = true;
                    }
                    WindowEvent::CloseRequested => {
                        info!("exiting due to closed window");
                        *control_flow = ControlFlow::Exit;
                    }
                    WindowEvent::MouseInput {
                        button: MouseButton::Left,
                        state: ElementState::Pressed,
                        ..
                    } => {
                        if mouse_captured {
                            self.sim.break_block();
                            break_blocks = true;
                        }
                        let _ = self
                            .window
                            .set_cursor_grab(CursorGrabMode::Confined)
                            .or_else(|_e| self.window.set_cursor_grab(CursorGrabMode::Locked));
                        self.window.set_cursor_visible(false);
                        mouse_captured = true;
                    }
                    WindowEvent::MouseInput {
                        button: MouseButton::Left,
                        state,
                        ..
                    } => {
                        break_blocks = state == ElementState::Pressed;
                    }
                    WindowEvent::MouseInput {
                        button: MouseButton::Right,
                        state,
                        ..
                    } => {
                        place_blocks = state == ElementState::Pressed;
                        if state == ElementState::Pressed {
                            self.sim.place_block();
                        }
                    }
                    WindowEvent::KeyboardInput {
                        input:
                            KeyboardInput {
                                state,
                                virtual_keycode: Some(key),
                                ..
                            },
                        ..
                    } => match key {
                        VirtualKeyCode::W => {
                            forward = state == ElementState::Pressed;
                        }
                        VirtualKeyCode::A => {
                            left = state == ElementState::Pressed;
                        }
                        VirtualKeyCode::S => {
                            back = state == ElementState::Pressed;
                        }
                        VirtualKeyCode::D => {
                            right = state == ElementState::Pressed;
                        }
                        VirtualKeyCode::R => {
                            up = state == ElementState::Pressed;
                        }
                        VirtualKeyCode::F => {
                            down = state == ElementState::Pressed;
                        }
                        VirtualKeyCode::Space => {
                            if !jump && state == ElementState::Pressed {
                                jump_queued = true;
                            }
                            jump = state == ElementState::Pressed;
                        }
                        VirtualKeyCode::V => {
                            if state == ElementState::Pressed {
                                noclip = !noclip;
                            }
                        }
                        VirtualKeyCode::Escape => {
                            let _ = self.window.set_cursor_grab(CursorGrabMode::None);
                            self.window.set_cursor_visible(true);
                            mouse_captured = false;
                        }
                        _ => {}
                    },
                    WindowEvent::Focused(focused) => {
                        if !focused {
                            let _ = self.window.set_cursor_grab(CursorGrabMode::None);
                            self.window.set_cursor_visible(true);
                            mouse_captured = false;
                        }
                    }
                    _ => {}
                },
                Event::LoopDestroyed => {
                    self.metrics.report();
                }
                _ => {}
            });
    }

    /// Draw a new frame
    fn draw(&mut self) {
        let swapchain = self.swapchain.as_mut().unwrap();
        let draw = self.draw.as_mut().unwrap();
        unsafe {
            // Wait for a frame's worth of rendering resources to become available
            draw.wait();
            // Get the index of the swapchain image we'll render to
            let frame_id = loop {
                // Check whether the window has been resized or similar
                if self.swapchain_needs_update {
                    // Wait for all in-flight frames to complete so we don't have a use-after-free
                    draw.wait_idle();
                    // Recreate the swapchain at a new size (or whatever)
                    swapchain.update(&self.surface_fn, self.surface, self.window.inner_size());
                    self.swapchain_needs_update = false;
                }
                match swapchain.acquire_next_image(draw.image_acquired()) {
                    Ok((idx, suboptimal)) => {
                        self.swapchain_needs_update = suboptimal;
                        break idx;
                    }
                    Err(vk::Result::ERROR_OUT_OF_DATE_KHR) => {
                        self.swapchain_needs_update = true;
                    }
                    Err(e) => {
                        panic!("acquire_next_image: {}", e);
                    }
                }
            };
            let aspect_ratio =
                swapchain.state.extent.width as f32 / swapchain.state.extent.height as f32;
            let frame = &swapchain.state.frames[frame_id as usize];
            let frustum = Frustum::from_vfov(f32::consts::FRAC_PI_4 * 1.2, aspect_ratio);
            // Render the frame
            draw.draw(
                &mut self.sim,
                frame.buffer,
                frame.depth_view,
                swapchain.state.extent,
                frame.present,
                &frustum,
            );
            // Submit the frame to be presented on the window
            match swapchain.queue_present(frame_id) {
                Ok(false) => {}
                Ok(true) | Err(vk::Result::ERROR_OUT_OF_DATE_KHR) => {
                    self.swapchain_needs_update = true;
                }
                Err(e) => panic!("queue_present: {}", e),
            };
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
    fn new(window: &Window, gfx: Arc<Base>, fallback_size: PhysicalSize<u32>) -> Self {
        let device = &*gfx.device;
        let swapchain_fn = khr::Swapchain::new(&gfx.core.instance, device);
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

        let desirable_format = |x: &vk::SurfaceFormatKHR| -> bool {
            x.format == desired_format.format && x.color_space == desired_format.color_space
        };

        if (surface_formats.len() != 1
            || (surface_formats[0].format != vk::Format::UNDEFINED
                || surface_formats[0].color_space != desired_format.color_space))
            && !surface_formats.iter().any(desirable_format)
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
                    fallback_size,
                )
            },
            format: desired_format,
        }
    }

    /// Recreate the swapchain based on the window's current capabilities
    ///
    /// # Safety
    /// - There must be no operations scheduled that access the current swapchain
    unsafe fn update(
        &mut self,
        surface_fn: &khr::Surface,
        surface: vk::SurfaceKHR,
        fallback_size: PhysicalSize<u32>,
    ) {
        self.state = SwapchainState::new(
            surface_fn,
            self.state.swapchain_fn.clone(),
            self.state.gfx.clone(),
            surface,
            self.format,
            self.state.handle,
            fallback_size,
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
        fallback_size: PhysicalSize<u32>,
    ) -> Self {
        let device = &*gfx.device;

        let surface_capabilities = surface_fn
            .get_physical_device_surface_capabilities(gfx.physical, surface)
            .unwrap();
        let extent = match surface_capabilities.current_extent.width {
            // If Vulkan doesn't know, winit probably does. Known to apply at least to Wayland.
            std::u32::MAX => vk::Extent2D {
                width: fallback_size.width,
                height: fallback_size.height,
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
                        .usage(
                            vk::ImageUsageFlags::DEPTH_STENCIL_ATTACHMENT
                                | vk::ImageUsageFlags::INPUT_ATTACHMENT,
                        ),
                );
                gfx.set_name(depth.handle, cstr!("depth"));
                gfx.set_name(depth.memory, cstr!("depth"));
                let depth_view = device
                    .create_image_view(
                        &vk::ImageViewCreateInfo::builder()
                            .image(depth.handle)
                            .view_type(vk::ImageViewType::TYPE_2D)
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
