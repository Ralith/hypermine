use std::sync::Arc;
use std::time::Instant;

use ash::{version::DeviceV1_0, vk};
use lahar::Staged;

use super::{Base, Sky};

pub struct Draw {
    gfx: Arc<Base>,
    cmd_pool: vk::CommandPool,
    states: Vec<State>,
    next_state: usize,
    epoch: Instant,
    common_pipeline_layout: vk::PipelineLayout,
    common_descriptor_pool: vk::DescriptorPool,
    sky: Sky,

    // Reusable storage for barriers for resources that will be accessed in the next frame
    image_barriers: Vec<vk::ImageMemoryBarrier>,
    buffer_barriers: Vec<vk::BufferMemoryBarrier>,
}

/// Maximum number of simultaneous frames in flight
const PIPELINE_DEPTH: u32 = 2;

impl Draw {
    pub fn new(gfx: Arc<Base>) -> Self {
        let device = &*gfx.device;
        unsafe {
            let cmd_pool = device
                .create_command_pool(
                    &vk::CommandPoolCreateInfo::builder()
                        .queue_family_index(gfx.queue_family)
                        .flags(vk::CommandPoolCreateFlags::RESET_COMMAND_BUFFER),
                    None,
                )
                .unwrap();
            let cmds = device
                .allocate_command_buffers(
                    &vk::CommandBufferAllocateInfo::builder()
                        .command_pool(cmd_pool)
                        .command_buffer_count(PIPELINE_DEPTH),
                )
                .unwrap();

            let common_pipeline_layout = device
                .create_pipeline_layout(
                    &vk::PipelineLayoutCreateInfo::builder().set_layouts(&[gfx.common_layout]),
                    None,
                )
                .unwrap();

            let common_descriptor_pool = device
                .create_descriptor_pool(
                    &vk::DescriptorPoolCreateInfo::builder()
                        .max_sets(PIPELINE_DEPTH)
                        .pool_sizes(&[vk::DescriptorPoolSize {
                            ty: vk::DescriptorType::UNIFORM_BUFFER,
                            descriptor_count: PIPELINE_DEPTH,
                        }]),
                    None,
                )
                .unwrap();

            let common_ds = device
                .allocate_descriptor_sets(
                    &vk::DescriptorSetAllocateInfo::builder()
                        .descriptor_pool(common_descriptor_pool)
                        .set_layouts(&vec![gfx.common_layout; PIPELINE_DEPTH as usize]),
                )
                .unwrap();

            let states = cmds
                .into_iter()
                .zip(common_ds)
                .map(|(cmd, common_ds)| {
                    let uniforms = Staged::new(
                        device,
                        &gfx.memory_properties,
                        vk::BufferUsageFlags::UNIFORM_BUFFER,
                    );
                    device.update_descriptor_sets(
                        &[vk::WriteDescriptorSet::builder()
                            .dst_set(common_ds)
                            .descriptor_type(vk::DescriptorType::UNIFORM_BUFFER)
                            .buffer_info(&[vk::DescriptorBufferInfo {
                                buffer: uniforms.buffer(),
                                offset: 0,
                                range: vk::WHOLE_SIZE,
                            }])
                            .build()],
                        &[],
                    );
                    let x = State {
                        cmd,
                        common_ds,
                        image_acquired: device.create_semaphore(&Default::default(), None).unwrap(),
                        fence: device
                            .create_fence(
                                &vk::FenceCreateInfo::builder()
                                    .flags(vk::FenceCreateFlags::SIGNALED),
                                None,
                            )
                            .unwrap(),
                        uniforms,
                    };
                    gfx.set_name(x.cmd, cstr!("frame"));
                    gfx.set_name(x.image_acquired, cstr!("image acquired"));
                    gfx.set_name(x.fence, cstr!("render complete"));
                    gfx.set_name(x.uniforms.buffer(), cstr!("uniforms"));
                    x
                })
                .collect();

            let sky = Sky::new(gfx.clone());

            Self {
                gfx,
                cmd_pool,
                states,
                next_state: 0,
                epoch: Instant::now(),
                common_pipeline_layout,
                common_descriptor_pool,
                sky,

                buffer_barriers: Vec::new(),
                image_barriers: Vec::new(),
            }
        }
    }

    /// Call before signaling image_acquired
    pub unsafe fn wait(&self) {
        let device = &*self.gfx.device;
        let fence = self.states[self.next_state].fence;
        device.wait_for_fences(&[fence], true, !0).unwrap();
    }

    /// Don't signal until after `wait`ing; call before `draw`
    pub fn image_acquired(&self) -> vk::Semaphore {
        self.states[self.next_state].image_acquired
    }

    /// Call after arranging for `image_acquired` to be signaled
    pub unsafe fn draw(
        &mut self,
        framebuffer: vk::Framebuffer,
        extent: vk::Extent2D,
        present: vk::Semaphore,
    ) {
        let device = &*self.gfx.device;
        let state = &mut self.states[self.next_state];
        device.reset_fences(&[state.fence]).unwrap();
        let cmd = state.cmd;

        device
            .begin_command_buffer(
                cmd,
                &vk::CommandBufferBeginInfo::builder()
                    .flags(vk::CommandBufferUsageFlags::ONE_TIME_SUBMIT),
            )
            .unwrap();

        // Schedule transfer of uniform data. Note that we defer actually preparing the data to just
        // before submitting the command buffer so time-sensitive values can be set with minimum
        // latency.
        state.uniforms.record_transfer(device, cmd);
        self.buffer_barriers.push(
            vk::BufferMemoryBarrier::builder()
                .src_access_mask(vk::AccessFlags::TRANSFER_WRITE)
                .dst_access_mask(vk::AccessFlags::SHADER_READ)
                .buffer(state.uniforms.buffer())
                .size(vk::WHOLE_SIZE)
                .build(),
        );

        // Ensure reads of just-transferred memory wait until it's ready
        device.cmd_pipeline_barrier(
            cmd,
            vk::PipelineStageFlags::TRANSFER,
            vk::PipelineStageFlags::VERTEX_SHADER | vk::PipelineStageFlags::FRAGMENT_SHADER,
            vk::DependencyFlags::default(),
            &[],
            &self.buffer_barriers,
            &self.image_barriers,
        );
        self.buffer_barriers.clear();
        self.image_barriers.clear();

        device.cmd_begin_render_pass(
            cmd,
            &vk::RenderPassBeginInfo::builder()
                .render_pass(self.gfx.render_pass)
                .framebuffer(framebuffer)
                .render_area(vk::Rect2D {
                    offset: vk::Offset2D::default(),
                    extent,
                })
                .clear_values(&[
                    vk::ClearValue {
                        color: vk::ClearColorValue {
                            float32: [0.0, 0.0, 0.0, 0.0],
                        },
                    },
                    vk::ClearValue {
                        depth_stencil: vk::ClearDepthStencilValue {
                            depth: 0.0,
                            stencil: 0,
                        },
                    },
                ]),
            vk::SubpassContents::INLINE,
        );

        // Set up common dynamic state
        let viewports = [vk::Viewport {
            x: 0.0,
            y: 0.0,
            width: extent.width as f32,
            height: extent.height as f32,
            min_depth: 0.0,
            max_depth: 1.0,
        }];
        let scissors = [vk::Rect2D {
            offset: vk::Offset2D { x: 0, y: 0 },
            extent: vk::Extent2D {
                width: extent.width as u32,
                height: extent.height as u32,
            },
        }];
        device.cmd_set_viewport(cmd, 0, &viewports);
        device.cmd_set_scissor(cmd, 0, &scissors);
        device.cmd_bind_descriptor_sets(
            cmd,
            vk::PipelineBindPoint::GRAPHICS,
            self.common_pipeline_layout,
            0,
            &[state.common_ds],
            &[],
        );

        // Record the actual rendering commands
        self.sky.draw(cmd);

        device.cmd_end_render_pass(cmd);
        device.end_command_buffer(cmd).unwrap();

        // Specify the uniform data before submitting the command to transfer it
        state.uniforms.write(
            device,
            Uniforms {
                projection: na::Projective3::identity(),
                time: self.epoch.elapsed().as_secs_f32().fract(),
            },
        );

        // Submit the commands to the GPU
        device
            .queue_submit(
                self.gfx.queue,
                &[vk::SubmitInfo::builder()
                    .command_buffers(&[cmd])
                    .wait_semaphores(&[state.image_acquired])
                    .wait_dst_stage_mask(&[vk::PipelineStageFlags::COLOR_ATTACHMENT_OUTPUT])
                    .signal_semaphores(&[present])
                    .build()],
                state.fence,
            )
            .unwrap();

        self.next_state = (self.next_state + 1) % self.states.len();
    }

    /// Wait for all drawing to complete
    pub fn wait_idle(&self) {
        let device = &*self.gfx.device;
        for state in &self.states {
            unsafe {
                device.wait_for_fences(&[state.fence], true, !0).unwrap();
            }
        }
    }
}

impl Drop for Draw {
    fn drop(&mut self) {
        let device = &*self.gfx.device;
        unsafe {
            for state in &mut self.states {
                device.wait_for_fences(&[state.fence], true, !0).unwrap();
                device.destroy_semaphore(state.image_acquired, None);
                device.destroy_fence(state.fence, None);
                state.uniforms.destroy(device);
            }
            device.destroy_command_pool(self.cmd_pool, None);
            device.destroy_descriptor_pool(self.common_descriptor_pool, None);
            device.destroy_pipeline_layout(self.common_pipeline_layout, None);
        }
    }
}

struct State {
    image_acquired: vk::Semaphore,
    fence: vk::Fence,
    cmd: vk::CommandBuffer,
    common_ds: vk::DescriptorSet,
    uniforms: Staged<Uniforms>,
}

#[repr(C)]
#[derive(Copy, Clone)]
pub struct Uniforms {
    projection: na::Projective3<f32>,
    time: f32,
}
