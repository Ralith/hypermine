use std::sync::Arc;

use ash::{version::DeviceV1_0, vk};

use super::Base;

pub struct Draw {
    gfx: Arc<Base>,
    cmd_pool: vk::CommandPool,
    states: Vec<State>,
    next_state: usize,
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
            let states = cmds
                .into_iter()
                .map(|cmd| State {
                    cmd,
                    image_acquired: device.create_semaphore(&Default::default(), None).unwrap(),
                    fence: device
                        .create_fence(
                            &vk::FenceCreateInfo::builder().flags(vk::FenceCreateFlags::SIGNALED),
                            None,
                        )
                        .unwrap(),
                })
                .collect();
            Self {
                gfx,
                cmd_pool,
                states,
                next_state: 0,
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
        let state = &self.states[self.next_state];
        device.reset_fences(&[state.fence]).unwrap();
        let cmd = state.cmd;

        device
            .begin_command_buffer(
                cmd,
                &vk::CommandBufferBeginInfo::builder()
                    .flags(vk::CommandBufferUsageFlags::ONE_TIME_SUBMIT),
            )
            .unwrap();

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
                            float32: [0.0, 0.0, 1.0, 0.0],
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
        device.cmd_end_render_pass(cmd);

        device.end_command_buffer(cmd).unwrap();

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
            for state in &self.states {
                device.wait_for_fences(&[state.fence], true, !0).unwrap();
                device.destroy_semaphore(state.image_acquired, None);
                device.destroy_fence(state.fence, None);
            }
            device.destroy_command_pool(self.cmd_pool, None);
        }
    }
}

struct State {
    image_acquired: vk::Semaphore,
    fence: vk::Fence,
    cmd: vk::CommandBuffer,
}
