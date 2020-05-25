use std::sync::Arc;
use std::time::Instant;

use ash::{version::DeviceV1_0, vk};
use lahar::Staged;
use metrics::timing;

use super::{fog, voxels, Base, Fog, Frustum, GltfScene, Meshes, Voxels};
use crate::{sim, Asset, Config, Loader, Sim};
use common::proto::{Character, Position};

/// Manages rendering, independent of what is being rendered to
pub struct Draw {
    gfx: Arc<Base>,
    cfg: Arc<Config>,
    /// Used to allocate the command buffers we render with
    cmd_pool: vk::CommandPool,
    /// Allows accurate frame timing information to be recorded
    timestamp_pool: vk::QueryPool,
    /// State that varies per frame in flight
    states: Vec<State>,
    /// The index of the next element of `states` to use
    next_state: usize,
    /// A reference time
    epoch: Instant,
    /// The lowest common denominator between the interfaces of our graphics pipelines
    ///
    /// Represents e.g. the binding for common uniforms
    common_pipeline_layout: vk::PipelineLayout,
    /// Descriptor pool from which descriptor sets shared between many pipelines are allocated
    common_descriptor_pool: vk::DescriptorPool,

    /// Drives async asset loading
    loader: Loader,

    //
    // Rendering pipelines
    //
    /// Populated after connect, once the voxel configuration is known
    voxels: Option<Voxels>,
    meshes: Meshes,
    fog: Fog,

    /// Reusable storage for barriers that prevent races between image upload and read
    image_barriers: Vec<vk::ImageMemoryBarrier>,
    /// Reusable storage for barriers that prevent races between buffer upload and read
    buffer_barriers: Vec<vk::BufferMemoryBarrier>,

    /// Miscellany
    character_model: Asset<GltfScene>,
}

/// Maximum number of simultaneous frames in flight
const PIPELINE_DEPTH: u32 = 2;
const TIMESTAMPS_PER_FRAME: u32 = 2;

impl Draw {
    pub fn new(gfx: Arc<Base>, cfg: Arc<Config>) -> Self {
        let device = &*gfx.device;
        unsafe {
            // Allocate a command buffer for each frame state
            let cmd_pool = device
                .create_command_pool(
                    &vk::CommandPoolCreateInfo::builder()
                        .queue_family_index(gfx.queue_family)
                        .flags(
                            vk::CommandPoolCreateFlags::RESET_COMMAND_BUFFER
                                | vk::CommandPoolCreateFlags::TRANSIENT,
                        ),
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

            let timestamp_pool = device
                .create_query_pool(
                    &vk::QueryPoolCreateInfo::builder()
                        .query_type(vk::QueryType::TIMESTAMP)
                        .query_count(TIMESTAMPS_PER_FRAME * PIPELINE_DEPTH),
                    None,
                )
                .unwrap();
            gfx.set_name(timestamp_pool, cstr!("timestamp pool"));

            let common_pipeline_layout = device
                .create_pipeline_layout(
                    &vk::PipelineLayoutCreateInfo::builder().set_layouts(&[gfx.common_layout]),
                    None,
                )
                .unwrap();

            // Allocate descriptor sets for data used by all graphics pipelines (e.g. common
            // uniforms)
            let common_descriptor_pool = device
                .create_descriptor_pool(
                    &vk::DescriptorPoolCreateInfo::builder()
                        .max_sets(PIPELINE_DEPTH)
                        .pool_sizes(&[
                            vk::DescriptorPoolSize {
                                ty: vk::DescriptorType::UNIFORM_BUFFER,
                                descriptor_count: PIPELINE_DEPTH,
                            },
                            vk::DescriptorPoolSize {
                                ty: vk::DescriptorType::INPUT_ATTACHMENT,
                                descriptor_count: PIPELINE_DEPTH,
                            },
                        ]),
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

            let mut loader = Loader::new(cfg.clone(), gfx.clone());

            // Construct the per-frame states
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
                            .dst_binding(0)
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
                        used: false,
                        in_flight: false,

                        voxels: None,
                    };
                    gfx.set_name(x.cmd, cstr!("frame"));
                    gfx.set_name(x.image_acquired, cstr!("image acquired"));
                    gfx.set_name(x.fence, cstr!("render complete"));
                    gfx.set_name(x.uniforms.buffer(), cstr!("uniforms"));
                    x
                })
                .collect();

            let meshes = Meshes::new(&gfx, loader.ctx().mesh_ds_layout);

            let fog = Fog::new(&gfx);

            gfx.save_pipeline_cache();

            let character_model = loader.load(
                "character model",
                super::GlbFile {
                    path: "character.glb".into(),
                },
            );

            Self {
                gfx,
                cfg,
                cmd_pool,
                timestamp_pool,
                states,
                next_state: 0,
                epoch: Instant::now(),
                common_pipeline_layout,
                common_descriptor_pool,

                loader,

                voxels: None,
                meshes,
                fog,

                buffer_barriers: Vec::new(),
                image_barriers: Vec::new(),

                character_model,
            }
        }
    }

    /// Called with server-defined world parameters once they're known
    pub fn configure(&mut self, params: &sim::Parameters) {
        let voxels = Voxels::new(
            &self.gfx,
            self.cfg.clone(),
            &mut self.loader,
            u32::from(params.chunk_size),
            PIPELINE_DEPTH,
        );
        for state in &mut self.states {
            state.voxels = Some(voxels::Frame::new(&self.gfx, &voxels));
        }
        self.voxels = Some(voxels);
    }

    /// Waits for a frame's worth of resources to become available for use in rendering a new frame
    ///
    /// Call before signaling the image_acquired semaphore or invoking `draw`.
    pub unsafe fn wait(&mut self) {
        let device = &*self.gfx.device;
        let state = &mut self.states[self.next_state];
        device.wait_for_fences(&[state.fence], true, !0).unwrap();
        state.in_flight = false;
    }

    /// Semaphore that must be signaled when an output framebuffer can be rendered to
    ///
    /// Don't signal until after `wait`ing; call before `draw`
    pub fn image_acquired(&self) -> vk::Semaphore {
        self.states[self.next_state].image_acquired
    }

    /// Submit commands to the GPU to draw a frame
    ///
    /// `framebuffer` must have a color and depth buffer attached and have the dimensions specified
    /// in `extent`. The `present` semaphore is signaled when rendering is complete and the color
    /// image can be presented.
    ///
    /// Submits commands that wait on `image_acquired` before writing to `framebuffer`'s color
    /// attachment.
    pub unsafe fn draw(
        &mut self,
        sim: &mut Sim,
        framebuffer: vk::Framebuffer,
        depth_view: vk::ImageView,
        extent: vk::Extent2D,
        present: vk::Semaphore,
        frustum: &Frustum,
    ) {
        let draw_started = Instant::now();
        let view = sim.view();
        let projection = frustum.projection(0.01);
        let view_projection = projection.matrix() * view.local.try_inverse().unwrap();
        self.loader.drive();

        let device = &*self.gfx.device;
        let state_index = self.next_state;
        let state = &mut self.states[self.next_state];
        let cmd = state.cmd;

        // We're using this state again, so put the fence back in the unsignaled state and compute
        // the next frame to use
        device.reset_fences(&[state.fence]).unwrap();
        self.next_state = (self.next_state + 1) % PIPELINE_DEPTH as usize;

        // Set up framebuffer attachments
        device.update_descriptor_sets(
            &[vk::WriteDescriptorSet::builder()
                .dst_set(state.common_ds)
                .dst_binding(1)
                .descriptor_type(vk::DescriptorType::INPUT_ATTACHMENT)
                .image_info(&[vk::DescriptorImageInfo {
                    sampler: vk::Sampler::null(),
                    image_view: depth_view,
                    image_layout: vk::ImageLayout::DEPTH_STENCIL_READ_ONLY_OPTIMAL,
                }])
                .build()],
            &[],
        );

        // Handle completed queries
        let first_query = state_index as u32 * TIMESTAMPS_PER_FRAME;
        if state.used {
            // Collect timestamps from the last time we drew this frame
            let mut queries = [0u64; TIMESTAMPS_PER_FRAME as usize];
            // `WAIT` is guaranteed not to block here because `Self::draw` is only called after
            // `Self::wait` ensures that the prior instance of this frame is complete.
            device
                .get_query_pool_results(
                    self.timestamp_pool,
                    first_query,
                    TIMESTAMPS_PER_FRAME,
                    &mut queries,
                    vk::QueryResultFlags::TYPE_64 | vk::QueryResultFlags::WAIT,
                )
                .unwrap();
            let nanos = self.gfx.limits.timestamp_period * (queries[1] - queries[0]) as f32;
            timing!("frame.gpu", nanos as u64);
        }

        device
            .begin_command_buffer(
                cmd,
                &vk::CommandBufferBeginInfo::builder()
                    .flags(vk::CommandBufferUsageFlags::ONE_TIME_SUBMIT),
            )
            .unwrap();
        device.cmd_reset_query_pool(cmd, self.timestamp_pool, first_query, TIMESTAMPS_PER_FRAME);
        let mut timestamp_index = first_query;
        device.cmd_write_timestamp(
            cmd,
            vk::PipelineStageFlags::BOTTOM_OF_PIPE,
            self.timestamp_pool,
            timestamp_index,
        );
        timestamp_index += 1;

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

        if let Some(ref mut voxels) = self.voxels {
            voxels.prepare(device, state.voxels.as_mut().unwrap(), sim, cmd, frustum);
        }

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

        // Record the actual rendering commands
        if let Some(ref mut voxels) = self.voxels {
            voxels.draw(
                device,
                &self.loader,
                state.common_ds,
                state.voxels.as_ref().unwrap(),
                cmd,
            );
        }

        if let Some(params) = sim.params() {
            for (node, transform) in sim
                .graph
                .nearby_nodes(&view, f64::from(self.cfg.local_simulation.view_distance))
            {
                for &entity in sim.graph_entities.get(node) {
                    if sim.local_character == Some(entity) {
                        // Don't draw ourself
                        continue;
                    }
                    let pos = sim
                        .world
                        .get::<Position>(entity)
                        .expect("positionless entity in graph");
                    if let Some(character_model) = self.loader.get(self.character_model) {
                        if let Ok(ch) = sim.world.get::<Character>(entity) {
                            let transform = transform
                                * pos.local
                                * na::Matrix4::new_scaling(params.meters_to_absolute)
                                * ch.orientation.to_homogeneous();
                            for mesh in &character_model.0 {
                                self.meshes
                                    .draw(device, state.common_ds, cmd, mesh, &transform);
                            }
                        }
                    }
                }
            }
        }

        device.cmd_next_subpass(cmd, vk::SubpassContents::INLINE);

        self.fog.draw(device, state.common_ds, cmd);

        // Finish up
        device.cmd_end_render_pass(cmd);
        device.cmd_write_timestamp(
            cmd,
            vk::PipelineStageFlags::BOTTOM_OF_PIPE,
            self.timestamp_pool,
            timestamp_index,
        );
        device.end_command_buffer(cmd).unwrap();

        // Specify the uniform data before actually submitting the command to transfer it
        state.uniforms.write(
            device,
            Uniforms {
                view_projection,
                inverse_projection: *projection.inverse().matrix(),
                fog_density: fog::density(self.cfg.local_simulation.view_distance, 1e-3, 5.0),
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
        state.used = true;
        state.in_flight = true;
        timing!("frame.cpu", draw_started.elapsed());
    }

    /// Wait for all drawing to complete
    ///
    /// Useful to e.g. ensure it's safe to deallocate an image that's being rendered to
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
                if state.in_flight {
                    device.wait_for_fences(&[state.fence], true, !0).unwrap();
                    state.in_flight = false;
                }
                device.destroy_semaphore(state.image_acquired, None);
                device.destroy_fence(state.fence, None);
                state.uniforms.destroy(device);
                if let Some(mut voxels) = state.voxels.take() {
                    voxels.destroy(device);
                }
            }
            device.destroy_command_pool(self.cmd_pool, None);
            device.destroy_query_pool(self.timestamp_pool, None);
            device.destroy_descriptor_pool(self.common_descriptor_pool, None);
            device.destroy_pipeline_layout(self.common_pipeline_layout, None);
            self.fog.destroy(device);
            self.meshes.destroy(device);
            if let Some(mut voxels) = self.voxels.take() {
                voxels.destroy(device);
            }
        }
    }
}

struct State {
    /// Semaphore signaled by someone else to indicate that output to the framebuffer can begin
    image_acquired: vk::Semaphore,
    /// Fence signaled when this state is no longer in use
    fence: vk::Fence,
    /// Command buffer we record he frame's rendering onto
    cmd: vk::CommandBuffer,
    /// Descriptor set for graphics-pipeline-independent data
    common_ds: vk::DescriptorSet,
    /// The common uniform buffer
    uniforms: Staged<Uniforms>,
    /// Whether this state has been previously used
    ///
    /// Indicates that e.g. valid timestamps are associated with this query
    used: bool,
    /// Whether this state is currently being accessed by the GPU
    ///
    /// True for the period between `cmd` being submitted and `fence` being waited.
    in_flight: bool,

    // Per-pipeline states
    voxels: Option<voxels::Frame>,
}

/// Data stored in the common uniform buffer
///
/// Alignment and padding must be manually managed to match the std140 ABI as expected by the
/// shaders.
#[repr(C)]
#[derive(Copy, Clone)]
struct Uniforms {
    /// Camera projection matrix
    view_projection: na::Matrix4<f32>,
    inverse_projection: na::Matrix4<f32>,
    fog_density: f32,
    /// Cycles through [0,1) once per second for simple animation effects
    time: f32,
}
