use std::{ptr, sync::Arc};

use ash::{version::DeviceV1_0, vk, Device};
use lahar::{DedicatedBuffer, DedicatedImage};
use vk_shader_macros::include_glsl;

use super::{surface_extraction::DrawBuffer, Asset, Base, Loader};
use crate::Config;
use common::{defer, world::Material};

const VERT: &[u32] = include_glsl!("shaders/voxels.vert");
const FRAG: &[u32] = include_glsl!("shaders/voxels.frag");

pub struct Voxels {
    gfx: Arc<Base>,
    static_ds_layout: vk::DescriptorSetLayout,
    frame_ds_layout: vk::DescriptorSetLayout,
    pipeline_layout: vk::PipelineLayout,
    pipeline: vk::Pipeline,
    descriptor_pool: vk::DescriptorPool,
    ds: vk::DescriptorSet,
    colors: Asset<DedicatedImage>,
    colors_view: vk::ImageView,
}

impl Voxels {
    pub fn new(config: &Config, loader: &mut Loader, buffer: &DrawBuffer, frames: u32) -> Self {
        let gfx = buffer.gfx.clone();
        let device = &*gfx.device;
        unsafe {
            // Construct the shader modules
            let vert = device
                .create_shader_module(&vk::ShaderModuleCreateInfo::builder().code(&VERT), None)
                .unwrap();
            // Note that these only need to live until the pipeline itself is constructed
            let v_guard = defer(|| device.destroy_shader_module(vert, None));

            let frag = device
                .create_shader_module(&vk::ShaderModuleCreateInfo::builder().code(&FRAG), None)
                .unwrap();
            let f_guard = defer(|| device.destroy_shader_module(frag, None));

            let static_ds_layout = device
                .create_descriptor_set_layout(
                    &vk::DescriptorSetLayoutCreateInfo::builder().bindings(&[
                        vk::DescriptorSetLayoutBinding {
                            binding: 0,
                            descriptor_type: vk::DescriptorType::STORAGE_BUFFER,
                            descriptor_count: 1,
                            stage_flags: vk::ShaderStageFlags::VERTEX,
                            p_immutable_samplers: ptr::null(),
                        },
                        vk::DescriptorSetLayoutBinding {
                            binding: 1,
                            descriptor_type: vk::DescriptorType::COMBINED_IMAGE_SAMPLER,
                            descriptor_count: 1,
                            stage_flags: vk::ShaderStageFlags::FRAGMENT,
                            p_immutable_samplers: &gfx.linear_sampler,
                        },
                    ]),
                    None,
                )
                .unwrap();
            let frame_ds_layout = device
                .create_descriptor_set_layout(
                    &vk::DescriptorSetLayoutCreateInfo::builder().bindings(&[
                        vk::DescriptorSetLayoutBinding {
                            binding: 0,
                            descriptor_type: vk::DescriptorType::STORAGE_BUFFER,
                            descriptor_count: 1,
                            stage_flags: vk::ShaderStageFlags::VERTEX,
                            p_immutable_samplers: ptr::null(),
                        },
                    ]),
                    None,
                )
                .unwrap();

            let descriptor_pool = device
                .create_descriptor_pool(
                    &vk::DescriptorPoolCreateInfo::builder()
                        .max_sets(1 + frames)
                        .pool_sizes(&[
                            vk::DescriptorPoolSize {
                                ty: vk::DescriptorType::STORAGE_BUFFER,
                                descriptor_count: 1 + frames,
                            },
                            vk::DescriptorPoolSize {
                                ty: vk::DescriptorType::COMBINED_IMAGE_SAMPLER,
                                descriptor_count: 1,
                            },
                        ]),
                    None,
                )
                .unwrap();
            let ds = device
                .allocate_descriptor_sets(
                    &vk::DescriptorSetAllocateInfo::builder()
                        .descriptor_pool(descriptor_pool)
                        .set_layouts(&[static_ds_layout]),
                )
                .unwrap()[0];
            device.update_descriptor_sets(
                &[vk::WriteDescriptorSet::builder()
                    .dst_set(ds)
                    .dst_binding(0)
                    .descriptor_type(vk::DescriptorType::STORAGE_BUFFER)
                    .buffer_info(&[vk::DescriptorBufferInfo {
                        buffer: buffer.face_buffer(),
                        offset: 0,
                        range: vk::WHOLE_SIZE,
                    }])
                    .build()],
                &[],
            );

            // Define the outward-facing interface of the shaders, incl. uniforms, samplers, etc.
            let pipeline_layout = device
                .create_pipeline_layout(
                    &vk::PipelineLayoutCreateInfo::builder()
                        .set_layouts(&[gfx.common_layout, static_ds_layout, frame_ds_layout])
                        .push_constant_ranges(&[vk::PushConstantRange {
                            stage_flags: vk::ShaderStageFlags::VERTEX,
                            offset: 0,
                            size: 12,
                        }]),
                    None,
                )
                .unwrap();

            let entry_point = cstr!("main").as_ptr();
            let mut pipelines = device
                .create_graphics_pipelines(
                    gfx.pipeline_cache,
                    &[vk::GraphicsPipelineCreateInfo::builder()
                        .stages(&[
                            vk::PipelineShaderStageCreateInfo {
                                stage: vk::ShaderStageFlags::VERTEX,
                                module: vert,
                                p_name: entry_point,
                                ..Default::default()
                            },
                            vk::PipelineShaderStageCreateInfo {
                                stage: vk::ShaderStageFlags::FRAGMENT,
                                module: frag,
                                p_name: entry_point,
                                ..Default::default()
                            },
                        ])
                        .vertex_input_state(&vk::PipelineVertexInputStateCreateInfo::default())
                        .input_assembly_state(
                            &vk::PipelineInputAssemblyStateCreateInfo::builder()
                                .topology(vk::PrimitiveTopology::TRIANGLE_LIST),
                        )
                        .viewport_state(
                            &vk::PipelineViewportStateCreateInfo::builder()
                                .scissor_count(1)
                                .viewport_count(1),
                        )
                        .rasterization_state(
                            &vk::PipelineRasterizationStateCreateInfo::builder()
                                .cull_mode(vk::CullModeFlags::BACK)
                                .front_face(vk::FrontFace::COUNTER_CLOCKWISE)
                                .polygon_mode(vk::PolygonMode::FILL)
                                .line_width(1.0),
                        )
                        .multisample_state(
                            &vk::PipelineMultisampleStateCreateInfo::builder()
                                .rasterization_samples(vk::SampleCountFlags::TYPE_1),
                        )
                        .depth_stencil_state(
                            &vk::PipelineDepthStencilStateCreateInfo::builder()
                                .depth_test_enable(true)
                                .depth_write_enable(true)
                                .depth_compare_op(vk::CompareOp::GREATER),
                        )
                        .color_blend_state(
                            &vk::PipelineColorBlendStateCreateInfo::builder().attachments(&[
                                vk::PipelineColorBlendAttachmentState {
                                    blend_enable: vk::TRUE,
                                    src_color_blend_factor: vk::BlendFactor::ONE,
                                    dst_color_blend_factor: vk::BlendFactor::ZERO,
                                    color_blend_op: vk::BlendOp::ADD,
                                    color_write_mask: vk::ColorComponentFlags::R
                                        | vk::ColorComponentFlags::G
                                        | vk::ColorComponentFlags::B,
                                    ..Default::default()
                                },
                            ]),
                        )
                        .dynamic_state(
                            &vk::PipelineDynamicStateCreateInfo::builder().dynamic_states(&[
                                vk::DynamicState::VIEWPORT,
                                vk::DynamicState::SCISSOR,
                            ]),
                        )
                        .layout(pipeline_layout)
                        .render_pass(gfx.render_pass)
                        .subpass(0)
                        .build()],
                    None,
                )
                .unwrap()
                .into_iter();

            let pipeline = pipelines.next().unwrap();
            gfx.set_name(pipeline, cstr!("voxels"));

            // Clean up the shaders explicitly, so the defer guards don't hold onto references we're
            // moving into `Self` to be returned
            v_guard.invoke();
            f_guard.invoke();

            let colors = loader.load(
                "voxel materials",
                super::PngArray {
                    path: config.data_dir.join("materials"),
                    size: common::world::Material::COUNT - 1,
                },
            );

            Self {
                gfx,
                static_ds_layout,
                frame_ds_layout,
                pipeline_layout,
                pipeline,
                descriptor_pool,
                ds,
                colors,
                colors_view: vk::ImageView::null(),
            }
        }
    }

    pub unsafe fn draw(
        &mut self,
        loader: &Loader,
        common_ds: vk::DescriptorSet,
        cmd: vk::CommandBuffer,
        buffer: &DrawBuffer,
        frame: &Frame,
        chunk: u32,
        reflected: bool,
    ) {
        let device = &*self.gfx.device;
        if self.colors_view == vk::ImageView::null() {
            if let Some(colors) = loader.get(self.colors) {
                self.colors_view = device
                    .create_image_view(
                        &vk::ImageViewCreateInfo::builder()
                            .image(colors.handle)
                            .view_type(vk::ImageViewType::TYPE_2D_ARRAY)
                            .format(vk::Format::R8G8B8A8_SRGB)
                            .subresource_range(vk::ImageSubresourceRange {
                                aspect_mask: vk::ImageAspectFlags::COLOR,
                                base_mip_level: 0,
                                level_count: 1,
                                base_array_layer: 0,
                                layer_count: (Material::COUNT - 1) as u32,
                            }),
                        None,
                    )
                    .unwrap();
                device.update_descriptor_sets(
                    &[vk::WriteDescriptorSet::builder()
                        .dst_set(self.ds)
                        .dst_binding(1)
                        .descriptor_type(vk::DescriptorType::COMBINED_IMAGE_SAMPLER)
                        .image_info(&[vk::DescriptorImageInfo {
                            sampler: vk::Sampler::null(),
                            image_view: self.colors_view,
                            image_layout: vk::ImageLayout::SHADER_READ_ONLY_OPTIMAL,
                        }])
                        .build()],
                    &[],
                );
            } else {
                return;
            }
        }

        device.cmd_bind_pipeline(cmd, vk::PipelineBindPoint::GRAPHICS, self.pipeline);
        device.cmd_bind_descriptor_sets(
            cmd,
            vk::PipelineBindPoint::GRAPHICS,
            self.pipeline_layout,
            0,
            &[common_ds, self.ds, frame.ds],
            &[],
        );
        let mut push_constants = [0; 12];
        push_constants[0..4].copy_from_slice(&chunk.to_ne_bytes());
        push_constants[4..8].copy_from_slice(&buffer.dimension().to_ne_bytes());
        push_constants[8..12].copy_from_slice(&u32::from(reflected).to_ne_bytes());
        device.cmd_push_constants(
            cmd,
            self.pipeline_layout,
            vk::ShaderStageFlags::VERTEX,
            0,
            &push_constants,
        );
        device.cmd_draw_indirect(
            cmd,
            buffer.indirect_buffer(),
            buffer.indirect_offset(chunk),
            1,
            16,
        );
    }
}

impl Drop for Voxels {
    fn drop(&mut self) {
        let device = &*self.gfx.device;
        unsafe {
            device.destroy_pipeline(self.pipeline, None);
            device.destroy_pipeline_layout(self.pipeline_layout, None);
            device.destroy_descriptor_set_layout(self.static_ds_layout, None);
            device.destroy_descriptor_set_layout(self.frame_ds_layout, None);
            device.destroy_descriptor_pool(self.descriptor_pool, None);
            if self.colors_view != vk::ImageView::null() {
                device.destroy_image_view(self.colors_view, None);
            }
        }
    }
}

pub struct Frame {
    transforms: DedicatedBuffer,
    ds: vk::DescriptorSet,
}

impl Frame {
    pub fn new(parent: &Voxels, count: u32) -> Self {
        let gfx = &parent.gfx;
        unsafe {
            let transforms = DedicatedBuffer::new(
                &gfx.device,
                &gfx.memory_properties,
                &vk::BufferCreateInfo::builder()
                    .size(vk::DeviceSize::from(count) * TRANSFORM_SIZE)
                    .usage(
                        vk::BufferUsageFlags::STORAGE_BUFFER | vk::BufferUsageFlags::TRANSFER_DST,
                    )
                    .sharing_mode(vk::SharingMode::EXCLUSIVE),
                vk::MemoryPropertyFlags::DEVICE_LOCAL,
            );
            gfx.set_name(transforms.handle, cstr!("voxel transforms"));

            let ds = gfx
                .device
                .allocate_descriptor_sets(
                    &vk::DescriptorSetAllocateInfo::builder()
                        .descriptor_pool(parent.descriptor_pool)
                        .set_layouts(&[parent.frame_ds_layout]),
                )
                .unwrap()[0];
            gfx.device.update_descriptor_sets(
                &[vk::WriteDescriptorSet::builder()
                    .dst_set(ds)
                    .dst_binding(0)
                    .descriptor_type(vk::DescriptorType::STORAGE_BUFFER)
                    .buffer_info(&[vk::DescriptorBufferInfo {
                        buffer: transforms.handle,
                        offset: 0,
                        range: vk::WHOLE_SIZE,
                    }])
                    .build()],
                &[],
            );

            Self { transforms, ds }
        }
    }

    pub fn transforms(&self) -> vk::Buffer {
        self.transforms.handle
    }
}

impl Frame {
    pub unsafe fn destroy(&mut self, device: &Device) {
        self.transforms.destroy(device);
    }
}

// 4x4 f32 matrix
pub const TRANSFORM_SIZE: vk::DeviceSize = 64;
