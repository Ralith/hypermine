use ash::{vk, Device};
use lahar::{DedicatedImage, DedicatedMapping};
use vk_shader_macros::include_glsl;

use super::surface_extraction::DrawBuffer;
use crate::{graphics::Base, Asset, Loader};
use common::{defer, world::Material};

const VERT: &[u32] = include_glsl!("shaders/voxels.vert");
const FRAG: &[u32] = include_glsl!("shaders/voxels.frag");

pub struct Surface {
    static_ds_layout: vk::DescriptorSetLayout,
    pipeline_layout: vk::PipelineLayout,
    pipeline: vk::Pipeline,
    descriptor_pool: vk::DescriptorPool,
    ds: vk::DescriptorSet,
    colors: Asset<DedicatedImage>,
    colors_view: vk::ImageView,
}

impl Surface {
    pub fn new(gfx: &Base, loader: &mut Loader, buffer: &DrawBuffer) -> Self {
        let device = &*gfx.device;
        unsafe {
            // Construct the shader modules
            let vert = device
                .create_shader_module(&vk::ShaderModuleCreateInfo::default().code(VERT), None)
                .unwrap();
            // Note that these only need to live until the pipeline itself is constructed
            let v_guard = defer(|| device.destroy_shader_module(vert, None));

            let frag = device
                .create_shader_module(&vk::ShaderModuleCreateInfo::default().code(FRAG), None)
                .unwrap();
            let f_guard = defer(|| device.destroy_shader_module(frag, None));

            let static_ds_layout = device
                .create_descriptor_set_layout(
                    &vk::DescriptorSetLayoutCreateInfo::default().bindings(&[
                        vk::DescriptorSetLayoutBinding {
                            binding: 0,
                            descriptor_type: vk::DescriptorType::STORAGE_BUFFER,
                            descriptor_count: 1,
                            stage_flags: vk::ShaderStageFlags::VERTEX,
                            ..Default::default()
                        },
                        vk::DescriptorSetLayoutBinding {
                            binding: 1,
                            descriptor_type: vk::DescriptorType::COMBINED_IMAGE_SAMPLER,
                            descriptor_count: 1,
                            stage_flags: vk::ShaderStageFlags::FRAGMENT,
                            p_immutable_samplers: &gfx.linear_sampler,
                            ..Default::default()
                        },
                    ]),
                    None,
                )
                .unwrap();

            let descriptor_pool = device
                .create_descriptor_pool(
                    &vk::DescriptorPoolCreateInfo::default()
                        .max_sets(1)
                        .pool_sizes(&[
                            vk::DescriptorPoolSize {
                                ty: vk::DescriptorType::STORAGE_BUFFER,
                                descriptor_count: 1,
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
                    &vk::DescriptorSetAllocateInfo::default()
                        .descriptor_pool(descriptor_pool)
                        .set_layouts(&[static_ds_layout]),
                )
                .unwrap()[0];
            device.update_descriptor_sets(
                &[vk::WriteDescriptorSet::default()
                    .dst_set(ds)
                    .dst_binding(0)
                    .descriptor_type(vk::DescriptorType::STORAGE_BUFFER)
                    .buffer_info(&[vk::DescriptorBufferInfo {
                        buffer: buffer.face_buffer(),
                        offset: 0,
                        range: vk::WHOLE_SIZE,
                    }])],
                &[],
            );

            // Define the outward-facing interface of the shaders, incl. uniforms, samplers, etc.
            let pipeline_layout = device
                .create_pipeline_layout(
                    &vk::PipelineLayoutCreateInfo::default()
                        .set_layouts(&[gfx.common_layout, static_ds_layout])
                        .push_constant_ranges(&[vk::PushConstantRange {
                            stage_flags: vk::ShaderStageFlags::VERTEX,
                            offset: 0,
                            size: 4,
                        }]),
                    None,
                )
                .unwrap();

            let entry_point = cstr!("main").as_ptr();
            let mut pipelines = device
                .create_graphics_pipelines(
                    gfx.pipeline_cache,
                    &[vk::GraphicsPipelineCreateInfo::default()
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
                        .vertex_input_state(
                            &vk::PipelineVertexInputStateCreateInfo::default()
                                .vertex_binding_descriptions(&[vk::VertexInputBindingDescription {
                                    binding: 0,
                                    stride: TRANSFORM_SIZE as u32,
                                    input_rate: vk::VertexInputRate::INSTANCE,
                                }])
                                .vertex_attribute_descriptions(&[
                                    vk::VertexInputAttributeDescription {
                                        location: 0,
                                        binding: 0,
                                        format: vk::Format::R32G32B32A32_SFLOAT,
                                        offset: 0,
                                    },
                                    vk::VertexInputAttributeDescription {
                                        location: 1,
                                        binding: 0,
                                        format: vk::Format::R32G32B32A32_SFLOAT,
                                        offset: 16,
                                    },
                                    vk::VertexInputAttributeDescription {
                                        location: 2,
                                        binding: 0,
                                        format: vk::Format::R32G32B32A32_SFLOAT,
                                        offset: 32,
                                    },
                                    vk::VertexInputAttributeDescription {
                                        location: 3,
                                        binding: 0,
                                        format: vk::Format::R32G32B32A32_SFLOAT,
                                        offset: 48,
                                    },
                                ]),
                        )
                        .input_assembly_state(
                            &vk::PipelineInputAssemblyStateCreateInfo::default()
                                .topology(vk::PrimitiveTopology::TRIANGLE_LIST),
                        )
                        .viewport_state(
                            &vk::PipelineViewportStateCreateInfo::default()
                                .scissor_count(1)
                                .viewport_count(1),
                        )
                        .rasterization_state(
                            &vk::PipelineRasterizationStateCreateInfo::default()
                                .cull_mode(vk::CullModeFlags::BACK)
                                .front_face(vk::FrontFace::COUNTER_CLOCKWISE)
                                .polygon_mode(vk::PolygonMode::FILL)
                                .line_width(1.0),
                        )
                        .multisample_state(
                            &vk::PipelineMultisampleStateCreateInfo::default()
                                .rasterization_samples(vk::SampleCountFlags::TYPE_1),
                        )
                        .depth_stencil_state(
                            &vk::PipelineDepthStencilStateCreateInfo::default()
                                .depth_test_enable(true)
                                .depth_write_enable(true)
                                .depth_compare_op(vk::CompareOp::GREATER),
                        )
                        .color_blend_state(
                            &vk::PipelineColorBlendStateCreateInfo::default().attachments(&[
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
                            &vk::PipelineDynamicStateCreateInfo::default().dynamic_states(&[
                                vk::DynamicState::VIEWPORT,
                                vk::DynamicState::SCISSOR,
                            ]),
                        )
                        .layout(pipeline_layout)
                        .render_pass(gfx.render_pass)
                        .subpass(0)],
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
                crate::graphics::PngArray {
                    path: "materials".into(),
                    size: common::world::Material::COUNT - 1,
                },
            );

            Self {
                static_ds_layout,
                pipeline_layout,
                pipeline,
                descriptor_pool,
                ds,
                colors,
                colors_view: vk::ImageView::null(),
            }
        }
    }

    pub unsafe fn bind(
        &mut self,
        device: &Device,
        loader: &Loader,
        dimension: u32,
        common_ds: vk::DescriptorSet,
        frame: &Frame,
        cmd: vk::CommandBuffer,
    ) -> bool {
        if self.colors_view == vk::ImageView::null() {
            if let Some(colors) = loader.get(self.colors) {
                self.colors_view = device
                    .create_image_view(
                        &vk::ImageViewCreateInfo::default()
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
                    &[vk::WriteDescriptorSet::default()
                        .dst_set(self.ds)
                        .dst_binding(1)
                        .descriptor_type(vk::DescriptorType::COMBINED_IMAGE_SAMPLER)
                        .image_info(&[vk::DescriptorImageInfo {
                            sampler: vk::Sampler::null(),
                            image_view: self.colors_view,
                            image_layout: vk::ImageLayout::SHADER_READ_ONLY_OPTIMAL,
                        }])],
                    &[],
                );
            } else {
                return false;
            }
        }

        device.cmd_bind_pipeline(cmd, vk::PipelineBindPoint::GRAPHICS, self.pipeline);
        device.cmd_bind_descriptor_sets(
            cmd,
            vk::PipelineBindPoint::GRAPHICS,
            self.pipeline_layout,
            0,
            &[common_ds, self.ds],
            &[],
        );
        device.cmd_bind_vertex_buffers(cmd, 0, &[frame.transforms.buffer()], &[0]);

        device.cmd_push_constants(
            cmd,
            self.pipeline_layout,
            vk::ShaderStageFlags::VERTEX,
            0,
            &dimension.to_ne_bytes(),
        );

        true
    }

    pub unsafe fn draw(
        &self,
        device: &Device,
        cmd: vk::CommandBuffer,
        buffer: &DrawBuffer,
        chunk: u32,
    ) {
        device.cmd_draw_indirect(
            cmd,
            buffer.indirect_buffer(),
            buffer.indirect_offset(chunk),
            1,
            16,
        );
    }

    pub unsafe fn destroy(&mut self, device: &Device) {
        device.destroy_pipeline(self.pipeline, None);
        device.destroy_pipeline_layout(self.pipeline_layout, None);
        device.destroy_descriptor_set_layout(self.static_ds_layout, None);
        device.destroy_descriptor_pool(self.descriptor_pool, None);
        if self.colors_view != vk::ImageView::null() {
            device.destroy_image_view(self.colors_view, None);
        }
    }
}

pub struct Frame {
    transforms: DedicatedMapping<[na::Matrix4<f32>]>,
}

impl Frame {
    pub fn new(gfx: &Base, count: u32) -> Self {
        unsafe {
            let transforms = DedicatedMapping::zeroed_array(
                &gfx.device,
                &gfx.memory_properties,
                vk::BufferUsageFlags::VERTEX_BUFFER | vk::BufferUsageFlags::TRANSFER_DST,
                count as usize * TRANSFORM_SIZE as usize,
            );
            gfx.set_name(transforms.buffer(), cstr!("voxel transforms"));
            Self { transforms }
        }
    }

    pub fn transforms_mut(&mut self) -> &mut [na::Matrix4<f32>] {
        &mut self.transforms
    }
}

impl Frame {
    pub unsafe fn destroy(&mut self, device: &Device) {
        self.transforms.destroy(device);
    }
}

// 4x4 f32 matrix
pub const TRANSFORM_SIZE: vk::DeviceSize = 64;
