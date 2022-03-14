use std::mem;

use ash::{version::DeviceV1_0, vk, Device};
use lahar::{BufferRegionAlloc, DedicatedImage};
use memoffset::offset_of;
use vk_shader_macros::include_glsl;

use super::Base;
use common::defer;

const VERT: &[u32] = include_glsl!("shaders/mesh.vert");
const FRAG: &[u32] = include_glsl!("shaders/mesh.frag");

pub struct Meshes {
    pipeline_layout: vk::PipelineLayout,
    pipeline: vk::Pipeline,
}

impl Meshes {
    #[allow(clippy::unneeded_field_pattern)] // Silence offset_of warnings nonsense
    pub fn new(gfx: &Base, ds_layout: vk::DescriptorSetLayout) -> Self {
        let device = &*gfx.device;
        unsafe {
            // Construct the shader modules
            let vert = device
                .create_shader_module(&vk::ShaderModuleCreateInfo::builder().code(VERT), None)
                .unwrap();
            // Note that these only need to live until the pipeline itself is constructed
            let v_guard = defer(|| device.destroy_shader_module(vert, None));

            let frag = device
                .create_shader_module(&vk::ShaderModuleCreateInfo::builder().code(FRAG), None)
                .unwrap();
            let f_guard = defer(|| device.destroy_shader_module(frag, None));

            // Define the outward-facing interface of the shaders, incl. uniforms, samplers, etc.
            let pipeline_layout = device
                .create_pipeline_layout(
                    &vk::PipelineLayoutCreateInfo::builder()
                        .set_layouts(&[gfx.common_layout, ds_layout])
                        .push_constant_ranges(&[vk::PushConstantRange {
                            stage_flags: vk::ShaderStageFlags::VERTEX,
                            offset: 0,
                            size: 64,
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
                        .vertex_input_state(
                            &vk::PipelineVertexInputStateCreateInfo::builder()
                                .vertex_binding_descriptions(&[vk::VertexInputBindingDescription {
                                    binding: 0,
                                    stride: mem::size_of::<Vertex>() as u32,
                                    input_rate: vk::VertexInputRate::VERTEX,
                                }])
                                .vertex_attribute_descriptions(&[
                                    vk::VertexInputAttributeDescription {
                                        location: 0,
                                        binding: 0,
                                        format: vk::Format::R32G32B32_SFLOAT,
                                        offset: offset_of!(Vertex, position) as u32,
                                    },
                                    vk::VertexInputAttributeDescription {
                                        location: 1,
                                        binding: 0,
                                        format: vk::Format::R32G32_SFLOAT,
                                        offset: offset_of!(Vertex, texcoords) as u32,
                                    },
                                    vk::VertexInputAttributeDescription {
                                        location: 2,
                                        binding: 0,
                                        format: vk::Format::R32G32B32_SFLOAT,
                                        offset: offset_of!(Vertex, normal) as u32,
                                    },
                                ]),
                        )
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
            gfx.set_name(pipeline, cstr!("sky"));

            // Clean up the shaders explicitly, so the defer guards don't hold onto references we're
            // moving into `Self` to be returned
            v_guard.invoke();
            f_guard.invoke();

            Self {
                pipeline_layout,
                pipeline,
            }
        }
    }

    pub unsafe fn draw(
        &mut self,
        device: &Device,
        common_ds: vk::DescriptorSet,
        cmd: vk::CommandBuffer,
        mesh: &Mesh,
        transform: &na::Matrix4<f32>,
    ) {
        device.cmd_bind_pipeline(cmd, vk::PipelineBindPoint::GRAPHICS, self.pipeline);
        device.cmd_bind_descriptor_sets(
            cmd,
            vk::PipelineBindPoint::GRAPHICS,
            self.pipeline_layout,
            0,
            &[common_ds, mesh.ds],
            &[],
        );
        device.cmd_push_constants(
            cmd,
            self.pipeline_layout,
            vk::ShaderStageFlags::VERTEX,
            0,
            &mem::transmute::<_, [u8; 64]>(*transform),
        );
        device.cmd_bind_vertex_buffers(cmd, 0, &[mesh.vertices.buffer], &[mesh.vertices.offset]);
        device.cmd_bind_index_buffer(
            cmd,
            mesh.indices.buffer,
            mesh.indices.offset,
            vk::IndexType::UINT32,
        );
        device.cmd_draw_indexed(cmd, mesh.index_count, 1, 0, 0, 0);
    }

    pub unsafe fn destroy(&mut self, device: &Device) {
        device.destroy_pipeline(self.pipeline, None);
        device.destroy_pipeline_layout(self.pipeline_layout, None);
    }
}

#[repr(C)]
pub struct Vertex {
    pub position: na::Point3<f32>,
    pub texcoords: na::Vector2<f32>,
    pub normal: na::Unit<na::Vector3<f32>>,
}

#[derive(Copy, Clone)]
pub struct Mesh {
    pub vertices: BufferRegionAlloc,
    pub indices: BufferRegionAlloc,
    pub index_count: u32,
    pub pool: vk::DescriptorPool,
    pub ds: vk::DescriptorSet,
    // TODO: Make shareable
    pub color: DedicatedImage,
    pub color_view: vk::ImageView,
}

impl crate::loader::Cleanup for Mesh {
    unsafe fn cleanup(mut self, gfx: &Base) {
        let device = &*gfx.device;
        device.destroy_descriptor_pool(self.pool, None);
        device.destroy_image_view(self.color_view, None);
        self.color.destroy(device);
    }
}
