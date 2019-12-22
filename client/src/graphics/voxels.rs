use std::sync::Arc;

use ash::{version::DeviceV1_0, vk};
use vk_shader_macros::include_glsl;

use super::{
    surface_extraction::{Chunk, DrawBuffer},
    Base, NOOP_STENCIL_STATE,
};
use common::defer;

const VERT: &[u32] = include_glsl!("shaders/voxels.vert");
const FRAG: &[u32] = include_glsl!("shaders/voxels.frag");

pub struct Voxels {
    gfx: Arc<Base>,
    pipeline_layout: vk::PipelineLayout,
    pipeline: vk::Pipeline,
}

impl Voxels {
    pub fn new(gfx: Arc<Base>) -> Self {
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

            // Define the outward-facing interface of the shaders, incl. uniforms, samplers, etc.
            let pipeline_layout = device
                .create_pipeline_layout(
                    &vk::PipelineLayoutCreateInfo::builder().set_layouts(&[gfx.common_layout]),
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
                                    stride: 4,
                                    input_rate: vk::VertexInputRate::VERTEX,
                                }])
                                .vertex_attribute_descriptions(&[
                                    vk::VertexInputAttributeDescription {
                                        location: 0,
                                        binding: 0,
                                        format: vk::Format::R8G8B8A8_UINT,
                                        offset: 0,
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
                                .cull_mode(vk::CullModeFlags::NONE)
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
                                .depth_compare_op(vk::CompareOp::GREATER_OR_EQUAL)
                                .front(NOOP_STENCIL_STATE)
                                .back(NOOP_STENCIL_STATE),
                        )
                        .color_blend_state(
                            &vk::PipelineColorBlendStateCreateInfo::builder().attachments(&[
                                vk::PipelineColorBlendAttachmentState {
                                    blend_enable: vk::TRUE,
                                    src_color_blend_factor: vk::BlendFactor::ONE,
                                    dst_color_blend_factor: vk::BlendFactor::ONE,
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

            Self {
                gfx,
                pipeline_layout,
                pipeline,
            }
        }
    }

    pub unsafe fn draw(&mut self, cmd: vk::CommandBuffer, buffer: &DrawBuffer, chunk: &Chunk) {
        let device = &*self.gfx.device;
        device.cmd_bind_pipeline(cmd, vk::PipelineBindPoint::GRAPHICS, self.pipeline);
        device.cmd_bind_vertex_buffers(cmd, 0, &[buffer.vertex_buffer()], &[0]);
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
        }
    }
}
