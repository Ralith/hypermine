use std::sync::Arc;

use ash::{version::DeviceV1_0, vk};
use vk_shader_macros::include_glsl;

use super::{Base, NOOP_STENCIL_STATE};
use common::defer;

const VERT: &[u32] = include_glsl!("shaders/fullscreen.vert");
const FRAG: &[u32] = include_glsl!("shaders/sky.frag");

pub struct Sky {
    gfx: Arc<Base>,
    pipeline_layout: vk::PipelineLayout,
    pipeline: vk::Pipeline,
}

impl Sky {
    pub fn new(gfx: Arc<Base>) -> Self {
        let device = &*gfx.device;
        unsafe {
            let vert = device
                .create_shader_module(&vk::ShaderModuleCreateInfo::builder().code(&VERT), None)
                .unwrap();
            let v_guard = defer(|| device.destroy_shader_module(vert, None));

            let frag = device
                .create_shader_module(&vk::ShaderModuleCreateInfo::builder().code(&FRAG), None)
                .unwrap();
            let f_guard = defer(|| device.destroy_shader_module(frag, None));

            let pipeline_layout = device
                .create_pipeline_layout(&vk::PipelineLayoutCreateInfo::builder(), None)
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
                                .depth_write_enable(false)
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
            v_guard.invoke();
            f_guard.invoke();

            let pipeline = pipelines.next().unwrap();
            gfx.set_name(pipeline, cstr!("sky"));

            Self {
                gfx,
                pipeline_layout,
                pipeline,
            }
        }
    }

    pub unsafe fn draw(&mut self, cmd: vk::CommandBuffer) {
        let device = &*self.gfx.device;
        device.cmd_bind_pipeline(cmd, vk::PipelineBindPoint::GRAPHICS, self.pipeline);
        device.cmd_draw(cmd, 3, 1, 0, 0);
    }
}

impl Drop for Sky {
    fn drop(&mut self) {
        let device = &*self.gfx.device;
        unsafe {
            device.destroy_pipeline(self.pipeline, None);
            device.destroy_pipeline_layout(self.pipeline_layout, None);
        }
    }
}
