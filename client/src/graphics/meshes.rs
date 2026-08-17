use std::mem;

use ash::{Device, vk};
use lahar::{BufferRegionAlloc, DedicatedImage};
use memoffset::offset_of;
use vk_shader_macros::include_glsl;

use crate::graphics::asset_loader::AssetLoadContext;

use super::Base;
use common::defer;

const VERT: &[u32] = include_glsl!("shaders/mesh.vert");
const FRAG: &[u32] = include_glsl!("shaders/mesh.frag");

pub struct Meshes {
    pipeline_layout: vk::PipelineLayout,
    pipeline: vk::Pipeline,
}

impl Meshes {
    pub fn new(gfx: &Base) -> Self {
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

            // Define the outward-facing interface of the shaders, incl. uniforms, samplers, etc.
            let pipeline_layout = device
                .create_pipeline_layout(
                    &vk::PipelineLayoutCreateInfo::default()
                        .set_layouts(&[
                            gfx.shader_data.common_layout,
                            gfx.shader_data.mesh_ds_layout,
                        ])
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
        unsafe {
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
                &mem::transmute::<na::Matrix4<f32>, [u8; 64]>(*transform),
            );
            device.cmd_bind_vertex_buffers(
                cmd,
                0,
                &[mesh.geometry.vertices.buffer],
                &[mesh.geometry.vertices.offset],
            );
            device.cmd_bind_index_buffer(
                cmd,
                mesh.geometry.indices.buffer,
                mesh.geometry.indices.offset,
                vk::IndexType::UINT32,
            );
            device.cmd_draw_indexed(cmd, mesh.geometry.index_count, 1, 0, 0, 0);
        }
    }

    pub unsafe fn destroy(&mut self, device: &Device) {
        unsafe {
            device.destroy_pipeline(self.pipeline, None);
            device.destroy_pipeline_layout(self.pipeline_layout, None);
        }
    }
}

#[repr(C)]
pub struct Vertex {
    pub position: na::Point3<f32>,
    pub texcoords: na::Vector2<f32>,
    pub normal: na::Unit<na::Vector3<f32>>,
}

pub struct MeshGeometryDefinition {
    pub vertices: Vec<Vertex>,
    pub indices: Vec<u32>,
}

pub struct MeshMaterialDefinition {
    pub width: u32,
    pub height: u32,
    pub srgb_rgba_color_data: Vec<u8>,
}

#[derive(Copy, Clone)]
pub struct Mesh {
    pub geometry: MeshGeometry,
    pub pool: vk::DescriptorPool,
    pub ds: vk::DescriptorSet,
    // TODO: Make shareable
    pub material: MeshMaterial,
}

impl Mesh {
    pub async fn from_definition(
        ctx: &AssetLoadContext,
        mesh_geometry: MeshGeometryDefinition,
        mesh_material: MeshMaterialDefinition,
    ) -> Self {
        unsafe {
            let (geometry, material) = tokio::join!(
                MeshGeometry::from_definition(ctx, mesh_geometry),
                MeshMaterial::from_definition(ctx, mesh_material)
            );

            let pool = ctx
                .device()
                .create_descriptor_pool(
                    &vk::DescriptorPoolCreateInfo::default()
                        .max_sets(1)
                        .pool_sizes(&[vk::DescriptorPoolSize {
                            ty: vk::DescriptorType::COMBINED_IMAGE_SAMPLER,
                            descriptor_count: 1,
                        }]),
                    None,
                )
                .unwrap();
            let ds = ctx
                .device()
                .allocate_descriptor_sets(
                    &vk::DescriptorSetAllocateInfo::default()
                        .descriptor_pool(pool)
                        .set_layouts(&[ctx.shader_data().mesh_ds_layout]),
                )
                .unwrap()[0];
            ctx.device().update_descriptor_sets(
                &[vk::WriteDescriptorSet::default()
                    .dst_set(ds)
                    .dst_binding(0)
                    .descriptor_type(vk::DescriptorType::COMBINED_IMAGE_SAMPLER)
                    .image_info(&[vk::DescriptorImageInfo {
                        sampler: vk::Sampler::null(),
                        image_view: material.color_view,
                        image_layout: vk::ImageLayout::SHADER_READ_ONLY_OPTIMAL,
                    }])],
                &[],
            );

            Mesh {
                geometry,
                pool,
                ds,
                material,
            }
        }
    }

    pub unsafe fn destroy(&mut self, device: &ash::Device) {
        unsafe {
            device.destroy_descriptor_pool(self.pool, None);
            self.material.destroy(device);
            self.geometry.destroy(device);
        }
    }
}

#[derive(Copy, Clone)]
pub struct MeshGeometry {
    pub vertices: BufferRegionAlloc,
    pub indices: BufferRegionAlloc,
    pub index_count: u32,
}

impl MeshGeometry {
    pub async fn from_definition(
        ctx: &AssetLoadContext,
        mesh_geometry: MeshGeometryDefinition,
    ) -> Self {
        unsafe {
            let work = ctx.begin_work();
            let finish_time = work.time().get();
            let vertex_staging =
                ctx.alloc_staging::<Vertex>(mesh_geometry.vertices.len(), 1, finish_time);
            let index_staging =
                ctx.alloc_staging::<u32>(mesh_geometry.indices.len(), 1, finish_time);
            std::ptr::copy_nonoverlapping(
                mesh_geometry.vertices.as_ptr(),
                vertex_staging.pointer.as_ptr(),
                mesh_geometry.vertices.len(),
            );
            std::ptr::copy_nonoverlapping(
                mesh_geometry.indices.as_ptr(),
                index_staging.pointer.as_ptr(),
                mesh_geometry.indices.len(),
            );
            let vertex_alloc = ctx.alloc_vertices(mesh_geometry.vertices.len());
            let index_alloc = ctx.alloc_indices(mesh_geometry.indices.len());
            ctx.device().cmd_copy_buffer(
                work.cmd(),
                vertex_staging.buffer,
                vertex_alloc.buffer,
                &[vk::BufferCopy {
                    src_offset: vertex_staging.offset,
                    dst_offset: vertex_alloc.offset,
                    size: vertex_staging.size,
                }],
            );
            ctx.device().cmd_copy_buffer(
                work.cmd(),
                index_staging.buffer,
                index_alloc.buffer,
                &[vk::BufferCopy {
                    src_offset: index_staging.offset,
                    dst_offset: index_alloc.offset,
                    size: index_staging.size,
                }],
            );
            ctx.device().cmd_pipeline_barrier(
                work.cmd(),
                vk::PipelineStageFlags::TRANSFER,
                vk::PipelineStageFlags::VERTEX_INPUT,
                vk::DependencyFlags::default(),
                &[],
                &[
                    vk::BufferMemoryBarrier::default()
                        .src_access_mask(vk::AccessFlags::TRANSFER_WRITE)
                        .dst_access_mask(vk::AccessFlags::VERTEX_ATTRIBUTE_READ)
                        .src_queue_family_index(vk::QUEUE_FAMILY_IGNORED)
                        .dst_queue_family_index(vk::QUEUE_FAMILY_IGNORED)
                        .buffer(vertex_alloc.buffer)
                        .offset(vertex_alloc.offset)
                        .size(vertex_staging.size),
                    vk::BufferMemoryBarrier::default()
                        .src_access_mask(vk::AccessFlags::TRANSFER_WRITE)
                        .dst_access_mask(vk::AccessFlags::INDEX_READ)
                        .src_queue_family_index(vk::QUEUE_FAMILY_IGNORED)
                        .dst_queue_family_index(vk::QUEUE_FAMILY_IGNORED)
                        .buffer(index_alloc.buffer)
                        .offset(index_alloc.offset)
                        .size(index_staging.size),
                ],
                &[],
            );
            work.end();
            ctx.wait_for_completion(finish_time).await;

            MeshGeometry {
                vertices: vertex_alloc,
                indices: index_alloc,
                index_count: u32::try_from(mesh_geometry.indices.len()).unwrap(),
            }
        }
    }

    pub unsafe fn destroy(&mut self, _device: &ash::Device) {
        // Nothing actually needs to be cleaned up here.
        // This implementation is left in so that we can remember to call it, ensuring
        // that if this ever changes, we don't forget to clean things up.
    }
}

#[derive(Copy, Clone)]
pub struct MeshMaterial {
    pub color: DedicatedImage,
    pub color_view: vk::ImageView,
}

impl MeshMaterial {
    pub async fn from_definition(
        ctx: &AssetLoadContext,
        mesh_material: MeshMaterialDefinition,
    ) -> Self {
        unsafe {
            let work = ctx.begin_work();
            let finish_time = work.time().get();
            let color_staging = ctx.alloc_staging::<u8>(
                mesh_material.width as usize * mesh_material.height as usize * 4,
                4,
                finish_time,
            );
            std::ptr::copy_nonoverlapping(
                mesh_material.srgb_rgba_color_data.as_ptr(),
                color_staging.pointer.as_ptr(),
                mesh_material.srgb_rgba_color_data.len(),
            );
            let color = DedicatedImage::new(
                ctx.device(),
                ctx.memory_properties(),
                &vk::ImageCreateInfo::default()
                    .image_type(vk::ImageType::TYPE_2D)
                    .format(vk::Format::R8G8B8A8_SRGB)
                    .extent(vk::Extent3D {
                        width: mesh_material.width,
                        height: mesh_material.height,
                        depth: 1,
                    })
                    .mip_levels(1)
                    .array_layers(1)
                    .samples(vk::SampleCountFlags::TYPE_1)
                    .usage(vk::ImageUsageFlags::SAMPLED | vk::ImageUsageFlags::TRANSFER_DST),
            );
            let range = vk::ImageSubresourceRange {
                aspect_mask: vk::ImageAspectFlags::COLOR,
                base_mip_level: 0,
                level_count: 1,
                base_array_layer: 0,
                layer_count: 1,
            };
            ctx.device().cmd_pipeline_barrier(
                work.cmd(),
                vk::PipelineStageFlags::TOP_OF_PIPE,
                vk::PipelineStageFlags::TRANSFER,
                vk::DependencyFlags::default(),
                &[],
                &[],
                &[vk::ImageMemoryBarrier::default()
                    .dst_access_mask(vk::AccessFlags::TRANSFER_WRITE)
                    .src_queue_family_index(vk::QUEUE_FAMILY_IGNORED)
                    .dst_queue_family_index(vk::QUEUE_FAMILY_IGNORED)
                    .old_layout(vk::ImageLayout::UNDEFINED)
                    .new_layout(vk::ImageLayout::TRANSFER_DST_OPTIMAL)
                    .image(color.handle)
                    .subresource_range(range)],
            );
            ctx.device().cmd_copy_buffer_to_image(
                work.cmd(),
                color_staging.buffer,
                color.handle,
                vk::ImageLayout::TRANSFER_DST_OPTIMAL,
                &[vk::BufferImageCopy {
                    buffer_offset: color_staging.offset,
                    image_subresource: vk::ImageSubresourceLayers {
                        aspect_mask: vk::ImageAspectFlags::COLOR,
                        mip_level: 0,
                        base_array_layer: 0,
                        layer_count: 1,
                    },
                    image_extent: vk::Extent3D {
                        width: mesh_material.width,
                        height: mesh_material.height,
                        depth: 1,
                    },
                    ..Default::default()
                }],
            );
            ctx.device().cmd_pipeline_barrier(
                work.cmd(),
                vk::PipelineStageFlags::TRANSFER,
                vk::PipelineStageFlags::FRAGMENT_SHADER,
                vk::DependencyFlags::default(),
                &[],
                &[],
                &[vk::ImageMemoryBarrier::default()
                    .src_access_mask(vk::AccessFlags::TRANSFER_WRITE)
                    .dst_access_mask(vk::AccessFlags::SHADER_READ)
                    .src_queue_family_index(vk::QUEUE_FAMILY_IGNORED)
                    .dst_queue_family_index(vk::QUEUE_FAMILY_IGNORED)
                    .old_layout(vk::ImageLayout::TRANSFER_DST_OPTIMAL)
                    .new_layout(vk::ImageLayout::SHADER_READ_ONLY_OPTIMAL)
                    .image(color.handle)
                    .subresource_range(range)],
            );
            work.end();
            ctx.wait_for_completion(finish_time).await;

            let color_view = ctx
                .device()
                .create_image_view(
                    &vk::ImageViewCreateInfo::default()
                        .image(color.handle)
                        .view_type(vk::ImageViewType::TYPE_2D)
                        .format(vk::Format::R8G8B8A8_SRGB)
                        .subresource_range(vk::ImageSubresourceRange {
                            aspect_mask: vk::ImageAspectFlags::COLOR,
                            base_mip_level: 0,
                            level_count: 1,
                            base_array_layer: 0,
                            layer_count: 1,
                        }),
                    None,
                )
                .unwrap();

            MeshMaterial { color, color_view }
        }
    }

    pub unsafe fn destroy(&mut self, device: &ash::Device) {
        unsafe {
            device.destroy_image_view(self.color_view, None);
            self.color.destroy(device);
        }
    }
}
