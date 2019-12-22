use std::{ptr, sync::Arc};

use ash::{version::DeviceV1_0, vk};
use lahar::{DedicatedBuffer, DedicatedMapping};
use vk_shader_macros::include_glsl;

use super::Base;
use common::{defer, world::Material};

const COUNT: &[u32] = include_glsl!("shaders/surface-extraction/count.comp");
const PREFIX_SUM: &[u32] = include_glsl!("shaders/surface-extraction/prefix-sum.comp");
const EMIT: &[u32] = include_glsl!("shaders/surface-extraction/emit.comp");

/// GPU-accelerated surface extraction from voxel chunks
pub struct SurfaceExtraction {
    gfx: Arc<Base>,
    ds_layout: vk::DescriptorSetLayout,
    pipeline_layout: vk::PipelineLayout,
    count: vk::Pipeline,
    prefix_sum: vk::Pipeline,
    emit: vk::Pipeline,
}

impl SurfaceExtraction {
    pub fn new(gfx: Arc<Base>) -> Self {
        let device = &*gfx.device;
        unsafe {
            let ds_layout = device
                .create_descriptor_set_layout(
                    &vk::DescriptorSetLayoutCreateInfo::builder().bindings(&[
                        vk::DescriptorSetLayoutBinding {
                            binding: 0,
                            descriptor_type: vk::DescriptorType::STORAGE_BUFFER,
                            descriptor_count: 1,
                            stage_flags: vk::ShaderStageFlags::COMPUTE,
                            p_immutable_samplers: ptr::null(),
                        },
                        vk::DescriptorSetLayoutBinding {
                            binding: 1,
                            descriptor_type: vk::DescriptorType::STORAGE_BUFFER,
                            descriptor_count: 1,
                            stage_flags: vk::ShaderStageFlags::COMPUTE,
                            p_immutable_samplers: ptr::null(),
                        },
                        vk::DescriptorSetLayoutBinding {
                            binding: 2,
                            descriptor_type: vk::DescriptorType::STORAGE_BUFFER,
                            descriptor_count: 1,
                            stage_flags: vk::ShaderStageFlags::COMPUTE,
                            p_immutable_samplers: ptr::null(),
                        },
                        vk::DescriptorSetLayoutBinding {
                            binding: 3,
                            descriptor_type: vk::DescriptorType::STORAGE_BUFFER,
                            descriptor_count: 1,
                            stage_flags: vk::ShaderStageFlags::COMPUTE,
                            p_immutable_samplers: ptr::null(),
                        },
                    ]),
                    None,
                )
                .unwrap();
            let pipeline_layout = device
                .create_pipeline_layout(
                    &vk::PipelineLayoutCreateInfo::builder()
                        .set_layouts(&[ds_layout])
                        .push_constant_ranges(&[vk::PushConstantRange {
                            stage_flags: vk::ShaderStageFlags::COMPUTE,
                            offset: 0,
                            size: 4,
                        }]),
                    None,
                )
                .unwrap();

            let count = device
                .create_shader_module(&vk::ShaderModuleCreateInfo::builder().code(&COUNT), None)
                .unwrap();
            let count_guard = defer(|| device.destroy_shader_module(count, None));
            let prefix_sum = device
                .create_shader_module(
                    &vk::ShaderModuleCreateInfo::builder().code(&PREFIX_SUM),
                    None,
                )
                .unwrap();
            let prefix_sum_guard = defer(|| device.destroy_shader_module(prefix_sum, None));
            let emit = device
                .create_shader_module(&vk::ShaderModuleCreateInfo::builder().code(&EMIT), None)
                .unwrap();
            let emit_guard = defer(|| device.destroy_shader_module(emit, None));

            let p_name = b"main\0".as_ptr() as *const i8;
            let mut pipelines = device
                .create_compute_pipelines(
                    gfx.pipeline_cache,
                    &[
                        vk::ComputePipelineCreateInfo {
                            stage: vk::PipelineShaderStageCreateInfo {
                                stage: vk::ShaderStageFlags::COMPUTE,
                                module: count,
                                p_name,
                                ..Default::default()
                            },
                            layout: pipeline_layout,
                            ..Default::default()
                        },
                        vk::ComputePipelineCreateInfo {
                            stage: vk::PipelineShaderStageCreateInfo {
                                stage: vk::ShaderStageFlags::COMPUTE,
                                module: prefix_sum,
                                p_name,
                                ..Default::default()
                            },
                            layout: pipeline_layout,
                            ..Default::default()
                        },
                        vk::ComputePipelineCreateInfo {
                            stage: vk::PipelineShaderStageCreateInfo {
                                stage: vk::ShaderStageFlags::COMPUTE,
                                module: emit,
                                p_name,
                                ..Default::default()
                            },
                            layout: pipeline_layout,
                            ..Default::default()
                        },
                    ],
                    None,
                )
                .unwrap()
                .into_iter();

            // Free shader modules now that the actual pipelines are built
            count_guard.invoke();
            prefix_sum_guard.invoke();
            emit_guard.invoke();

            let count = pipelines.next().unwrap();
            gfx.set_name(count, cstr!("count"));
            let prefix_sum = pipelines.next().unwrap();
            gfx.set_name(prefix_sum, cstr!("prefix_sum"));
            let emit = pipelines.next().unwrap();
            gfx.set_name(prefix_sum, cstr!("emit"));

            Self {
                gfx,
                ds_layout,
                pipeline_layout,
                count,
                prefix_sum,
                emit,
            }
        }
    }
}

impl Drop for SurfaceExtraction {
    fn drop(&mut self) {
        let device = &*self.gfx.device;
        unsafe {
            device.destroy_descriptor_set_layout(self.ds_layout, None);
            device.destroy_pipeline_layout(self.pipeline_layout, None);
            device.destroy_pipeline(self.count, None);
            device.destroy_pipeline(self.prefix_sum, None);
            device.destroy_pipeline(self.emit, None);
        }
    }
}

/// Scratch space for actually performing the extraction
pub struct ScratchBuffer {
    gfx: Arc<Base>,
    dimension: u32,
    voxels: DedicatedMapping<[Material]>,
    counts: DedicatedBuffer,
    descriptor_pool: vk::DescriptorPool,
    descriptor_sets: Vec<vk::DescriptorSet>,
}

impl ScratchBuffer {
    pub fn new(ctx: &SurfaceExtraction, concurrency: u32, dimension: u32) -> Self {
        let gfx = ctx.gfx.clone();
        let device = &*gfx.device;
        unsafe {
            let voxels = DedicatedMapping::zeroed_array(
                device,
                &gfx.memory_properties,
                vk::BufferUsageFlags::STORAGE_BUFFER,
                (concurrency * dimension.pow(3)) as usize,
            );
            gfx.set_name(voxels.buffer(), cstr!("voxels"));

            let dispatch_count = dispatch_count(dimension);
            let counts = DedicatedBuffer::new(
                device,
                &gfx.memory_properties,
                &vk::BufferCreateInfo::builder()
                    .size(dispatch_count * 4 * concurrency as vk::DeviceSize)
                    .usage(vk::BufferUsageFlags::STORAGE_BUFFER)
                    .sharing_mode(vk::SharingMode::EXCLUSIVE),
                vk::MemoryPropertyFlags::DEVICE_LOCAL,
            );
            gfx.set_name(counts.handle, cstr!("counts"));

            let descriptor_pool = device
                .create_descriptor_pool(
                    &vk::DescriptorPoolCreateInfo::builder()
                        .max_sets(concurrency)
                        .pool_sizes(&[vk::DescriptorPoolSize {
                            ty: vk::DescriptorType::STORAGE_BUFFER,
                            descriptor_count: 4 * concurrency,
                        }]),
                    None,
                )
                .unwrap();
            let descriptor_sets = device
                .allocate_descriptor_sets(
                    &vk::DescriptorSetAllocateInfo::builder()
                        .descriptor_pool(descriptor_pool)
                        .set_layouts(&vec![ctx.ds_layout; concurrency as usize]),
                )
                .unwrap();

            Self {
                gfx,
                dimension,
                voxels,
                counts,
                descriptor_pool,
                descriptor_sets,
            }
        }
    }

    pub fn storage(&mut self, index: usize) -> &mut [Material] {
        let size = self.dimension.pow(3) as usize;
        &mut self.voxels[size * index..size * (index + 1)]
    }

    pub unsafe fn extract(
        &mut self,
        ctx: &SurfaceExtraction,
        index: usize,
        cmd: vk::CommandBuffer,
        indirect: vk::Buffer,
        indirect_offset: vk::DeviceSize,
        vertex: vk::Buffer,
        vertex_offset: vk::DeviceSize,
    ) {
        let device = &*self.gfx.device;

        let voxel_count = self.dimension.pow(3) as usize;
        let max_faces = 3 * (self.dimension.pow(3) + self.dimension.pow(2));
        let dispatch_count = dispatch_count(self.dimension);
        self.voxels
            .flush_elts(device, voxel_count * index..voxel_count * (index + 1));

        let counts_offset = (dispatch_count as usize * index * 4) as vk::DeviceSize;
        let counts_range = dispatch_count as vk::DeviceSize * 4;

        device.update_descriptor_sets(
            &[
                vk::WriteDescriptorSet::builder()
                    .dst_set(self.descriptor_sets[index])
                    .dst_binding(0)
                    .descriptor_type(vk::DescriptorType::STORAGE_BUFFER)
                    .buffer_info(&[vk::DescriptorBufferInfo {
                        buffer: self.voxels.buffer(),
                        offset: (voxel_count * index * 2) as vk::DeviceSize,
                        range: voxel_count as vk::DeviceSize * 2,
                    }])
                    .build(),
                vk::WriteDescriptorSet::builder()
                    .dst_set(self.descriptor_sets[index])
                    .dst_binding(1)
                    .descriptor_type(vk::DescriptorType::STORAGE_BUFFER)
                    .buffer_info(&[vk::DescriptorBufferInfo {
                        buffer: self.counts.handle,
                        offset: counts_offset,
                        range: counts_range,
                    }])
                    .build(),
                vk::WriteDescriptorSet::builder()
                    .dst_set(self.descriptor_sets[index])
                    .dst_binding(2)
                    .descriptor_type(vk::DescriptorType::STORAGE_BUFFER)
                    .buffer_info(&[vk::DescriptorBufferInfo {
                        buffer: indirect,
                        offset: indirect_offset,
                        range: INDIRECT_SIZE,
                    }])
                    .build(),
                vk::WriteDescriptorSet::builder()
                    .dst_set(self.descriptor_sets[index])
                    .dst_binding(3)
                    .descriptor_type(vk::DescriptorType::STORAGE_BUFFER)
                    .buffer_info(&[vk::DescriptorBufferInfo {
                        buffer: vertex,
                        offset: vertex_offset,
                        range: max_faces as vk::DeviceSize * FACE_SIZE,
                    }])
                    .build(),
            ],
            &[],
        );
        device.cmd_bind_descriptor_sets(
            cmd,
            vk::PipelineBindPoint::COMPUTE,
            ctx.pipeline_layout,
            0,
            &[self.descriptor_sets[index]],
            &[],
        );

        device.cmd_bind_pipeline(cmd, vk::PipelineBindPoint::COMPUTE, ctx.count);
        device.cmd_dispatch(
            cmd,
            (self.dimension + 1) * 3,
            self.dimension + 1,
            self.dimension + 1,
        );

        device.cmd_bind_pipeline(cmd, vk::PipelineBindPoint::COMPUTE, ctx.prefix_sum);
        let passes = dispatch_count.next_power_of_two().trailing_zeros();
        for i in 0..passes {
            device.cmd_pipeline_barrier(
                cmd,
                vk::PipelineStageFlags::COMPUTE_SHADER,
                vk::PipelineStageFlags::COMPUTE_SHADER,
                Default::default(),
                &[],
                &[vk::BufferMemoryBarrier {
                    src_access_mask: vk::AccessFlags::SHADER_WRITE,
                    dst_access_mask: vk::AccessFlags::SHADER_READ,
                    src_queue_family_index: vk::QUEUE_FAMILY_IGNORED,
                    dst_queue_family_index: vk::QUEUE_FAMILY_IGNORED,
                    buffer: self.counts.handle,
                    offset: (voxel_count * index * 4) as vk::DeviceSize,
                    size: voxel_count as vk::DeviceSize * 4,
                    ..Default::default()
                }],
                &[],
            );

            device.cmd_push_constants(
                cmd,
                ctx.pipeline_layout,
                vk::ShaderStageFlags::COMPUTE,
                0,
                &i.to_ne_bytes(),
            );
            device.cmd_dispatch(
                cmd,
                (self.dimension + 1) * 3,
                self.dimension + 1,
                self.dimension + 1,
            );
        }

        device.cmd_pipeline_barrier(
            cmd,
            vk::PipelineStageFlags::COMPUTE_SHADER,
            vk::PipelineStageFlags::COMPUTE_SHADER,
            Default::default(),
            &[],
            &[vk::BufferMemoryBarrier {
                src_access_mask: vk::AccessFlags::SHADER_WRITE,
                dst_access_mask: vk::AccessFlags::SHADER_READ,
                src_queue_family_index: vk::QUEUE_FAMILY_IGNORED,
                dst_queue_family_index: vk::QUEUE_FAMILY_IGNORED,
                buffer: self.counts.handle,
                offset: (voxel_count * index * 4) as vk::DeviceSize,
                size: voxel_count as vk::DeviceSize * 4,
                ..Default::default()
            }],
            &[],
        );

        device.cmd_bind_pipeline(cmd, vk::PipelineBindPoint::COMPUTE, ctx.emit);
        device.cmd_push_constants(
            cmd,
            ctx.pipeline_layout,
            vk::ShaderStageFlags::COMPUTE,
            0,
            &((vertex_offset / VERTEX_SIZE) as u32).to_ne_bytes(),
        );
        device.cmd_dispatch(
            cmd,
            (self.dimension + 1) * 3,
            self.dimension + 1,
            self.dimension + 1,
        );

        device.cmd_pipeline_barrier(
            cmd,
            vk::PipelineStageFlags::COMPUTE_SHADER,
            vk::PipelineStageFlags::VERTEX_SHADER | vk::PipelineStageFlags::DRAW_INDIRECT,
            Default::default(),
            &[],
            &[
                vk::BufferMemoryBarrier {
                    src_access_mask: vk::AccessFlags::SHADER_WRITE,
                    dst_access_mask: vk::AccessFlags::SHADER_READ,
                    src_queue_family_index: vk::QUEUE_FAMILY_IGNORED,
                    dst_queue_family_index: vk::QUEUE_FAMILY_IGNORED,
                    buffer: vertex,
                    offset: vertex_offset,
                    size: max_faces as vk::DeviceSize * FACE_SIZE,
                    ..Default::default()
                },
                vk::BufferMemoryBarrier {
                    src_access_mask: vk::AccessFlags::SHADER_WRITE,
                    dst_access_mask: vk::AccessFlags::INDIRECT_COMMAND_READ,
                    src_queue_family_index: vk::QUEUE_FAMILY_IGNORED,
                    dst_queue_family_index: vk::QUEUE_FAMILY_IGNORED,
                    buffer: indirect,
                    offset: indirect_offset,
                    size: INDIRECT_SIZE,
                    ..Default::default()
                },
            ],
            &[],
        );
    }
}

impl Drop for ScratchBuffer {
    fn drop(&mut self) {
        let device = &*self.gfx.device;
        unsafe {
            device.destroy_descriptor_pool(self.descriptor_pool, None);
            self.voxels.destroy(device);
            self.counts.destroy(device);
        }
    }
}

/// Number of dispatches necessary to cover every possible face
fn dispatch_count(dimension: u32) -> vk::DeviceSize {
    // 3 faces per voxel * number of voxels, padded on each dimension for outside faces
    3 * (dimension as vk::DeviceSize + 1).pow(3)
}

/// Manages storage for ready-to-render voxels
pub struct DrawBuffer {
    gfx: Arc<Base>,
    indirect: DedicatedBuffer,
    vertices: DedicatedBuffer,
    freelist: Vec<u32>,
}

impl DrawBuffer {
    /// Allocate a buffer suitable for rendering at most `count` chunks having `dimension` voxels
    /// along each edge
    pub fn new(gfx: Arc<Base>, count: u32, dimension: u32) -> Self {
        let device = &*gfx.device;

        unsafe {
            let indirect = DedicatedBuffer::new(
                device,
                &gfx.memory_properties,
                &vk::BufferCreateInfo::builder()
                    .size(count as vk::DeviceSize * INDIRECT_SIZE)
                    .usage(
                        vk::BufferUsageFlags::STORAGE_BUFFER
                            | vk::BufferUsageFlags::INDIRECT_BUFFER,
                    )
                    .sharing_mode(vk::SharingMode::EXCLUSIVE),
                vk::MemoryPropertyFlags::DEVICE_LOCAL,
            );
            gfx.set_name(indirect.handle, cstr!("indirect"));

            let max_faces = 3 * (dimension.pow(3) + dimension.pow(2));
            let vertices = DedicatedBuffer::new(
                device,
                &gfx.memory_properties,
                &vk::BufferCreateInfo::builder()
                    .size((count * max_faces) as vk::DeviceSize * FACE_SIZE)
                    .usage(
                        vk::BufferUsageFlags::STORAGE_BUFFER | vk::BufferUsageFlags::VERTEX_BUFFER,
                    )
                    .sharing_mode(vk::SharingMode::EXCLUSIVE),
                vk::MemoryPropertyFlags::DEVICE_LOCAL,
            );
            gfx.set_name(vertices.handle, cstr!("vertices"));

            Self {
                gfx,
                indirect,
                vertices,
                freelist: (0..count).rev().collect(),
            }
        }
    }

    /// Allocate storage for a chunk's surface
    pub fn alloc(&mut self) -> Option<Chunk> {
        Some(Chunk(self.freelist.pop()?))
    }

    /// Release storage for reuse
    pub fn free(&mut self, chunk: Chunk) {
        self.freelist.push(chunk.0);
    }

    /// Buffer containing vertex data
    pub fn vertex_buffer(&self) -> vk::Buffer {
        self.vertices.handle
    }

    /// Buffer containing vertex counts for use with cmd_draw_indirect
    pub fn indirect_buffer(&self) -> vk::Buffer {
        self.indirect.handle
    }

    /// The offset into the vertex buffer at which a chunk's vertex data can be found
    pub fn vertex_offset(&self, chunk: &Chunk) -> vk::DeviceSize {
        vk::DeviceSize::from(chunk.0) * FACE_SIZE
    }

    /// The offset into the indirect buffer at which a chunk's vertex data can be found
    pub fn indirect_offset(&self, chunk: &Chunk) -> vk::DeviceSize {
        vk::DeviceSize::from(chunk.0) * INDIRECT_SIZE
    }
}

impl Drop for DrawBuffer {
    fn drop(&mut self) {
        let device = &*self.gfx.device;
        unsafe {
            self.indirect.destroy(device);
            self.vertices.destroy(device);
        }
    }
}

// Size of the VkDrawIndirectCommand struct
const INDIRECT_SIZE: vk::DeviceSize = 16;

// Two triangles whose vertices are each encoded in 4 bytes
const FACE_SIZE: vk::DeviceSize = 2 * 3 * VERTEX_SIZE;

// R8G8B8A8
const VERTEX_SIZE: vk::DeviceSize = 4;

#[derive(Debug, Eq, PartialEq)]
pub struct Chunk(u32);
