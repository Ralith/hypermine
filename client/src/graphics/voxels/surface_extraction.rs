use std::{mem, ptr};

use ash::{version::DeviceV1_0, vk, Device};
use lahar::{DedicatedBuffer, DedicatedMapping};
use vk_shader_macros::include_glsl;

use super::Base;
use common::{defer, world::Material};

const COUNT: &[u32] = include_glsl!("shaders/surface-extraction/count.comp");
const PREFIX_SUM: &[u32] = include_glsl!("shaders/surface-extraction/prefix-sum.comp");
const EMIT: &[u32] = include_glsl!("shaders/surface-extraction/emit.comp");

/// GPU-accelerated surface extraction from voxel chunks
pub struct SurfaceExtraction {
    ds_layout: vk::DescriptorSetLayout,
    pipeline_layout: vk::PipelineLayout,
    count: vk::Pipeline,
    prefix_sum: vk::Pipeline,
    emit: vk::Pipeline,
}

impl SurfaceExtraction {
    pub fn new(gfx: &Base) -> Self {
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
                            size: 8,
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
                ds_layout,
                pipeline_layout,
                count,
                prefix_sum,
                emit,
            }
        }
    }

    pub unsafe fn destroy(&mut self, device: &Device) {
        device.destroy_descriptor_set_layout(self.ds_layout, None);
        device.destroy_pipeline_layout(self.pipeline_layout, None);
        device.destroy_pipeline(self.count, None);
        device.destroy_pipeline(self.prefix_sum, None);
        device.destroy_pipeline(self.emit, None);
    }
}

/// Scratch space for actually performing the extraction
pub struct ScratchBuffer {
    dimension: u32,
    voxels_staging: DedicatedMapping<[Material]>,
    voxels: DedicatedBuffer,
    counts: DedicatedBuffer,
    descriptor_pool: vk::DescriptorPool,
    descriptor_sets: Vec<vk::DescriptorSet>,
    free_slots: Vec<u32>,
}

impl ScratchBuffer {
    pub fn new(gfx: &Base, ctx: &SurfaceExtraction, concurrency: u32, dimension: u32) -> Self {
        let device = &*gfx.device;
        unsafe {
            // Padded by 2 on each dimension so each voxel of interest has a full neighborhood
            let mut voxels_size = concurrency * (dimension + 2).pow(3);
            if voxels_size % 2 == 1 {
                // Pad to even length so the shaders can safely read in 32 bit units
                voxels_size += 1;
            }
            let voxels_staging = DedicatedMapping::zeroed_array(
                device,
                &gfx.memory_properties,
                vk::BufferUsageFlags::TRANSFER_SRC,
                voxels_size as usize,
            );
            gfx.set_name(voxels_staging.buffer(), cstr!("voxels staging"));

            let voxels = DedicatedBuffer::new(
                device,
                &gfx.memory_properties,
                &vk::BufferCreateInfo::builder()
                    .size(
                        mem::size_of::<Material>() as vk::DeviceSize
                            * voxels_size as vk::DeviceSize,
                    )
                    .usage(
                        vk::BufferUsageFlags::STORAGE_BUFFER | vk::BufferUsageFlags::TRANSFER_DST,
                    )
                    .sharing_mode(vk::SharingMode::EXCLUSIVE),
                vk::MemoryPropertyFlags::DEVICE_LOCAL,
            );
            gfx.set_name(voxels.handle, cstr!("voxels"));

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
                dimension,
                voxels_staging,
                voxels,
                counts,
                descriptor_pool,
                descriptor_sets,
                free_slots: (0..concurrency).collect(),
            }
        }
    }

    pub fn alloc(&mut self) -> Option<u32> {
        self.free_slots.pop()
    }

    pub fn free(&mut self, index: u32) {
        debug_assert!(
            !self.free_slots.contains(&index),
            "double-free of surface extraction scratch slot"
        );
        self.free_slots.push(index);
    }

    /// Includes a one-voxel margin around the entire volume
    pub fn storage(&mut self, index: u32) -> &mut [Material] {
        let size = (self.dimension + 2).pow(3) as usize;
        let index = index as usize;
        &mut self.voxels_staging[size * index..size * (index + 1)]
    }

    #[allow(clippy::too_many_arguments)] // Refactoring ideas welcome
    pub unsafe fn extract(
        &mut self,
        device: &Device,
        ctx: &SurfaceExtraction,
        index: u32,
        reverse_winding: bool,
        cmd: vk::CommandBuffer,
        indirect: (vk::Buffer, vk::DeviceSize),
        face: (vk::Buffer, vk::DeviceSize),
    ) {
        let index = index as usize;

        let voxel_count = (self.dimension + 2).pow(3) as usize;
        let max_faces = 3 * (self.dimension.pow(3) + self.dimension.pow(2));
        let dispatch_count = dispatch_count(self.dimension);
        self.voxels_staging.flush(device);

        let counts_offset = (dispatch_count as usize * index * 4) as vk::DeviceSize;
        let counts_range = dispatch_count as vk::DeviceSize * 4;

        let voxels_offset =
            (voxel_count * index) as vk::DeviceSize * mem::size_of::<Material>() as vk::DeviceSize;
        let voxels_range =
            voxel_count as vk::DeviceSize * mem::size_of::<Material>() as vk::DeviceSize;

        device.update_descriptor_sets(
            &[
                vk::WriteDescriptorSet::builder()
                    .dst_set(self.descriptor_sets[index])
                    .dst_binding(0)
                    .descriptor_type(vk::DescriptorType::STORAGE_BUFFER)
                    .buffer_info(&[vk::DescriptorBufferInfo {
                        buffer: self.voxels.handle,
                        offset: voxels_offset,
                        range: voxels_range,
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
                        buffer: indirect.0,
                        offset: indirect.1,
                        range: INDIRECT_SIZE,
                    }])
                    .build(),
                vk::WriteDescriptorSet::builder()
                    .dst_set(self.descriptor_sets[index])
                    .dst_binding(3)
                    .descriptor_type(vk::DescriptorType::STORAGE_BUFFER)
                    .buffer_info(&[vk::DescriptorBufferInfo {
                        buffer: face.0,
                        offset: face.1,
                        range: max_faces as vk::DeviceSize * FACE_SIZE,
                    }])
                    .build(),
            ],
            &[],
        );

        device.cmd_copy_buffer(
            cmd,
            self.voxels_staging.buffer(),
            self.voxels.handle,
            &[vk::BufferCopy {
                src_offset: voxels_offset,
                dst_offset: voxels_offset,
                size: voxels_range,
            }],
        );

        device.cmd_pipeline_barrier(
            cmd,
            vk::PipelineStageFlags::TRANSFER,
            vk::PipelineStageFlags::COMPUTE_SHADER,
            Default::default(),
            &[],
            &[vk::BufferMemoryBarrier {
                src_access_mask: vk::AccessFlags::TRANSFER_WRITE,
                dst_access_mask: vk::AccessFlags::SHADER_READ,
                src_queue_family_index: vk::QUEUE_FAMILY_IGNORED,
                dst_queue_family_index: vk::QUEUE_FAMILY_IGNORED,
                buffer: self.voxels.handle,
                offset: voxels_offset,
                size: voxels_range,
                ..Default::default()
            }],
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
                    offset: counts_offset,
                    size: counts_range,
                    ..Default::default()
                }],
                &[],
            );

            let mut push_constants = [0; 8];
            push_constants[0..4].copy_from_slice(&i.to_ne_bytes());
            push_constants[5] = u8::from(reverse_winding);
            device.cmd_push_constants(
                cmd,
                ctx.pipeline_layout,
                vk::ShaderStageFlags::COMPUTE,
                0,
                &push_constants,
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
                offset: counts_offset,
                size: counts_range,
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
            &((face.1 / FACE_SIZE) as u32).to_ne_bytes(),
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
                    buffer: face.0,
                    offset: face.1,
                    size: max_faces as vk::DeviceSize * FACE_SIZE,
                    ..Default::default()
                },
                vk::BufferMemoryBarrier {
                    src_access_mask: vk::AccessFlags::SHADER_WRITE,
                    dst_access_mask: vk::AccessFlags::INDIRECT_COMMAND_READ,
                    src_queue_family_index: vk::QUEUE_FAMILY_IGNORED,
                    dst_queue_family_index: vk::QUEUE_FAMILY_IGNORED,
                    buffer: indirect.0,
                    offset: indirect.1,
                    size: INDIRECT_SIZE,
                    ..Default::default()
                },
            ],
            &[],
        );
    }

    pub unsafe fn destroy(&mut self, device: &Device) {
        device.destroy_descriptor_pool(self.descriptor_pool, None);
        self.voxels_staging.destroy(device);
        self.voxels.destroy(device);
        self.counts.destroy(device);
    }
}

/// Number of dispatches necessary to cover every possible face
fn dispatch_count(dimension: u32) -> vk::DeviceSize {
    // 3 faces per voxel * number of voxels, padded on each dimension for outside faces
    3 * (dimension as vk::DeviceSize + 1).pow(3)
}

/// Manages storage for ready-to-render voxels
pub struct DrawBuffer {
    indirect: DedicatedBuffer,
    faces: DedicatedBuffer,
    dimension: u32,
}

impl DrawBuffer {
    /// Allocate a buffer suitable for rendering at most `count` chunks having `dimension` voxels
    /// along each edge
    pub fn new(gfx: &Base, count: u32, dimension: u32) -> Self {
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
            let faces = DedicatedBuffer::new(
                device,
                &gfx.memory_properties,
                &vk::BufferCreateInfo::builder()
                    .size((count * max_faces) as vk::DeviceSize * FACE_SIZE)
                    .usage(vk::BufferUsageFlags::STORAGE_BUFFER)
                    .sharing_mode(vk::SharingMode::EXCLUSIVE),
                vk::MemoryPropertyFlags::DEVICE_LOCAL,
            );
            gfx.set_name(faces.handle, cstr!("faces"));

            Self {
                indirect,
                faces,
                dimension,
            }
        }
    }

    /// Buffer containing face data
    pub fn face_buffer(&self) -> vk::Buffer {
        self.faces.handle
    }

    /// Buffer containing face counts for use with cmd_draw_indirect
    pub fn indirect_buffer(&self) -> vk::Buffer {
        self.indirect.handle
    }

    /// The offset into the face buffer at which a chunk's face data can be found
    pub fn face_offset(&self, chunk: u32) -> vk::DeviceSize {
        let max_faces = 3 * (self.dimension.pow(3) + self.dimension.pow(2));
        vk::DeviceSize::from(chunk) * max_faces as vk::DeviceSize * FACE_SIZE
    }

    /// The offset into the indirect buffer at which a chunk's face data can be found
    pub fn indirect_offset(&self, chunk: u32) -> vk::DeviceSize {
        vk::DeviceSize::from(chunk) * INDIRECT_SIZE
    }

    /// Number of voxels along a chunk edge
    pub fn dimension(&self) -> u32 {
        self.dimension
    }

    pub unsafe fn destroy(&mut self, device: &Device) {
        self.indirect.destroy(device);
        self.faces.destroy(device);
    }
}

// Size of the VkDrawIndirectCommand struct
const INDIRECT_SIZE: vk::DeviceSize = 16;

const FACE_SIZE: vk::DeviceSize = 8;
