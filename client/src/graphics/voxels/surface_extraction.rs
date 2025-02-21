use std::ffi::c_char;
use std::mem;

use ash::{Device, vk};
use lahar::{DedicatedBuffer, DedicatedMapping};
use vk_shader_macros::include_glsl;

use crate::graphics::{Base, VkDrawIndirectCommand, as_bytes};
use common::{defer, world::Material};

const EXTRACT: &[u32] = include_glsl!("shaders/surface-extraction/extract.comp", target: vulkan1_1);

/// GPU-accelerated surface extraction from voxel chunks
pub struct SurfaceExtraction {
    params_layout: vk::DescriptorSetLayout,
    ds_layout: vk::DescriptorSetLayout,
    pipeline_layout: vk::PipelineLayout,
    extract: vk::Pipeline,
}

impl SurfaceExtraction {
    pub fn new(gfx: &Base) -> Self {
        let device = &*gfx.device;
        unsafe {
            let params_layout = device
                .create_descriptor_set_layout(
                    &vk::DescriptorSetLayoutCreateInfo::default().bindings(&[
                        vk::DescriptorSetLayoutBinding {
                            binding: 0,
                            descriptor_type: vk::DescriptorType::UNIFORM_BUFFER,
                            descriptor_count: 1,
                            stage_flags: vk::ShaderStageFlags::COMPUTE,
                            ..Default::default()
                        },
                    ]),
                    None,
                )
                .unwrap();
            let ds_layout = device
                .create_descriptor_set_layout(
                    &vk::DescriptorSetLayoutCreateInfo::default().bindings(&[
                        vk::DescriptorSetLayoutBinding {
                            binding: 0,
                            descriptor_type: vk::DescriptorType::STORAGE_BUFFER,
                            descriptor_count: 1,
                            stage_flags: vk::ShaderStageFlags::COMPUTE,
                            ..Default::default()
                        },
                        vk::DescriptorSetLayoutBinding {
                            binding: 1,
                            descriptor_type: vk::DescriptorType::STORAGE_BUFFER,
                            descriptor_count: 1,
                            stage_flags: vk::ShaderStageFlags::COMPUTE,
                            ..Default::default()
                        },
                        vk::DescriptorSetLayoutBinding {
                            binding: 2,
                            descriptor_type: vk::DescriptorType::STORAGE_BUFFER,
                            descriptor_count: 1,
                            stage_flags: vk::ShaderStageFlags::COMPUTE,
                            ..Default::default()
                        },
                        vk::DescriptorSetLayoutBinding {
                            binding: 3,
                            descriptor_type: vk::DescriptorType::STORAGE_BUFFER,
                            descriptor_count: 1,
                            stage_flags: vk::ShaderStageFlags::COMPUTE,
                            ..Default::default()
                        },
                    ]),
                    None,
                )
                .unwrap();
            let pipeline_layout = device
                .create_pipeline_layout(
                    &vk::PipelineLayoutCreateInfo::default()
                        .set_layouts(&[params_layout, ds_layout])
                        .push_constant_ranges(&[vk::PushConstantRange {
                            stage_flags: vk::ShaderStageFlags::COMPUTE,
                            offset: 0,
                            size: 4,
                        }]),
                    None,
                )
                .unwrap();

            let extract = device
                .create_shader_module(&vk::ShaderModuleCreateInfo::default().code(EXTRACT), None)
                .unwrap();
            let extract_guard = defer(|| device.destroy_shader_module(extract, None));

            let specialization_map_entries = [
                vk::SpecializationMapEntry {
                    constant_id: 0,
                    offset: 0,
                    size: 4,
                },
                vk::SpecializationMapEntry {
                    constant_id: 1,
                    offset: 4,
                    size: 4,
                },
                vk::SpecializationMapEntry {
                    constant_id: 2,
                    offset: 8,
                    size: 4,
                },
            ];
            let specialization = vk::SpecializationInfo::default()
                .map_entries(&specialization_map_entries)
                .data(as_bytes(&WORKGROUP_SIZE));

            let p_name = c"main".as_ptr() as *const c_char;
            let mut pipelines = device
                .create_compute_pipelines(
                    gfx.pipeline_cache,
                    &[vk::ComputePipelineCreateInfo {
                        stage: vk::PipelineShaderStageCreateInfo {
                            stage: vk::ShaderStageFlags::COMPUTE,
                            module: extract,
                            p_name,
                            p_specialization_info: &specialization,
                            ..Default::default()
                        },
                        layout: pipeline_layout,
                        ..Default::default()
                    }],
                    None,
                )
                .unwrap()
                .into_iter();

            // Free shader modules now that the actual pipelines are built
            extract_guard.invoke();

            let extract = pipelines.next().unwrap();
            gfx.set_name(extract, cstr!("extract"));

            Self {
                params_layout,
                ds_layout,
                pipeline_layout,
                extract,
            }
        }
    }

    pub unsafe fn destroy(&mut self, device: &Device) {
        unsafe {
            device.destroy_descriptor_set_layout(self.params_layout, None);
            device.destroy_descriptor_set_layout(self.ds_layout, None);
            device.destroy_pipeline_layout(self.pipeline_layout, None);
            device.destroy_pipeline(self.extract, None);
        }
    }
}

/// Scratch space for actually performing the extraction
pub struct ScratchBuffer {
    dimension: u32,
    params: DedicatedBuffer,
    /// Size of a single entry in the voxel buffer
    voxel_buffer_unit: vk::DeviceSize,
    /// Size of a single entry in the state buffer
    state_buffer_unit: vk::DeviceSize,
    voxels_staging: DedicatedMapping<[Material]>,
    voxels: DedicatedBuffer,
    state: DedicatedBuffer,
    descriptor_pool: vk::DescriptorPool,
    params_ds: vk::DescriptorSet,
    descriptor_sets: Vec<vk::DescriptorSet>,
    free_slots: Vec<u32>,
    concurrency: u32,
}

impl ScratchBuffer {
    pub fn new(gfx: &Base, ctx: &SurfaceExtraction, concurrency: u32, dimension: u32) -> Self {
        let device = &*gfx.device;
        // Padded by 2 on each dimension so each voxel of interest has a full neighborhood
        let voxel_buffer_unit = round_up(
            mem::size_of::<Material>() as vk::DeviceSize * (dimension as vk::DeviceSize + 2).pow(3),
            // Pad at least to multiples of 4 so the shaders can safely read in 32 bit units
            gfx.limits.min_storage_buffer_offset_alignment.max(4),
        );
        let voxels_size = concurrency as vk::DeviceSize * voxel_buffer_unit;

        let state_buffer_unit = round_up(4, gfx.limits.min_storage_buffer_offset_alignment);
        unsafe {
            let params = DedicatedBuffer::new(
                device,
                &gfx.memory_properties,
                &vk::BufferCreateInfo::default()
                    .size(mem::size_of::<Params>() as vk::DeviceSize)
                    .usage(
                        vk::BufferUsageFlags::UNIFORM_BUFFER | vk::BufferUsageFlags::TRANSFER_DST,
                    )
                    .sharing_mode(vk::SharingMode::EXCLUSIVE),
                vk::MemoryPropertyFlags::DEVICE_LOCAL,
            );
            gfx.set_name(params.handle, cstr!("surface extraction params"));

            let voxels_staging = DedicatedMapping::zeroed_array(
                device,
                &gfx.memory_properties,
                vk::BufferUsageFlags::TRANSFER_SRC,
                (voxels_size / mem::size_of::<Material>() as vk::DeviceSize) as usize,
            );
            gfx.set_name(voxels_staging.buffer(), cstr!("voxels staging"));

            let voxels = DedicatedBuffer::new(
                device,
                &gfx.memory_properties,
                &vk::BufferCreateInfo::default()
                    .size(voxels_size)
                    .usage(
                        vk::BufferUsageFlags::STORAGE_BUFFER | vk::BufferUsageFlags::TRANSFER_DST,
                    )
                    .sharing_mode(vk::SharingMode::EXCLUSIVE),
                vk::MemoryPropertyFlags::DEVICE_LOCAL,
            );
            gfx.set_name(voxels.handle, cstr!("voxels"));

            let state = DedicatedBuffer::new(
                device,
                &gfx.memory_properties,
                &vk::BufferCreateInfo::default()
                    .size(state_buffer_unit * vk::DeviceSize::from(concurrency))
                    .usage(
                        vk::BufferUsageFlags::STORAGE_BUFFER | vk::BufferUsageFlags::TRANSFER_DST,
                    )
                    .sharing_mode(vk::SharingMode::EXCLUSIVE),
                vk::MemoryPropertyFlags::DEVICE_LOCAL,
            );
            gfx.set_name(state.handle, cstr!("surface extraction state"));

            let descriptor_pool = device
                .create_descriptor_pool(
                    &vk::DescriptorPoolCreateInfo::default()
                        .max_sets(concurrency + 1)
                        .pool_sizes(&[
                            vk::DescriptorPoolSize {
                                ty: vk::DescriptorType::UNIFORM_BUFFER,
                                descriptor_count: 1,
                            },
                            vk::DescriptorPoolSize {
                                ty: vk::DescriptorType::STORAGE_BUFFER,
                                descriptor_count: 4 * concurrency,
                            },
                        ]),
                    None,
                )
                .unwrap();
            let mut layouts = Vec::with_capacity(concurrency as usize + 1);
            layouts.resize(concurrency as usize, ctx.ds_layout);
            layouts.push(ctx.params_layout);
            let mut descriptor_sets = device
                .allocate_descriptor_sets(
                    &vk::DescriptorSetAllocateInfo::default()
                        .descriptor_pool(descriptor_pool)
                        .set_layouts(&layouts),
                )
                .unwrap();

            let params_ds = descriptor_sets.pop().unwrap();
            device.update_descriptor_sets(
                &[vk::WriteDescriptorSet::default()
                    .dst_set(params_ds)
                    .dst_binding(0)
                    .descriptor_type(vk::DescriptorType::UNIFORM_BUFFER)
                    .buffer_info(&[vk::DescriptorBufferInfo {
                        buffer: params.handle,
                        offset: 0,
                        range: vk::WHOLE_SIZE,
                    }])],
                &[],
            );

            Self {
                dimension,
                params,
                voxel_buffer_unit,
                state_buffer_unit,
                voxels_staging,
                voxels,
                state,
                descriptor_pool,
                params_ds,
                descriptor_sets,
                free_slots: (0..concurrency).collect(),
                concurrency,
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
        let start = index as usize * (self.voxel_buffer_unit as usize / mem::size_of::<Material>());
        let length = (self.dimension + 2).pow(3) as usize;
        &mut self.voxels_staging[start..start + length]
    }

    pub unsafe fn extract(
        &mut self,
        device: &Device,
        ctx: &SurfaceExtraction,
        indirect_buffer: vk::Buffer,
        face_buffer: vk::Buffer,
        cmd: vk::CommandBuffer,
        tasks: &[ExtractTask],
    ) {
        unsafe {
            // Prevent overlap with the last batch of work
            device.cmd_pipeline_barrier(
                cmd,
                vk::PipelineStageFlags::COMPUTE_SHADER,
                vk::PipelineStageFlags::TRANSFER,
                Default::default(),
                &[vk::MemoryBarrier {
                    src_access_mask: vk::AccessFlags::SHADER_READ,
                    dst_access_mask: vk::AccessFlags::TRANSFER_WRITE,
                    ..Default::default()
                }],
                &[],
                &[],
            );
            // HACKITY HACK: Queue submit synchronization validation thinks we're
            // racing with the preceding chunk draws. Our logic to allocate unique
            // ranges should be preventing this, so this may be a false positive.
            // However, if that's true, why does the validation error only trigger a
            // handful of times at startup? Perhaps we're freeing and reusing
            // storage before the previous draw completes, and validation is somehow
            // smart enough to notice?
            device.cmd_pipeline_barrier(
                cmd,
                vk::PipelineStageFlags::VERTEX_SHADER,
                vk::PipelineStageFlags::COMPUTE_SHADER,
                Default::default(),
                &[],
                &[vk::BufferMemoryBarrier {
                    buffer: face_buffer,
                    src_access_mask: vk::AccessFlags::SHADER_READ,
                    dst_access_mask: vk::AccessFlags::SHADER_WRITE,
                    offset: 0,
                    size: vk::WHOLE_SIZE,
                    ..Default::default()
                }],
                &[],
            );

            // Prepare shared state
            device.cmd_update_buffer(
                cmd,
                self.params.handle,
                0,
                as_bytes(&Params {
                    dimension: self.dimension,
                }),
            );
            device.cmd_fill_buffer(cmd, self.state.handle, 0, vk::WHOLE_SIZE, 0);

            let voxel_count = (self.dimension + 2).pow(3) as usize;
            let voxels_range =
                voxel_count as vk::DeviceSize * mem::size_of::<Material>() as vk::DeviceSize;
            let max_faces = 3 * (self.dimension.pow(3) + self.dimension.pow(2));
            let dispatch = dispatch_sizes(self.dimension);
            device.cmd_bind_descriptor_sets(
                cmd,
                vk::PipelineBindPoint::COMPUTE,
                ctx.pipeline_layout,
                0,
                &[self.params_ds],
                &[],
            );

            // Prepare each task
            for task in tasks {
                assert!(
                    task.index < self.concurrency,
                    "index {} out of bounds for concurrency {}",
                    task.index,
                    self.concurrency
                );
                let index = task.index as usize;

                let voxels_offset = self.voxel_buffer_unit * index as vk::DeviceSize;

                device.update_descriptor_sets(
                    &[
                        vk::WriteDescriptorSet::default()
                            .dst_set(self.descriptor_sets[index])
                            .dst_binding(0)
                            .descriptor_type(vk::DescriptorType::STORAGE_BUFFER)
                            .buffer_info(&[vk::DescriptorBufferInfo {
                                buffer: self.voxels.handle,
                                offset: voxels_offset,
                                range: voxels_range,
                            }]),
                        vk::WriteDescriptorSet::default()
                            .dst_set(self.descriptor_sets[index])
                            .dst_binding(1)
                            .descriptor_type(vk::DescriptorType::STORAGE_BUFFER)
                            .buffer_info(&[vk::DescriptorBufferInfo {
                                buffer: self.state.handle,
                                offset: self.state_buffer_unit * vk::DeviceSize::from(task.index),
                                range: 4,
                            }]),
                        vk::WriteDescriptorSet::default()
                            .dst_set(self.descriptor_sets[index])
                            .dst_binding(2)
                            .descriptor_type(vk::DescriptorType::STORAGE_BUFFER)
                            .buffer_info(&[vk::DescriptorBufferInfo {
                                buffer: indirect_buffer,
                                offset: task.indirect_offset,
                                range: INDIRECT_SIZE,
                            }]),
                        vk::WriteDescriptorSet::default()
                            .dst_set(self.descriptor_sets[index])
                            .dst_binding(3)
                            .descriptor_type(vk::DescriptorType::STORAGE_BUFFER)
                            .buffer_info(&[vk::DescriptorBufferInfo {
                                buffer: face_buffer,
                                offset: task.face_offset,
                                range: max_faces as vk::DeviceSize * FACE_SIZE,
                            }]),
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
                device.cmd_update_buffer(
                    cmd,
                    indirect_buffer,
                    task.indirect_offset,
                    as_bytes(&VkDrawIndirectCommand {
                        vertex_count: 0,
                        instance_count: 1,
                        first_vertex: (task.face_offset / FACE_SIZE) as u32 * 6,
                        first_instance: task.draw_id,
                    }),
                )
            }

            device.cmd_pipeline_barrier(
                cmd,
                vk::PipelineStageFlags::TRANSFER,
                vk::PipelineStageFlags::COMPUTE_SHADER,
                Default::default(),
                &[vk::MemoryBarrier {
                    src_access_mask: vk::AccessFlags::TRANSFER_WRITE,
                    dst_access_mask: vk::AccessFlags::SHADER_READ
                        | vk::AccessFlags::SHADER_WRITE
                        | vk::AccessFlags::UNIFORM_READ,
                    ..Default::default()
                }],
                &[],
                &[],
            );

            // Write faces to memory
            device.cmd_bind_pipeline(cmd, vk::PipelineBindPoint::COMPUTE, ctx.extract);
            for task in tasks {
                device.cmd_push_constants(
                    cmd,
                    ctx.pipeline_layout,
                    vk::ShaderStageFlags::COMPUTE,
                    0,
                    &u32::from(task.reverse_winding).to_ne_bytes(),
                );
                device.cmd_bind_descriptor_sets(
                    cmd,
                    vk::PipelineBindPoint::COMPUTE,
                    ctx.pipeline_layout,
                    1,
                    &[self.descriptor_sets[task.index as usize]],
                    &[],
                );
                device.cmd_dispatch(cmd, dispatch.x, dispatch.y, dispatch.z);
            }

            device.cmd_pipeline_barrier(
                cmd,
                vk::PipelineStageFlags::COMPUTE_SHADER,
                vk::PipelineStageFlags::VERTEX_SHADER | vk::PipelineStageFlags::DRAW_INDIRECT,
                Default::default(),
                &[vk::MemoryBarrier {
                    src_access_mask: vk::AccessFlags::SHADER_WRITE,
                    dst_access_mask: vk::AccessFlags::SHADER_READ
                        | vk::AccessFlags::INDIRECT_COMMAND_READ,
                    ..Default::default()
                }],
                &[],
                &[],
            );
        }
    }

    pub unsafe fn destroy(&mut self, device: &Device) {
        unsafe {
            device.destroy_descriptor_pool(self.descriptor_pool, None);
            self.params.destroy(device);
            self.voxels_staging.destroy(device);
            self.voxels.destroy(device);
            self.state.destroy(device);
        }
    }
}

/// Specifies a single chunk's worth of surface extraction work
#[derive(Debug, Copy, Clone)]
pub struct ExtractTask {
    pub indirect_offset: vk::DeviceSize,
    pub face_offset: vk::DeviceSize,
    pub index: u32,
    pub draw_id: u32,
    pub reverse_winding: bool,
}

fn dispatch_sizes(dimension: u32) -> na::Vector3<u32> {
    fn divide_rounding_up(x: u32, y: u32) -> u32 {
        debug_assert!(x > 0 && y > 0);
        (x - 1) / y + 1
    }

    // We add 1 to each dimension because we only look at negative-facing faces of the target voxel
    na::Vector3::new(
        // Extending the X axis accounts for 3 possible faces per voxel
        divide_rounding_up((dimension + 1) * 3, WORKGROUP_SIZE[0]),
        divide_rounding_up(dimension + 1, WORKGROUP_SIZE[1]),
        divide_rounding_up(dimension + 1, WORKGROUP_SIZE[2]),
    )
}

#[repr(C)]
#[derive(Copy, Clone)]
struct Params {
    dimension: u32,
}

/// Manages storage for ready-to-render voxels
pub struct DrawBuffer {
    indirect: DedicatedBuffer,
    faces: DedicatedBuffer,
    dimension: u32,
    face_buffer_unit: vk::DeviceSize,
    count: u32,
}

impl DrawBuffer {
    /// Allocate a buffer suitable for rendering at most `count` chunks having `dimension` voxels
    /// along each edge
    pub fn new(gfx: &Base, count: u32, dimension: u32) -> Self {
        let device = &*gfx.device;

        let max_faces = 3 * (dimension.pow(3) + dimension.pow(2));
        let face_buffer_unit = round_up(
            max_faces as vk::DeviceSize * FACE_SIZE,
            gfx.limits.min_storage_buffer_offset_alignment,
        );
        let face_buffer_size = count as vk::DeviceSize * face_buffer_unit;

        unsafe {
            let indirect = DedicatedBuffer::new(
                device,
                &gfx.memory_properties,
                &vk::BufferCreateInfo::default()
                    .size(count as vk::DeviceSize * INDIRECT_SIZE)
                    .usage(
                        vk::BufferUsageFlags::STORAGE_BUFFER
                            | vk::BufferUsageFlags::INDIRECT_BUFFER
                            | vk::BufferUsageFlags::TRANSFER_DST,
                    )
                    .sharing_mode(vk::SharingMode::EXCLUSIVE),
                vk::MemoryPropertyFlags::DEVICE_LOCAL,
            );
            gfx.set_name(indirect.handle, cstr!("indirect"));

            let faces = DedicatedBuffer::new(
                device,
                &gfx.memory_properties,
                &vk::BufferCreateInfo::default()
                    .size(face_buffer_size)
                    .usage(vk::BufferUsageFlags::STORAGE_BUFFER)
                    .sharing_mode(vk::SharingMode::EXCLUSIVE),
                vk::MemoryPropertyFlags::DEVICE_LOCAL,
            );
            gfx.set_name(faces.handle, cstr!("faces"));

            Self {
                indirect,
                faces,
                dimension,
                face_buffer_unit,
                count,
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
        assert!(chunk < self.count);
        vk::DeviceSize::from(chunk) * self.face_buffer_unit
    }

    /// The offset into the indirect buffer at which a chunk's face data can be found
    pub fn indirect_offset(&self, chunk: u32) -> vk::DeviceSize {
        assert!(chunk < self.count);
        vk::DeviceSize::from(chunk) * INDIRECT_SIZE
    }

    /// Number of voxels along a chunk edge
    pub fn dimension(&self) -> u32 {
        self.dimension
    }

    pub unsafe fn destroy(&mut self, device: &Device) {
        unsafe {
            self.indirect.destroy(device);
            self.faces.destroy(device);
        }
    }
}

// Size of the VkDrawIndirectCommand struct
const INDIRECT_SIZE: vk::DeviceSize = 16;

const FACE_SIZE: vk::DeviceSize = 8;

const WORKGROUP_SIZE: [u32; 3] = [4, 4, 4];

fn round_up(value: vk::DeviceSize, alignment: vk::DeviceSize) -> vk::DeviceSize {
    value.div_ceil(alignment) * alignment
}
