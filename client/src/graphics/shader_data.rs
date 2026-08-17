use std::sync::Mutex;

use ash::vk;
use lahar::BufferRegion;

pub struct ShaderData {
    pub vertex_alloc: Mutex<BufferRegion>,
    pub index_alloc: Mutex<BufferRegion>,
    /// A reasonable general-purpose texture sampler
    pub linear_sampler: vk::Sampler,
    pub mesh_ds_layout: vk::DescriptorSetLayout,
    /// Layout of common shader resources, such as the common uniform buffer
    pub common_layout: vk::DescriptorSetLayout,
}

impl ShaderData {
    pub fn new(
        device: &ash::Device,
        memory_properties: &vk::PhysicalDeviceMemoryProperties,
    ) -> Self {
        let vertex_alloc = unsafe {
            BufferRegion::new(
                device,
                memory_properties,
                16 * 1024 * 1024,
                vk::BufferUsageFlags::TRANSFER_DST | vk::BufferUsageFlags::VERTEX_BUFFER,
            )
        };
        let index_alloc = unsafe {
            BufferRegion::new(
                device,
                memory_properties,
                16 * 1024 * 1024,
                vk::BufferUsageFlags::TRANSFER_DST | vk::BufferUsageFlags::INDEX_BUFFER,
            )
        };
        let linear_sampler = unsafe {
            device
                .create_sampler(
                    &vk::SamplerCreateInfo::default()
                        .min_filter(vk::Filter::LINEAR)
                        .mag_filter(vk::Filter::LINEAR)
                        .mipmap_mode(vk::SamplerMipmapMode::NEAREST)
                        .address_mode_u(vk::SamplerAddressMode::CLAMP_TO_EDGE)
                        .address_mode_v(vk::SamplerAddressMode::CLAMP_TO_EDGE)
                        .address_mode_w(vk::SamplerAddressMode::CLAMP_TO_EDGE),
                    None,
                )
                .unwrap()
        };
        let mesh_ds_layout = unsafe {
            device
                .create_descriptor_set_layout(
                    &vk::DescriptorSetLayoutCreateInfo::default().bindings(&[
                        vk::DescriptorSetLayoutBinding {
                            binding: 0,
                            descriptor_type: vk::DescriptorType::COMBINED_IMAGE_SAMPLER,
                            descriptor_count: 1,
                            stage_flags: vk::ShaderStageFlags::FRAGMENT,
                            p_immutable_samplers: &linear_sampler,
                            ..vk::DescriptorSetLayoutBinding::default()
                        },
                    ]),
                    None,
                )
                .unwrap()
        };
        let common_layout = unsafe {
            device
                .create_descriptor_set_layout(
                    &vk::DescriptorSetLayoutCreateInfo::default().bindings(&[
                        // Uniforms
                        vk::DescriptorSetLayoutBinding {
                            binding: 0,
                            descriptor_type: vk::DescriptorType::UNIFORM_BUFFER,
                            descriptor_count: 1,
                            stage_flags: vk::ShaderStageFlags::VERTEX
                                | vk::ShaderStageFlags::FRAGMENT,
                            ..Default::default()
                        },
                        // Depth buffer
                        vk::DescriptorSetLayoutBinding {
                            binding: 1,
                            descriptor_type: vk::DescriptorType::INPUT_ATTACHMENT,
                            descriptor_count: 1,
                            stage_flags: vk::ShaderStageFlags::FRAGMENT,
                            ..Default::default()
                        },
                    ]),
                    None,
                )
                .unwrap()
        };

        ShaderData {
            vertex_alloc: Mutex::new(vertex_alloc),
            index_alloc: Mutex::new(index_alloc),
            linear_sampler,
            mesh_ds_layout,
            common_layout,
        }
    }

    pub unsafe fn destroy(&mut self, device: &ash::Device) {
        unsafe {
            device.destroy_descriptor_set_layout(self.common_layout, None);
            device.destroy_descriptor_set_layout(self.mesh_ds_layout, None);
            device.destroy_sampler(self.linear_sampler, None);
            self.index_alloc.lock().unwrap().destroy(device);
            self.vertex_alloc.lock().unwrap().destroy(device);
        }
    }
}
