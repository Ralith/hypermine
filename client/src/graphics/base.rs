//! Common state shared throughout the graphics system

use std::ffi::CStr;
use std::path::PathBuf;
use std::sync::Arc;
use std::{fs, io, mem, ptr};
use tracing::{info, trace, warn};

use ash::{
    version::{DeviceV1_0, InstanceV1_0},
    vk, Device,
};

use super::Core;

/// Vulkan resources shared between many parts of the renderer
pub struct Base {
    pub core: Arc<Core>,
    /// The physical device (i.e. GPU) we're rendering with
    pub physical: vk::PhysicalDevice,
    /// The logical device, containing functions used for rendering
    pub device: Arc<Device>,
    /// The queue family we're rendering in
    pub queue_family: u32,
    /// The queue used for graphics and presentation
    pub queue: vk::Queue,
    /// Information about the types of device-visible memory that can be allocated
    pub memory_properties: vk::PhysicalDeviceMemoryProperties,
    /// Cache used to speed up graphics pipeline construction
    pub pipeline_cache: vk::PipelineCache,
    /// Context in which the main rendering work occurs
    pub render_pass: vk::RenderPass,
    /// A reasonable general-purpose texture sampler
    pub linear_sampler: vk::Sampler,
    /// Layout of common shader resourcs, such as the common uniform buffer
    pub common_layout: vk::DescriptorSetLayout,
    pub limits: vk::PhysicalDeviceLimits,
    pub timestamp_bits: u32,
    pipeline_cache_path: Option<PathBuf>,
}

impl Drop for Base {
    fn drop(&mut self) {
        unsafe {
            self.device
                .destroy_pipeline_cache(self.pipeline_cache, None);
            self.device.destroy_render_pass(self.render_pass, None);
            self.device.destroy_sampler(self.linear_sampler, None);
            self.device
                .destroy_descriptor_set_layout(self.common_layout, None);
            self.device.destroy_device(None);
        }
    }
}

impl Base {
    pub fn new(
        core: Arc<Core>,
        pipeline_cache_path: Option<PathBuf>,
        device_exts: &[&CStr],
        mut device_filter: impl FnMut(vk::PhysicalDevice, u32) -> bool,
    ) -> Option<Self> {
        let pipeline_cache_data = if let Some(ref path) = pipeline_cache_path {
            match fs::read(path) {
                Ok(x) => x,
                Err(e) => {
                    if e.kind() == io::ErrorKind::NotFound {
                        info!("creating fresh pipeline cache");
                    } else {
                        warn!(path=%path.display(), "failed to load pipeline cache: {}", e);
                    }
                    Vec::new()
                }
            }
        } else {
            Vec::new()
        };
        unsafe {
            let instance = &core.instance;
            // Select a physical device and queue family to use for rendering
            let (physical, queue_family_index, queue_family_properties) = instance
                .enumerate_physical_devices()
                .unwrap()
                .into_iter()
                .find_map(|physical| {
                    instance
                        .get_physical_device_queue_family_properties(physical)
                        .into_iter()
                        .enumerate()
                        .filter_map(|(queue_family_index, info)| {
                            let supports_graphic_and_surface =
                                info.queue_flags.contains(vk::QueueFlags::GRAPHICS)
                                    && device_filter(physical, queue_family_index as u32);
                            if supports_graphic_and_surface {
                                Some((physical, queue_family_index as u32, info))
                            } else {
                                None
                            }
                        })
                        .next()
                })?;
            let physical_properties = instance.get_physical_device_properties(physical);
            let name = std::str::from_utf8(mem::transmute::<_, &[u8]>(
                &physical_properties.device_name[..physical_properties
                    .device_name
                    .iter()
                    .position(|&x| x == 0)
                    .unwrap()],
            ))
            .unwrap();
            info!(name, "selected device");

            // Create the logical device and common resources descended from it
            let device_exts = device_exts.iter().map(|x| x.as_ptr()).collect::<Vec<_>>();
            let device = Arc::new(
                instance
                    .create_device(
                        physical,
                        &vk::DeviceCreateInfo::builder()
                            .queue_create_infos(&[vk::DeviceQueueCreateInfo::builder()
                                .queue_family_index(queue_family_index)
                                .queue_priorities(&[1.0])
                                .build()])
                            .enabled_extension_names(&device_exts),
                        None,
                    )
                    .unwrap(),
            );
            let queue = device.get_device_queue(queue_family_index, 0);
            let memory_properties = instance.get_physical_device_memory_properties(physical);
            let pipeline_cache = device
                .create_pipeline_cache(
                    &vk::PipelineCacheCreateInfo::builder().initial_data(&pipeline_cache_data),
                    None,
                )
                .unwrap();

            let render_pass = device
                .create_render_pass(
                    &vk::RenderPassCreateInfo::builder()
                        .attachments(&[
                            vk::AttachmentDescription {
                                format: COLOR_FORMAT,
                                samples: vk::SampleCountFlags::TYPE_1,
                                load_op: vk::AttachmentLoadOp::CLEAR,
                                store_op: vk::AttachmentStoreOp::STORE,
                                initial_layout: vk::ImageLayout::UNDEFINED,
                                final_layout: vk::ImageLayout::PRESENT_SRC_KHR,
                                ..Default::default()
                            },
                            vk::AttachmentDescription {
                                format: vk::Format::D32_SFLOAT,
                                samples: vk::SampleCountFlags::TYPE_1,
                                load_op: vk::AttachmentLoadOp::CLEAR,
                                store_op: vk::AttachmentStoreOp::DONT_CARE,
                                initial_layout: vk::ImageLayout::UNDEFINED,
                                final_layout: vk::ImageLayout::DEPTH_STENCIL_ATTACHMENT_OPTIMAL,
                                ..Default::default()
                            },
                        ])
                        .subpasses(&[vk::SubpassDescription::builder()
                            .color_attachments(&[vk::AttachmentReference {
                                attachment: 0,
                                layout: vk::ImageLayout::COLOR_ATTACHMENT_OPTIMAL,
                            }])
                            .depth_stencil_attachment(&vk::AttachmentReference {
                                attachment: 1,
                                layout: vk::ImageLayout::DEPTH_STENCIL_ATTACHMENT_OPTIMAL,
                            })
                            .pipeline_bind_point(vk::PipelineBindPoint::GRAPHICS)
                            .build()])
                        .dependencies(&[vk::SubpassDependency {
                            src_subpass: vk::SUBPASS_EXTERNAL,
                            dst_subpass: 0,
                            src_stage_mask: vk::PipelineStageFlags::COLOR_ATTACHMENT_OUTPUT,
                            dst_stage_mask: vk::PipelineStageFlags::COLOR_ATTACHMENT_OUTPUT,
                            dst_access_mask: vk::AccessFlags::COLOR_ATTACHMENT_READ
                                | vk::AccessFlags::COLOR_ATTACHMENT_WRITE,
                            ..Default::default()
                        }]),
                    None,
                )
                .unwrap();

            let linear_sampler = device
                .create_sampler(
                    &vk::SamplerCreateInfo::builder()
                        .min_filter(vk::Filter::LINEAR)
                        .mag_filter(vk::Filter::LINEAR)
                        .mipmap_mode(vk::SamplerMipmapMode::NEAREST)
                        .address_mode_u(vk::SamplerAddressMode::CLAMP_TO_EDGE)
                        .address_mode_v(vk::SamplerAddressMode::CLAMP_TO_EDGE)
                        .address_mode_w(vk::SamplerAddressMode::CLAMP_TO_EDGE),
                    None,
                )
                .unwrap();

            let common_layout = device
                .create_descriptor_set_layout(
                    &vk::DescriptorSetLayoutCreateInfo::builder().bindings(&[
                        vk::DescriptorSetLayoutBinding {
                            binding: 0,
                            descriptor_type: vk::DescriptorType::UNIFORM_BUFFER,
                            descriptor_count: 1,
                            stage_flags: vk::ShaderStageFlags::VERTEX
                                | vk::ShaderStageFlags::FRAGMENT,
                            p_immutable_samplers: ptr::null(),
                        },
                    ]),
                    None,
                )
                .unwrap();

            Some(Self {
                core,
                physical,
                device,
                queue_family: queue_family_index,
                queue,
                memory_properties,
                pipeline_cache,
                render_pass,
                linear_sampler,
                common_layout,
                pipeline_cache_path,
                limits: physical_properties.limits,
                timestamp_bits: queue_family_properties.timestamp_valid_bits,
            })
        }
    }

    pub fn save_pipeline_cache(&self) {
        let path = match self.pipeline_cache_path {
            Some(ref x) => x,
            None => return,
        };
        let data = unsafe {
            self.device
                .get_pipeline_cache_data(self.pipeline_cache)
                .unwrap()
        };
        match fs::create_dir_all(path.parent().unwrap()).and_then(|()| fs::write(path, &data)) {
            Ok(()) => {
                trace!(len = data.len(), "wrote pipeline cache");
            }
            Err(e) => {
                warn!(path=%path.display(), "failed to save pipeline cache: {}", e);
            }
        }
    }

    /// Set an object's name for use in diagnostics
    pub unsafe fn set_name<T: vk::Handle>(&self, object: T, name: &CStr) {
        let ex = match self.core.debug_utils.as_ref() {
            Some(x) => x,
            None => return,
        };
        ex.debug_utils_set_object_name(
            self.device.handle(),
            &vk::DebugUtilsObjectNameInfoEXT::builder()
                .object_type(T::TYPE)
                .object_handle(object.as_raw())
                .object_name(name),
        )
        .unwrap();
    }

    #[cfg(test)]
    pub fn headless() -> Self {
        let core = Core::new(&[]);
        Self::new(Arc::new(core), None, &[], |_, _| true).unwrap()
    }
}

/// The pixel format we render in
pub const COLOR_FORMAT: vk::Format = vk::Format::B8G8R8A8_SRGB;
