use std::{fs, fs::File, path::PathBuf};

use anyhow::{anyhow, bail, Context};
use ash::vk;
use lahar::DedicatedImage;
use tracing::trace;

use crate::loader::{LoadCtx, LoadFuture, Loadable};

pub struct PngArray {
    pub path: PathBuf,
    pub size: usize,
}

impl Loadable for PngArray {
    type Output = DedicatedImage;

    fn load(self, handle: &LoadCtx) -> LoadFuture<'_, Self::Output> {
        Box::pin(async move {
            let full_path = handle
                .cfg
                .find_asset(&self.path)
                .ok_or_else(|| anyhow!("{} not found", self.path.display()))?;
            let mut paths = fs::read_dir(&full_path)
                .with_context(|| format!("reading {}", full_path.display()))?
                .map(|x| x.map(|x| x.path()))
                .collect::<Result<Vec<_>, _>>()
                .with_context(|| format!("reading {}", full_path.display()))?;
            if paths.is_empty() {
                bail!("{} is empty", full_path.display());
            }
            if paths.len() < self.size {
                bail!(
                    "{}: expected {} textures, found {}",
                    full_path.display(),
                    self.size,
                    paths.len()
                );
            }
            paths.sort();
            paths.truncate(self.size);
            let mut dims: Option<(u32, u32)> = None;
            let mut mem = None;
            for (i, path) in paths.iter().enumerate() {
                trace!(layer=i, path=%path.display(), "loading");
                let file =
                    File::open(path).with_context(|| format!("reading {}", path.display()))?;
                let decoder = png::Decoder::new(file);
                let mut reader = decoder
                    .read_info()
                    .with_context(|| format!("decoding {}", path.display()))?;
                let info = reader.info();
                if let Some(dims) = dims {
                    if dims != (info.width, info.height) {
                        bail!(
                            "inconsistent dimensions: expected {}x{}, got {}x{}",
                            dims.0,
                            dims.1,
                            info.width,
                            info.height
                        );
                    }
                } else {
                    dims = Some((info.width, info.height));
                    mem = Some(
                        handle
                            .staging
                            .alloc(info.width as usize * info.height as usize * 4 * self.size)
                            .await
                            .ok_or_else(|| {
                                anyhow!("{}: image array too large", full_path.display())
                            })?,
                    );
                }
                let mem = mem.as_mut().unwrap();
                let step_size = info.width as usize * info.height as usize * 4;
                reader
                    .next_frame(&mut mem[i * step_size..(i + 1) * step_size])
                    .with_context(|| format!("decoding {}", path.display()))?;
            }
            let (width, height) = dims.unwrap();
            let mem = mem.unwrap();
            unsafe {
                let image = DedicatedImage::new(
                    &handle.gfx.device,
                    &handle.gfx.memory_properties,
                    &vk::ImageCreateInfo::builder()
                        .image_type(vk::ImageType::TYPE_2D)
                        .format(vk::Format::R8G8B8A8_SRGB)
                        .extent(vk::Extent3D {
                            width,
                            height,
                            depth: 1,
                        })
                        .mip_levels(1)
                        .array_layers(self.size as u32)
                        .samples(vk::SampleCountFlags::TYPE_1)
                        .usage(vk::ImageUsageFlags::SAMPLED | vk::ImageUsageFlags::TRANSFER_DST),
                );

                let range = vk::ImageSubresourceRange {
                    aspect_mask: vk::ImageAspectFlags::COLOR,
                    base_mip_level: 0,
                    level_count: 1,
                    base_array_layer: 0,
                    layer_count: self.size as u32,
                };
                let src = handle.staging.buffer();
                let buffer_offset = mem.offset();
                let dst = image.handle;

                let device = &handle.gfx.device;

                handle.prepare_commands(|work| {
                    device.cmd_pipeline_barrier(
                        work.cmd,
                        vk::PipelineStageFlags::TOP_OF_PIPE,
                        vk::PipelineStageFlags::TRANSFER,
                        vk::DependencyFlags::default(),
                        &[],
                        &[],
                        &[vk::ImageMemoryBarrier::builder()
                            .dst_access_mask(vk::AccessFlags::TRANSFER_WRITE)
                            .src_queue_family_index(vk::QUEUE_FAMILY_IGNORED)
                            .dst_queue_family_index(vk::QUEUE_FAMILY_IGNORED)
                            .old_layout(vk::ImageLayout::UNDEFINED)
                            .new_layout(vk::ImageLayout::TRANSFER_DST_OPTIMAL)
                            .image(dst)
                            .subresource_range(range)
                            .build()],
                    );
                    device.cmd_copy_buffer_to_image(
                        work.cmd,
                        src,
                        dst,
                        vk::ImageLayout::TRANSFER_DST_OPTIMAL,
                        &[vk::BufferImageCopy {
                            buffer_offset,
                            image_subresource: vk::ImageSubresourceLayers {
                                aspect_mask: vk::ImageAspectFlags::COLOR,
                                mip_level: 0,
                                base_array_layer: 0,
                                layer_count: range.layer_count,
                            },
                            image_extent: vk::Extent3D {
                                width,
                                height,
                                depth: 1,
                            },
                            ..Default::default()
                        }],
                    );
                    device.cmd_pipeline_barrier(
                        work.cmd,
                        vk::PipelineStageFlags::TRANSFER,
                        vk::PipelineStageFlags::FRAGMENT_SHADER,
                        vk::DependencyFlags::default(),
                        &[],
                        &[],
                        &[vk::ImageMemoryBarrier::builder()
                            .src_access_mask(vk::AccessFlags::TRANSFER_WRITE)
                            .dst_access_mask(vk::AccessFlags::SHADER_READ)
                            .src_queue_family_index(vk::QUEUE_FAMILY_IGNORED)
                            .dst_queue_family_index(vk::QUEUE_FAMILY_IGNORED)
                            .old_layout(vk::ImageLayout::TRANSFER_DST_OPTIMAL)
                            .new_layout(vk::ImageLayout::SHADER_READ_ONLY_OPTIMAL)
                            .image(dst)
                            .subresource_range(range)
                            .build()],
                    );
                }).await;

                // parallel_queue_handle.destroy(device);

                trace!(
                    width = width,
                    height = height,
                    path = %full_path.display(),
                    "loaded array"
                );
                Ok(image)
            }
        })
    }
}
