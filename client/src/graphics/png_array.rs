use std::{
    fs::{self, File},
    io::BufReader,
    path::PathBuf,
};

use anyhow::{Context, anyhow, bail, ensure};
use ash::vk;
use common::Anonymize;
use lahar::DedicatedImage;

use crate::graphics::asset_loader::AssetLoadContext;

pub struct PngArray {
    pub path: PathBuf,
    pub size: usize,
}

impl PngArray {
    async fn load_inner(self, context: &skid_steer::Context<'_>) -> anyhow::Result<DedicatedImage> {
        tracing::trace!("Started loading png array");
        let ctx: &AssetLoadContext = context.get().unwrap();
        let full_path = ctx
            .find_asset(&self.path)
            .ok_or_else(|| anyhow!("{} not found", self.path.anonymize().display()))?;
        let mut paths = fs::read_dir(&full_path)
            .with_context(|| format!("reading {}", full_path.anonymize().display()))?
            .map(|x| x.map(|x| x.path()))
            .collect::<Result<Vec<_>, _>>()
            .with_context(|| format!("reading {}", full_path.anonymize().display()))?;
        if paths.len() < self.size {
            bail!(
                "{}: expected {} textures, found {}",
                full_path.anonymize().display(),
                self.size,
                paths.len()
            );
        }
        paths.sort();
        paths.truncate(self.size);
        let mut dims: Option<(u32, u32)> = None;
        let mut image_data: Vec<u8> = Vec::new();
        for (i, path) in paths.iter().enumerate() {
            tracing::trace!(layer=i, path=%path.anonymize().display(), "loading");
            let file = File::open(path)
                .with_context(|| format!("reading {}", path.anonymize().display()))?;
            let decoder = png::Decoder::new(BufReader::new(file));
            let mut reader = decoder
                .read_info()
                .with_context(|| format!("decoding {}", path.anonymize().display()))?;
            let info = reader.info();
            let step_size = info.width as usize * info.height as usize * 4;
            ensure!(info.color_type == png::ColorType::Rgba);
            ensure!(info.bit_depth == png::BitDepth::Eight);
            ensure!(reader.output_buffer_size() == Some(step_size));
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
                image_data.resize(step_size * self.size, 0);
            }
            reader
                .next_frame(&mut image_data[i * step_size..(i + 1) * step_size])
                .with_context(|| format!("decoding {}", path.anonymize().display()))?;
        }
        let (width, height) = dims.unwrap();
        unsafe {
            let image = DedicatedImage::new(
                ctx.device(),
                ctx.memory_properties(),
                &vk::ImageCreateInfo::default()
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
            let work = ctx.begin_work();
            let finish_time = work.time().get();
            let mem = ctx.alloc_staging(image_data.len(), 4, finish_time);
            std::ptr::copy_nonoverlapping(
                image_data.as_ptr(),
                mem.pointer.as_ptr(),
                image_data.len(),
            );
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
                    .image(image.handle)
                    .subresource_range(range)],
            );
            ctx.device().cmd_copy_buffer_to_image(
                work.cmd(),
                mem.buffer,
                image.handle,
                vk::ImageLayout::TRANSFER_DST_OPTIMAL,
                &[vk::BufferImageCopy {
                    buffer_offset: mem.offset,
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
                    .image(image.handle)
                    .subresource_range(range)],
            );
            work.end();
            tracing::trace!("Awaiting parallel queue");
            ctx.wait_for_completion(finish_time).await;
            tracing::trace!("Finished awaiting parallel queue");

            tracing::trace!(
                width = width,
                height = height,
                path = %full_path.anonymize().display(),
                "loaded array"
            );
            tracing::trace!("png_array loaded");
            Ok(image)
        }
    }
}

impl skid_steer::Source for PngArray {
    type Output = DedicatedImage;

    async fn load(self, context: &skid_steer::Context<'_>) -> Option<DedicatedImage> {
        self.load_inner(context)
            .await
            .inspect_err(|e| tracing::error!("{}", e))
            .ok()
    }

    fn free(mut output: Self::Output, context: &skid_steer::Context) {
        let ctx: &AssetLoadContext = context.get().unwrap();
        unsafe { output.destroy(ctx.device()) };
    }
}
