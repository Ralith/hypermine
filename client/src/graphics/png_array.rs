use std::{fs, fs::File, path::PathBuf};

use anyhow::{anyhow, bail, Context};
use ash::vk;
use lahar::DedicatedImage;
use tracing::trace;

use super::{loader, LoadFuture, Loadable};

pub struct PngArray {
    pub path: PathBuf,
    pub size: usize,
}

impl Loadable for PngArray {
    type Output = DedicatedImage;

    fn load<'a>(self, handle: &'a loader::Handle) -> LoadFuture<'a, Self::Output> {
        Box::pin(async move {
            let mut paths = fs::read_dir(&self.path)
                .with_context(|| format!("reading {}", self.path.display()))?
                .map(|x| x.map(|x| x.path()))
                .collect::<Result<Vec<_>, _>>()
                .with_context(|| format!("reading {}", self.path.display()))?;
            if paths.is_empty() {
                bail!("{} is empty", self.path.display());
            }
            if paths.len() < self.size {
                bail!(
                    "{}: expected {} textures, found {}",
                    self.path.display(),
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
                let (info, mut reader) = decoder
                    .read_info()
                    .with_context(|| format!("decoding {}", path.display()))?;
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
                                anyhow!("{}: image array too large", self.path.display())
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

                mem.flush();
                handle
                    .transfer
                    .upload_image(
                        handle.staging.buffer(),
                        image.handle,
                        vk::BufferImageCopy {
                            buffer_offset: mem.offset(),
                            image_subresource: vk::ImageSubresourceLayers {
                                aspect_mask: vk::ImageAspectFlags::COLOR,
                                mip_level: 0,
                                base_array_layer: 0,
                                layer_count: self.size as u32,
                            },
                            image_extent: vk::Extent3D {
                                width,
                                height,
                                depth: 1,
                            },
                            ..Default::default()
                        },
                        true,
                    )
                    .await?;
                trace!(
                    width = width,
                    height = height,
                    path = %self.path.display(),
                    "loaded array"
                );
                Ok(image)
            }
        })
    }
}
