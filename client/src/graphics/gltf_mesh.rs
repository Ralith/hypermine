use std::{
    borrow::Cow,
    fs,
    fs::File,
    mem,
    path::{Path, PathBuf},
    ptr,
};

use anyhow::{anyhow, bail, Context, Result};
use ash::vk;
use futures_util::future::{try_join_all, BoxFuture, FutureExt};
use lahar::{BufferRegionAlloc, DedicatedImage};
use tracing::{error, trace};

use super::{meshes::Vertex, Base, Mesh};
use crate::loader::{Cleanup, LoadCtx, LoadFuture, Loadable};

pub struct GlbFile {
    pub path: PathBuf,
}

impl Loadable for GlbFile {
    type Output = GltfScene;

    fn load(self, ctx: &LoadCtx) -> LoadFuture<'_, Self::Output> {
        Box::pin(self.load(ctx))
    }
}

impl GlbFile {
    async fn load(self, ctx: &LoadCtx) -> Result<GltfScene> {
        let path = ctx
            .cfg
            .find_asset(&self.path)
            .ok_or_else(|| anyhow!("{} not found", self.path.display()))?;

        let glb = gltf::Glb::from_reader(
            File::open(&path).with_context(|| format!("opening {}", path.display()))?,
        )
        .with_context(|| format!("reading {}", path.display()))?;
        let gltf = gltf::Document::from_json(
            gltf::json::deserialize::from_slice(&glb.json).context("JSON parsing")?,
        )
        .context("GLTF parsing")?;
        let buffer = glb
            .bin
            .as_ref()
            .ok_or_else(|| anyhow!("missing binary payload"))?;

        let scene = gltf
            .default_scene()
            .ok_or_else(|| anyhow!("no default scene"))?;
        let identity = na::Matrix4::identity();
        let meshes = try_join_all(
            scene
                .nodes()
                .map(|node| load_node(ctx, buffer, &identity, node)),
        )
        .await?
        .into_iter()
        .flatten()
        .collect();
        Ok(GltfScene(meshes))
    }
}

pub struct GltfScene(pub Vec<Mesh>);

impl Cleanup for GltfScene {
    unsafe fn cleanup(self, gfx: &Base) {
        for mesh in self.0 {
            mesh.cleanup(gfx);
        }
    }
}

fn load_node<'a>(
    ctx: &'a LoadCtx,
    buffer: &'a [u8],
    transform: &'a na::Matrix4<f32>,
    node: gltf::Node<'a>,
) -> BoxFuture<'a, Result<Vec<Mesh>>> {
    async move {
        let transform = transform * na::Matrix4::from(node.transform().matrix());
        let (mut local, children) = tokio::try_join!(
            async {
                if let Some(mesh) = node.mesh() {
                    Ok(load_mesh(ctx, buffer, &transform, &mesh).await?)
                } else {
                    Ok(Vec::new())
                }
            },
            try_join_all(
                node.children()
                    .map(|child| load_node(ctx, buffer, &transform, child))
            )
        )?;

        local.extend(children.into_iter().flatten());

        Ok(local)
    }
    .boxed()
}

async fn load_mesh(
    ctx: &LoadCtx,
    buffer: &[u8],
    transform: &na::Matrix4<f32>,
    mesh: &gltf::Mesh<'_>,
) -> Result<Vec<Mesh>> {
    try_join_all(
        mesh.primitives()
            .map(|x| load_primitive(ctx, buffer, transform, x)),
    )
    .await
}

async fn load_primitive(
    ctx: &LoadCtx,
    buffer: &[u8],
    transform: &na::Matrix4<f32>,
    prim: gltf::Primitive<'_>,
) -> Result<Mesh> {
    let device = &*ctx.gfx.device;
    let texcoord_index = prim
        .material()
        .pbr_metallic_roughness()
        .base_color_texture()
        .map(|x| x.tex_coord());

    // Concurrent upload
    // TODO: Don't leak resources on error
    let (geom, color) = tokio::join!(
        load_geom(ctx, buffer, &prim, transform, texcoord_index),
        load_material(ctx, buffer, &prim)
    );
    let geom = geom?;
    let color = color?;

    unsafe {
        let color_view = device
            .create_image_view(
                &vk::ImageViewCreateInfo::builder()
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
        let pool = device
            .create_descriptor_pool(
                &vk::DescriptorPoolCreateInfo::builder()
                    .max_sets(1)
                    .pool_sizes(&[vk::DescriptorPoolSize {
                        ty: vk::DescriptorType::COMBINED_IMAGE_SAMPLER,
                        descriptor_count: 1,
                    }]),
                None,
            )
            .unwrap();
        let ds = device
            .allocate_descriptor_sets(
                &vk::DescriptorSetAllocateInfo::builder()
                    .descriptor_pool(pool)
                    .set_layouts(&[ctx.mesh_ds_layout]),
            )
            .unwrap()[0];
        device.update_descriptor_sets(
            &[vk::WriteDescriptorSet::builder()
                .dst_set(ds)
                .dst_binding(0)
                .descriptor_type(vk::DescriptorType::COMBINED_IMAGE_SAMPLER)
                .image_info(&[vk::DescriptorImageInfo {
                    sampler: vk::Sampler::null(),
                    image_view: color_view,
                    image_layout: vk::ImageLayout::SHADER_READ_ONLY_OPTIMAL,
                }])
                .build()],
            &[],
        );

        Ok(Mesh {
            vertices: geom.vertices,
            indices: geom.indices,
            index_count: geom.index_count,
            pool,
            ds,
            color,
            color_view,
        })
    }
}

struct Geometry {
    vertices: BufferRegionAlloc,
    indices: BufferRegionAlloc,
    index_count: u32,
}

async fn load_geom(
    ctx: &LoadCtx,
    buffer: &[u8],
    prim: &gltf::Primitive<'_>,
    transform: &na::Matrix4<f32>,
    texcoord_index: Option<u32>,
) -> Result<Geometry> {
    let normal_transform = match transform.try_inverse() {
        None => {
            error!("non-invertible transform");
            na::Matrix4::identity()
        }
        Some(x) => x.transpose(),
    };

    let prim = prim.reader(|x| {
        if let gltf::buffer::Source::Bin = x.source() {
            Some(buffer)
        } else {
            None
        }
    });
    let positions = prim
        .read_positions()
        .ok_or_else(|| anyhow!("vertex positions missing"))?;
    let mut texcoords = texcoord_index
        .map(|i| -> Result<_> {
            Ok(prim
                .read_tex_coords(i)
                .ok_or_else(|| anyhow!("texcoords missing"))?
                .into_f32())
        })
        .transpose()?;
    let normals = prim
        .read_normals()
        .ok_or_else(|| anyhow!("normals missing"))?;
    let vertex_count = positions.len();
    if vertex_count != normals.len()
        || texcoords
            .as_ref()
            .map_or(false, |x| vertex_count != x.len())
    {
        bail!("inconsistent vertex attribute counts");
    }
    let byte_size = vertex_count * mem::size_of::<Vertex>();

    let mut v_staging = ctx
        .staging
        .alloc(byte_size)
        .await
        .ok_or_else(|| anyhow!("too large"))?;
    for ((pos, norm), storage) in positions
        .zip(normals)
        .zip(v_staging.chunks_exact_mut(mem::size_of::<Vertex>()))
    {
        let v = Vertex {
            position: na::Point3::from_homogeneous(
                transform * (na::Point3::from(pos)).to_homogeneous(),
            )
            .unwrap_or_else(na::Point3::origin),
            texcoords: texcoords
                .as_mut()
                .map_or_else(na::zero, |x| x.next().unwrap().into()),
            normal: na::Unit::new_normalize(
                (normal_transform * na::Vector3::from(norm).to_homogeneous()).xyz(),
            ),
        };
        // write_unaligned accepts misaligned pointers
        #[allow(clippy::cast_ptr_alignment)]
        unsafe {
            ptr::write_unaligned(storage.as_ptr() as *mut Vertex, v);
        }
    }

    let indices = prim
        .read_indices()
        .ok_or_else(|| anyhow!("indices missing"))?
        .into_u32();
    let index_count = indices.len();
    let mut i_staging = ctx
        .staging
        .alloc(index_count * 4)
        .await
        .ok_or_else(|| anyhow!("too large"))?;
    for (idx, storage) in indices.zip(i_staging.chunks_exact_mut(4)) {
        storage.copy_from_slice(&idx.to_ne_bytes());
    }

    let vert_alloc =
        ctx.vertex_alloc
            .lock()
            .unwrap()
            .alloc(&ctx.gfx.device, byte_size as vk::DeviceSize, 4);
    let staging_buffer = ctx.staging.buffer();
    let vert_buffer = vert_alloc.buffer;
    let vert_src_offset = v_staging.offset();
    let vert_dst_offset = vert_alloc.offset;
    let vertex_upload = unsafe {
        ctx.transfer.run(move |xf, cmd| {
            xf.device.cmd_copy_buffer(
                cmd,
                staging_buffer,
                vert_buffer,
                &[vk::BufferCopy {
                    src_offset: vert_src_offset,
                    dst_offset: vert_dst_offset,
                    size: byte_size as vk::DeviceSize,
                }],
            );
            xf.stages |= vk::PipelineStageFlags::VERTEX_INPUT;
            xf.buffer_barriers.push(
                vk::BufferMemoryBarrier::builder()
                    .src_access_mask(vk::AccessFlags::TRANSFER_WRITE)
                    .dst_access_mask(vk::AccessFlags::VERTEX_ATTRIBUTE_READ)
                    .src_queue_family_index(xf.queue_family)
                    .dst_queue_family_index(xf.dst_queue_family)
                    .buffer(vert_buffer)
                    .offset(vert_dst_offset)
                    .size(byte_size as vk::DeviceSize)
                    .build(),
            );
        })
    };

    let idx_alloc = ctx.index_alloc.lock().unwrap().alloc(
        &ctx.gfx.device,
        index_count as vk::DeviceSize * 4,
        4,
    );
    let idx_buffer = idx_alloc.buffer;
    let idx_src_offset = i_staging.offset();
    let idx_dst_offset = idx_alloc.offset;
    let index_upload = unsafe {
        ctx.transfer.run(move |xf, cmd| {
            xf.device.cmd_copy_buffer(
                cmd,
                staging_buffer,
                idx_buffer,
                &[vk::BufferCopy {
                    src_offset: idx_src_offset,
                    dst_offset: idx_dst_offset,
                    size: index_count as vk::DeviceSize * 4,
                }],
            );
            xf.stages |= vk::PipelineStageFlags::VERTEX_INPUT;
            xf.buffer_barriers.push(
                vk::BufferMemoryBarrier::builder()
                    .src_access_mask(vk::AccessFlags::TRANSFER_WRITE)
                    .dst_access_mask(vk::AccessFlags::INDEX_READ)
                    .src_queue_family_index(xf.queue_family)
                    .dst_queue_family_index(xf.dst_queue_family)
                    .buffer(idx_buffer)
                    .offset(idx_dst_offset)
                    .size(index_count as vk::DeviceSize * 4)
                    .build(),
            );
        })
    };
    // Upload concurrently
    let (r1, r2) = tokio::join!(vertex_upload, index_upload);
    r1?;
    r2?;
    Ok(Geometry {
        vertices: vert_alloc,
        indices: idx_alloc,
        index_count: index_count as u32,
    })
}

async fn load_material(
    ctx: &LoadCtx,
    buffer: &[u8],
    prim: &gltf::Primitive<'_>,
) -> Result<DedicatedImage> {
    let device = &*ctx.gfx.device;
    let color = match prim
        .material()
        .pbr_metallic_roughness()
        .base_color_texture()
    {
        None => {
            return load_solid_color(
                ctx,
                prim.material().pbr_metallic_roughness().base_color_factor(),
            )
            .await
        }
        Some(x) => x,
    };
    let color_data = match color.texture().source().source() {
        gltf::image::Source::Uri { uri, .. } => {
            let path = ctx
                .cfg
                .find_asset(Path::new(uri))
                .ok_or_else(|| anyhow!("texture {} not found", uri))?;
            trace!(path = %path.display(), "reading texture");
            Cow::Owned(fs::read(&path).context("reading texture")?)
        }
        gltf::image::Source::View { view, .. } => {
            match view.buffer().source() {
                gltf::buffer::Source::Bin => {}
                gltf::buffer::Source::Uri(_) => {
                    bail!("external buffers unsupported");
                }
            }
            Cow::Borrowed(&buffer[view.offset()..view.offset() + view.length()])
        }
    };
    let mut color_data = &color_data[..];
    let mut color_reader = png::Decoder::new(&mut color_data)
        .read_info()
        .with_context(|| "decoding PNG header")?;
    let (width, height) = {
        let info = color_reader.info();
        (info.width, info.height)
    };
    let mut color_staging = ctx
        .staging
        .alloc(width as usize * height as usize * 4)
        .await
        .ok_or_else(|| anyhow!("texture too large"))?;
    color_reader
        .next_frame(&mut color_staging)
        .with_context(|| "decoding PNG data")?;
    let color = unsafe {
        DedicatedImage::new(
            device,
            &ctx.gfx.memory_properties,
            &vk::ImageCreateInfo::builder()
                .image_type(vk::ImageType::TYPE_2D)
                .format(vk::Format::R8G8B8A8_SRGB)
                .extent(vk::Extent3D {
                    width,
                    height,
                    depth: 1,
                })
                .mip_levels(1)
                .array_layers(1)
                .samples(vk::SampleCountFlags::TYPE_1)
                .usage(vk::ImageUsageFlags::SAMPLED | vk::ImageUsageFlags::TRANSFER_DST),
        )
    };
    let staging_buffer = ctx.staging.buffer();
    let color_handle = color.handle;
    let color_offset = color_staging.offset();
    unsafe {
        ctx.transfer
            .run(move |xf, cmd| {
                let range = vk::ImageSubresourceRange {
                    aspect_mask: vk::ImageAspectFlags::COLOR,
                    base_mip_level: 0,
                    level_count: 1,
                    base_array_layer: 0,
                    layer_count: 1,
                };
                xf.device.cmd_pipeline_barrier(
                    cmd,
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
                        .image(color_handle)
                        .subresource_range(range)
                        .build()],
                );
                xf.device.cmd_copy_buffer_to_image(
                    cmd,
                    staging_buffer,
                    color_handle,
                    vk::ImageLayout::TRANSFER_DST_OPTIMAL,
                    &[vk::BufferImageCopy {
                        buffer_offset: color_offset,
                        image_subresource: vk::ImageSubresourceLayers {
                            aspect_mask: vk::ImageAspectFlags::COLOR,
                            mip_level: 0,
                            base_array_layer: 0,
                            layer_count: 1,
                        },
                        image_extent: vk::Extent3D {
                            width,
                            height,
                            depth: 1,
                        },
                        ..Default::default()
                    }],
                );
                xf.stages |= vk::PipelineStageFlags::FRAGMENT_SHADER;
                xf.image_barriers.push(
                    vk::ImageMemoryBarrier::builder()
                        .src_access_mask(vk::AccessFlags::TRANSFER_WRITE)
                        .dst_access_mask(vk::AccessFlags::SHADER_READ)
                        .src_queue_family_index(xf.queue_family)
                        .dst_queue_family_index(xf.dst_queue_family)
                        .old_layout(vk::ImageLayout::TRANSFER_DST_OPTIMAL)
                        .new_layout(vk::ImageLayout::SHADER_READ_ONLY_OPTIMAL)
                        .image(color_handle)
                        .subresource_range(range)
                        .build(),
                );
            })
            .await?;
    }
    Ok(color)
}

async fn load_solid_color(ctx: &LoadCtx, rgba: [f32; 4]) -> Result<DedicatedImage> {
    unsafe {
        let image = DedicatedImage::new(
            &ctx.gfx.device,
            &ctx.gfx.memory_properties,
            &vk::ImageCreateInfo::builder()
                .image_type(vk::ImageType::TYPE_2D)
                .format(vk::Format::R8G8B8A8_SRGB)
                .extent(vk::Extent3D {
                    width: 1,
                    height: 1,
                    depth: 1,
                })
                .mip_levels(1)
                .array_layers(1)
                .samples(vk::SampleCountFlags::TYPE_1)
                .usage(vk::ImageUsageFlags::SAMPLED | vk::ImageUsageFlags::TRANSFER_DST),
        );
        let handle = image.handle;
        ctx.transfer
            .run(move |xf, cmd| {
                let range = vk::ImageSubresourceRange {
                    aspect_mask: vk::ImageAspectFlags::COLOR,
                    base_mip_level: 0,
                    level_count: 1,
                    base_array_layer: 0,
                    layer_count: 1,
                };
                xf.device.cmd_pipeline_barrier(
                    cmd,
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
                        .image(handle)
                        .subresource_range(range)
                        .build()],
                );
                xf.device.cmd_clear_color_image(
                    cmd,
                    handle,
                    vk::ImageLayout::TRANSFER_DST_OPTIMAL,
                    &vk::ClearColorValue { float32: rgba },
                    &[range],
                );
                xf.stages |= vk::PipelineStageFlags::FRAGMENT_SHADER;
                xf.image_barriers.push(
                    vk::ImageMemoryBarrier::builder()
                        .src_access_mask(vk::AccessFlags::TRANSFER_WRITE)
                        .dst_access_mask(vk::AccessFlags::SHADER_READ)
                        .src_queue_family_index(xf.queue_family)
                        .dst_queue_family_index(xf.dst_queue_family)
                        .old_layout(vk::ImageLayout::TRANSFER_DST_OPTIMAL)
                        .new_layout(vk::ImageLayout::SHADER_READ_ONLY_OPTIMAL)
                        .image(handle)
                        .subresource_range(range)
                        .build(),
                );
            })
            .await?;
        Ok(image)
    }
}
