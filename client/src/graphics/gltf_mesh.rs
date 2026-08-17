use std::{
    borrow::Cow,
    fs::{self, File},
    io::Cursor,
    path::{Path, PathBuf},
};

use anyhow::{Context, Result, anyhow, bail};
use color::{AlphaColor, LinearSrgb};
use common::Anonymize;
use futures_util::future::{FutureExt, LocalBoxFuture, try_join_all};
use tracing::{error, trace};

use super::{Mesh, meshes::Vertex};
use crate::graphics::{
    asset_loader::AssetLoadContext,
    meshes::{MeshGeometryDefinition, MeshMaterialDefinition},
};

pub struct GlbFile {
    pub path: PathBuf,
}

impl skid_steer::Source for GlbFile {
    type Output = GltfScene;

    async fn load<'a>(self, context: &'a skid_steer::Context<'a>) -> Option<Self::Output> {
        let ctx: &AssetLoadContext = context.get().unwrap();
        self.load_inner(ctx)
            .await
            .inspect_err(|e| tracing::error!("{}", e))
            .ok()
    }

    fn free(mut output: Self::Output, context: &skid_steer::Context) {
        let ctx: &AssetLoadContext = context.get().unwrap();
        unsafe {
            output.destroy(ctx.device());
        }
    }
}

impl GlbFile {
    async fn load_inner(self, ctx: &AssetLoadContext) -> Result<GltfScene> {
        let path = ctx
            .find_asset(&self.path)
            .ok_or_else(|| anyhow!("{} not found", self.path.anonymize().display()))?;

        let glb = gltf::Glb::from_reader(
            File::open(&path).with_context(|| format!("opening {}", path.anonymize().display()))?,
        )
        .with_context(|| format!("reading {}", path.anonymize().display()))?;
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

impl GltfScene {
    unsafe fn destroy(&mut self, device: &ash::Device) {
        unsafe {
            for mesh in &mut self.0 {
                mesh.destroy(device);
            }
        }
    }
}

fn load_node<'a>(
    ctx: &'a AssetLoadContext,
    buffer: &'a [u8],
    transform: &'a na::Matrix4<f32>,
    node: gltf::Node<'a>,
) -> LocalBoxFuture<'a, Result<Vec<Mesh>>> {
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
    .boxed_local()
}

async fn load_mesh(
    ctx: &AssetLoadContext,
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
    ctx: &AssetLoadContext,
    buffer: &[u8],
    transform: &na::Matrix4<f32>,
    prim: gltf::Primitive<'_>,
) -> Result<Mesh> {
    let texcoord_index = prim
        .material()
        .pbr_metallic_roughness()
        .base_color_texture()
        .map(|x| x.tex_coord());

    // Concurrent upload
    // TODO: Don't leak resources on error
    let (geom, color) = tokio::join!(
        load_geom(buffer, &prim, transform, texcoord_index),
        load_material(ctx, buffer, &prim)
    );
    let geom = geom?;
    let color = color?;
    Ok(Mesh::from_definition(ctx, geom, color).await)
}

async fn load_geom(
    buffer: &[u8],
    prim: &gltf::Primitive<'_>,
    transform: &na::Matrix4<f32>,
    texcoord_index: Option<u32>,
) -> Result<MeshGeometryDefinition> {
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
    let texcoords = texcoord_index
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
    if vertex_count != normals.len() || texcoords.as_ref().is_some_and(|x| vertex_count != x.len())
    {
        bail!("inconsistent vertex attribute counts");
    }
    let vertices: Vec<_> = positions
        .zip(normals)
        .zip(
            texcoords
                .into_iter()
                .flatten()
                .chain(std::iter::repeat([0.0, 0.0])),
        )
        .map(|((position, normal), texcoords)| Vertex {
            position: na::Point3::from_homogeneous(
                transform * (na::Point3::from(position)).to_homogeneous(),
            )
            .unwrap_or_else(na::Point3::origin),
            normal: na::Unit::new_normalize(
                (normal_transform * na::Vector3::from(normal).to_homogeneous()).xyz(),
            ),
            texcoords: texcoords.into(),
        })
        .collect();
    let indices: Vec<_> = prim
        .read_indices()
        .ok_or_else(|| anyhow!("indices missing"))?
        .into_u32()
        .collect();
    Ok(MeshGeometryDefinition { vertices, indices })
}

async fn load_material(
    ctx: &AssetLoadContext,
    buffer: &[u8],
    prim: &gltf::Primitive<'_>,
) -> Result<MeshMaterialDefinition> {
    let color = match prim
        .material()
        .pbr_metallic_roughness()
        .base_color_texture()
    {
        None => {
            return Ok(MeshMaterialDefinition {
                width: 1,
                height: 1,
                srgb_rgba_color_data: AlphaColor::<LinearSrgb>::new(
                    prim.material().pbr_metallic_roughness().base_color_factor(),
                )
                .to_rgba8()
                .to_u8_array()
                .to_vec(),
            });
        }
        Some(x) => x,
    };
    if prim.material().pbr_metallic_roughness().base_color_factor() != [1.0, 1.0, 1.0, 1.0] {
        tracing::warn!(
            "Ignoring base color factor {:?}, as this setting is currently only supported for GLTF materials without color textures.",
            prim.material().pbr_metallic_roughness().base_color_factor()
        );
    }
    let color_data = match color.texture().source().source() {
        gltf::image::Source::Uri { uri, .. } => {
            let path = ctx
                .find_asset(Path::new(uri))
                .ok_or_else(|| anyhow!("texture {} not found", uri))?;
            trace!(path = %path.anonymize().display(), "reading texture");
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
    let mut color_reader = png::Decoder::new(Cursor::new(&mut color_data))
        .read_info()
        .with_context(|| "decoding PNG header")?;
    let (width, height) = {
        let info = color_reader.info();
        (info.width, info.height)
    };
    let mut image_data = vec![0; width as usize * height as usize * 4];
    color_reader
        .next_frame(&mut image_data)
        .with_context(|| "decoding PNG data")?;
    Ok(MeshMaterialDefinition {
        width,
        height,
        srgb_rgba_color_data: image_data,
    })
}
