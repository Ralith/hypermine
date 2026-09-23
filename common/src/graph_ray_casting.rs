use crate::{
    chunk_ray_casting::{ChunkCastHit, chunk_ray_cast},
    collision_math::Ray,
    graph::Graph,
    node::{Chunk, ChunkId, VoxelData},
    proto::Position,
    traversal::RayTraverser,
    voxel_math::{CoordAxis, CoordSign, Coords},
    world::Material,
};

/// Performs ray casting against the voxels in the `DualGraph`
///
/// The `ray` parameter and any resulting hit normals are given in the local coordinate system of `position`.
///
/// The `tanh_distance` is the hyperbolic tangent of the cast_distance, or the distance along the ray to check for hits.
pub fn ray_cast(
    graph: &Graph,
    position: &Position,
    ray: &Ray,
    mut tanh_distance: f32,
) -> Result<Option<GraphCastHit>, OutOfBounds> {
    let fallback_voxel_data = VoxelData::Solid(Material::Dirt);

    // A ray cast is assumed to be a miss until a collision is found.
    // This `hit` variable gets updated over time before being returned.
    let mut hit: Option<PossibleGraphCastHit> = None;

    let mut traverser = RayTraverser::new(graph, *position, ray, 0.0);
    while let Some((chunk, transform)) = traverser.next(tanh_distance) {
        let (voxel_data, in_bounds) = match chunk.map(|c| &graph[c]) {
            Some(Chunk::Populated {
                voxels: voxel_data, ..
            }) => (voxel_data, true),
            _ => (&fallback_voxel_data, false),
        };

        hit = chunk_ray_cast(
            voxel_data,
            graph.layout(),
            &(transform * ray),
            tanh_distance,
        )
        .map_or(hit, |hit| {
            tanh_distance = hit.tanh_distance;
            Some(PossibleGraphCastHit {
                inner: hit,
                chunk,
                in_bounds,
            })
        });
    }

    let Some(hit) = hit else {
        return Ok(None);
    };
    if !hit.in_bounds {
        return Err(OutOfBounds);
    }
    Ok(Some(GraphCastHit {
        tanh_distance: hit.inner.tanh_distance,
        chunk: hit.chunk.expect("in bounds"),
        voxel_coords: hit.inner.voxel_coords,
        face_axis: hit.inner.face_axis,
        face_sign: hit.inner.face_sign,
    }))
}

#[derive(Debug)]
pub struct OutOfBounds;

/// Information about the intersection at the end of a ray segment.
#[derive(Debug)]
pub struct GraphCastHit {
    /// The tanh of the distance traveled along the ray to result in this hit.
    pub tanh_distance: f32,

    /// Which chunk in the graph the hit occurred in
    pub chunk: ChunkId,

    /// The coordinates of the block that was hit, including margins.
    pub voxel_coords: Coords,

    /// Which of the three axes is orthogonal to the face of the block that was hit.
    pub face_axis: CoordAxis,

    /// The direction along `face_axis` corresponding to the outside of the face that was hit.
    pub face_sign: CoordSign,
}

/// Information about a discovered intersection at the end of a ray segment. May be revised
/// if a closer intersection is discovered.
///
/// For ray casts against graphs with convex chunks, the first discovered collision should be
/// the closest collision, as traversals are designed to be in order, but we do not rely on
/// this assumption, as it could be brittle.. For instance, shape casting cannot use
/// this assumption.
struct PossibleGraphCastHit {
    /// Data returned when performing a collision query within the chunk
    inner: ChunkCastHit,

    /// Which chunk in the graph the hit occurred in, or `None` if the hit was outside the graph
    chunk: Option<ChunkId>,

    /// Whether the hit was into a generated region. If `false`, the collision result is ambiguous
    /// because the ray went out of bounds
    in_bounds: bool,
}
