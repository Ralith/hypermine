use crate::{
    dodeca::Vertex,
    graph::Graph,
    math,
    node::{Chunk, ChunkId, VoxelData},
    voxel_math::{ChunkAxisPermutation, ChunkDirection, CoordAxis, CoordSign, Coords},
    world::Material,
};

/// Updates the margins of both `voxels` and `neighbor_voxels` at the side they meet at.
/// It is assumed that `voxels` corresponds to a chunk that lies at `vertex` and that
/// `neighbor_voxels` is at direction `direction` from `voxels`.
pub fn fix_margins(
    dimension: u8,
    vertex: Vertex,
    voxels: &mut VoxelData,
    direction: ChunkDirection,
    neighbor_voxels: &mut VoxelData,
) {
    let neighbor_axis_permutation = neighbor_axis_permutation(vertex, direction);

    let margin_coord = CoordsWithMargins::margin_coord(dimension, direction.sign);
    let boundary_coord = CoordsWithMargins::boundary_coord(dimension, direction.sign);
    let voxel_data = voxels.data_mut(dimension);
    let neighbor_voxel_data = neighbor_voxels.data_mut(dimension);
    for j in 0..dimension {
        for i in 0..dimension {
            // Determine coordinates of the boundary voxel (to read from) and the margin voxel (to write to)
            // in voxel_data's perspective. To convert to neighbor_voxel_data's perspective, left-multiply
            // by neighbor_axis_permutation.
            let coords_of_boundary_voxel = CoordsWithMargins(math::tuv_to_xyz(
                direction.axis as usize,
                [boundary_coord, i + 1, j + 1],
            ));
            let coords_of_margin_voxel = CoordsWithMargins(math::tuv_to_xyz(
                direction.axis as usize,
                [margin_coord, i + 1, j + 1],
            ));

            // Use neighbor_voxel_data to set margins of voxel_data
            voxel_data[coords_of_margin_voxel.to_index(dimension)] = neighbor_voxel_data
                [(neighbor_axis_permutation * coords_of_boundary_voxel).to_index(dimension)];

            // Use voxel_data to set margins of neighbor_voxel_data
            neighbor_voxel_data
                [(neighbor_axis_permutation * coords_of_margin_voxel).to_index(dimension)] =
                voxel_data[coords_of_boundary_voxel.to_index(dimension)];
        }
    }
}

/// Updates the margins of a given VoxelData to match the voxels they're next to. This is a good assumption to start
/// with before taking into account neighboring chunks because it means that no surface will be present on the boundaries
/// of the chunk, resulting in the least rendering. This is also generally accurate when the neighboring chunks are solid.
pub fn initialize_margins(dimension: u8, voxels: &mut VoxelData) {
    // If voxels is solid, the margins are already set up the way they should be.
    if voxels.is_solid() {
        return;
    }

    for direction in ChunkDirection::iter() {
        let margin_coord = CoordsWithMargins::margin_coord(dimension, direction.sign);
        let boundary_coord = CoordsWithMargins::boundary_coord(dimension, direction.sign);
        let chunk_data = voxels.data_mut(dimension);
        for j in 0..dimension {
            for i in 0..dimension {
                // Determine coordinates of the boundary voxel (to read from) and the margin voxel (to write to).
                let coords_of_boundary_voxel = CoordsWithMargins(math::tuv_to_xyz(
                    direction.axis as usize,
                    [boundary_coord, i + 1, j + 1],
                ));
                let coords_of_margin_voxel = CoordsWithMargins(math::tuv_to_xyz(
                    direction.axis as usize,
                    [margin_coord, i + 1, j + 1],
                ));

                chunk_data[coords_of_margin_voxel.to_index(dimension)] =
                    chunk_data[coords_of_boundary_voxel.to_index(dimension)];
            }
        }
    }
}

/// Assuming that the voxel at `chunk` and `coords` is set to `material`, updates the cooresponding
/// margin in the chunk at direction `direction` from `chunk` if such a margin exists. Unpopulated chunks
/// are ignored.
pub fn update_margin_voxel(
    graph: &mut Graph,
    chunk: ChunkId,
    coords: Coords,
    direction: ChunkDirection,
    material: Material,
) {
    let coords: CoordsWithMargins = coords.into();
    let dimension = graph.layout().dimension();
    let boundary_coord = match direction.sign {
        CoordSign::Plus => dimension,
        CoordSign::Minus => 1,
    };
    if coords[direction.axis] != boundary_coord {
        // There is nothing to do if we're not on an boundary voxel.
        return;
    }
    let Some(Chunk::Populated {
        voxels: neighbor_voxels,
        surface: neighbor_surface,
        old_surface: neighbor_old_surface,
        ..
    }) = graph
        .get_chunk_neighbor(chunk, direction.axis, direction.sign)
        .map(|chunk_id| &mut graph[chunk_id])
    else {
        // If the neighboring chunk to check is not populated, there is nothing to do.
        return;
    };

    let margin_coord = match direction.sign {
        CoordSign::Plus => dimension + 1,
        CoordSign::Minus => 0,
    };
    let neighbor_axis_permutation = neighbor_axis_permutation(chunk.vertex, direction);
    let mut neighbor_coords = coords;
    neighbor_coords[direction.axis] = margin_coord;
    neighbor_coords = neighbor_axis_permutation * neighbor_coords;

    neighbor_voxels.data_mut(dimension)[neighbor_coords.to_index(dimension)] = material;
    *neighbor_old_surface = neighbor_surface.take().or(*neighbor_old_surface);
}

fn neighbor_axis_permutation(vertex: Vertex, direction: ChunkDirection) -> ChunkAxisPermutation {
    match direction.sign {
        CoordSign::Plus => vertex.chunk_axis_permutations()[direction.axis as usize],
        CoordSign::Minus => ChunkAxisPermutation::IDENTITY,
    }
}

/// Coordinates for a discrete voxel within a chunk, including margins
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct CoordsWithMargins(pub [u8; 3]);

impl CoordsWithMargins {
    /// Returns the array index in `VoxelData` corresponding to these coordinates
    pub fn to_index(self, chunk_size: u8) -> usize {
        let chunk_size_with_margin = chunk_size as usize + 2;
        (self.0[0] as usize)
            + (self.0[1] as usize) * chunk_size_with_margin
            + (self.0[2] as usize) * chunk_size_with_margin.pow(2)
    }

    /// Returns the x, y, or z coordinate that would correspond to the margin in the direction of `sign`
    pub fn margin_coord(chunk_size: u8, sign: CoordSign) -> u8 {
        match sign {
            CoordSign::Plus => chunk_size + 1,
            CoordSign::Minus => 0,
        }
    }

    /// Returns the x, y, or z coordinate that would correspond to the voxel meeting the chunk boundary in the direction of `sign`
    pub fn boundary_coord(chunk_size: u8, sign: CoordSign) -> u8 {
        match sign {
            CoordSign::Plus => chunk_size,
            CoordSign::Minus => 1,
        }
    }
}

impl From<Coords> for CoordsWithMargins {
    #[inline]
    fn from(value: Coords) -> Self {
        CoordsWithMargins([value.0[0] + 1, value.0[1] + 1, value.0[2] + 1])
    }
}

impl std::ops::Index<CoordAxis> for CoordsWithMargins {
    type Output = u8;

    #[inline]
    fn index(&self, coord_axis: CoordAxis) -> &u8 {
        self.0.index(coord_axis as usize)
    }
}

impl std::ops::IndexMut<CoordAxis> for CoordsWithMargins {
    #[inline]
    fn index_mut(&mut self, coord_axis: CoordAxis) -> &mut u8 {
        self.0.index_mut(coord_axis as usize)
    }
}

impl std::ops::Mul<CoordsWithMargins> for ChunkAxisPermutation {
    type Output = CoordsWithMargins;

    fn mul(self, rhs: CoordsWithMargins) -> Self::Output {
        let mut result = CoordsWithMargins([0; 3]);
        for axis in CoordAxis::iter() {
            result[self[axis]] = rhs[axis];
        }
        result
    }
}

#[cfg(test)]
mod tests {
    use crate::{dodeca::Vertex, graph::NodeId, node, voxel_math::Coords, world::Material};

    use super::*;

    #[test]
    fn test_fix_margins() {
        // This test case can set up empirically by placing blocks and printing their coordinates to confirm which
        // coordinates are adjacent to each other.

        // `voxels` lives at vertex F
        let mut voxels = VoxelData::Solid(Material::Void);
        voxels.data_mut(12)[Coords([11, 2, 10]).to_index(12)] = Material::WoodPlanks;

        // `neighbor_voxels` lives at vertex J
        let mut neighbor_voxels = VoxelData::Solid(Material::Void);
        neighbor_voxels.data_mut(12)[Coords([2, 10, 11]).to_index(12)] = Material::Grass;

        // Sanity check that voxel adjacencies are as expected. If the test fails here, it's likely that "dodeca.rs" was
        // redesigned, and the test itself will have to be fixed, rather than the code being tested.
        assert_eq!(Vertex::F.adjacent_vertices()[0], Vertex::J);
        assert_eq!(Vertex::J.adjacent_vertices()[2], Vertex::F);

        // Sanity check that voxels are populated as expected, using `CoordsWithMargins` for consistency with the actual
        // test case.
        assert_eq!(
            voxels.get(CoordsWithMargins([12, 3, 11]).to_index(12)),
            Material::WoodPlanks
        );
        assert_eq!(
            neighbor_voxels.get(CoordsWithMargins([3, 11, 12]).to_index(12)),
            Material::Grass
        );

        fix_margins(
            12,
            Vertex::F,
            &mut voxels,
            ChunkDirection::PLUS_X,
            &mut neighbor_voxels,
        );

        // Actual verification: Check that the margins were set correctly
        assert_eq!(
            voxels.get(CoordsWithMargins([13, 3, 11]).to_index(12)),
            Material::Grass
        );
        assert_eq!(
            neighbor_voxels.get(CoordsWithMargins([3, 11, 13]).to_index(12)),
            Material::WoodPlanks
        );
    }

    #[test]
    fn test_initialize_margins() {
        let mut voxels = VoxelData::Solid(Material::Void);
        voxels.data_mut(12)[Coords([11, 2, 10]).to_index(12)] = Material::WoodPlanks;
        assert_eq!(
            voxels.get(CoordsWithMargins([12, 3, 11]).to_index(12)),
            Material::WoodPlanks
        );

        initialize_margins(12, &mut voxels);

        assert_eq!(
            voxels.get(CoordsWithMargins([13, 3, 11]).to_index(12)),
            Material::WoodPlanks
        );
    }

    #[test]
    fn test_update_margin_voxel() {
        let mut graph = Graph::new(12);
        let current_vertex = Vertex::A;
        let neighbor_vertex = current_vertex.adjacent_vertices()[1];
        let neighbor_node =
            graph.ensure_neighbor(NodeId::ROOT, current_vertex.canonical_sides()[0]);
        node::populate_fresh_nodes(&mut graph);

        // These are the chunks this test will work with.
        let current_chunk = ChunkId::new(NodeId::ROOT, current_vertex);
        let node_neighbor_chunk = ChunkId::new(neighbor_node, current_vertex);
        let vertex_neighbor_chunk = ChunkId::new(NodeId::ROOT, neighbor_vertex);

        // Populate relevant chunks with void
        for chunk in [current_chunk, node_neighbor_chunk, vertex_neighbor_chunk] {
            *graph.get_chunk_mut(chunk).unwrap() = Chunk::Populated {
                voxels: VoxelData::Solid(Material::Void),
                modified: false,
                surface: None,
                old_surface: None,
            };
        }

        // Update and check the margins of node_neighbor_chunk
        update_margin_voxel(
            &mut graph,
            current_chunk,
            Coords([0, 7, 9]),
            ChunkDirection::MINUS_X,
            Material::WoodPlanks,
        );
        let Chunk::Populated {
            voxels: node_neighbor_voxels,
            ..
        } = graph.get_chunk_mut(node_neighbor_chunk).unwrap()
        else {
            panic!("node_neighbor_chunk should have just been populated by this test");
        };
        assert_eq!(
            node_neighbor_voxels.get(CoordsWithMargins([0, 8, 10]).to_index(12)),
            Material::WoodPlanks
        );

        // Update and check the margins of vertex_neighbor_chunk
        update_margin_voxel(
            &mut graph,
            current_chunk,
            Coords([5, 11, 9]),
            ChunkDirection::PLUS_Y,
            Material::Grass,
        );
        let Chunk::Populated {
            voxels: vertex_neighbor_voxels,
            ..
        } = graph.get_chunk_mut(vertex_neighbor_chunk).unwrap()
        else {
            panic!("vertex_neighbor_chunk should have just been populated by this test");
        };
        assert_eq!(
            vertex_neighbor_voxels.get(CoordsWithMargins([6, 10, 13]).to_index(12)),
            Material::Grass
        );
    }
}
