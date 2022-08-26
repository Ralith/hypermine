use crate::node::DualGraph;
use crate::{dodeca::Vertex, graph::NodeId};
use std::fmt;

/*
The set of voxels that a collision body covers within a chunk
*/
#[derive(Clone, Copy)]
pub struct ChunkBoundingBox {
    pub node: NodeId,
    pub chunk: Vertex,
    pub min_xyz: na::Vector3<u32>,
    pub max_xyz: na::Vector3<u32>,
    pub dimension: u8,
}

pub struct VoxelAddress {
    pub node: NodeId,
    pub chunk: Vertex,
    pub index: u32,
}

/*
The set of voxels that a collision body covers.
*/
pub struct BoundingBox {
    pub bounding_boxes: Vec<ChunkBoundingBox>,
}

/// from a node coordinate in an arbitrary node, returns the chunk the point would reside in.
/// Does not make any attempt to verify the location is within the node in question.
/// guaranteed to return something if this precondition is met.
fn chunk_from_location(location: na::Vector4<f64>) -> Vertex {
    let epsilon = 0.001;
    for v in Vertex::iter() {
        let pos = (v.node_to_chunk() * location).xyz();
        if (pos.x >= -epsilon) && (pos.y >= -epsilon) && (pos.z >= -epsilon) {
            return v;
        }
    }
    panic!();
}

// The pattern I use to unwrap the results of neighbor is kind of awkward.
impl BoundingBox {
    pub fn create_aabb(
        start_node: NodeId,
        position: na::Vector4<f64>,
        radius: f64,
        graph: &DualGraph,
        dimension: u8,
    ) -> Self {
        assert!(
            radius <= 1.0,
            "Error: the radius of a bounding box may not exceed 1 absolute unit."
        );

        let start_chunk = chunk_from_location(position);
        let mut bounding_boxes = Vec::<ChunkBoundingBox>::new();
        let sides = start_chunk.canonical_sides();

        // get BBs for the chunks within the node.
        for v in Vertex::iter() {
            Self::add_sub_bb(
                &mut bounding_boxes,
                ChunkBoundingBox::get_chunk_bounding_box(
                    start_node, v, position, radius, dimension,
                ),
            );
        }

        // get BBs for foreign chunks sharing a dodeca side
        for &side in sides.iter() {
            if let Some(node) = graph.neighbor(start_node, side) {
                let translated_position = side.reflection() * position;
                for v in side.vertices().iter() {
                    Self::add_sub_bb(
                        &mut bounding_boxes,
                        ChunkBoundingBox::get_chunk_bounding_box(
                            node,
                            *v,
                            translated_position,
                            radius,
                            dimension,
                        ),
                    );
                }
            }
        }

        if let Some(opposite_node) = graph.opposing_node(start_node, start_chunk) {
            let opposite_position =
                sides[0].reflection() * sides[1].reflection() * sides[2].reflection() * position;

            // get BBs for the chunks sharing an edge
            for &side in sides.iter() {
                let node = graph.neighbor(opposite_node, side);
                let translated_position = side.reflection() * opposite_position;
                // the chunk next to the origin chunk
                Self::add_sub_bb(
                    &mut bounding_boxes,
                    ChunkBoundingBox::get_chunk_bounding_box(
                        node.unwrap(),
                        start_chunk,
                        translated_position,
                        radius,
                        dimension,
                    ),
                );
                // the chunk on the opposite side of the edge
                Self::add_sub_bb(
                    &mut bounding_boxes,
                    ChunkBoundingBox::get_chunk_bounding_box(
                        node.unwrap(),
                        side.perpendicular_vertex(start_chunk).unwrap(),
                        translated_position,
                        radius,
                        dimension,
                    ),
                );
            }

            // get BB for the chunk sharing only a vertex.
            Self::add_sub_bb(
                &mut bounding_boxes,
                ChunkBoundingBox::get_chunk_bounding_box(
                    opposite_node,
                    start_chunk,
                    opposite_position,
                    radius,
                    dimension,
                ),
            );
        }

        BoundingBox { bounding_boxes }
    }

    // adds a sub-bounding box to a list if it exists
    fn add_sub_bb(list: &mut Vec<ChunkBoundingBox>, result: Option<ChunkBoundingBox>) {
        if let Some(x) = result {
            list.push(x);
        }
    }

    pub fn every_voxel_address(&self) -> impl Iterator<Item = VoxelAddress> + '_ {
        self.bounding_boxes.iter().flat_map(|cbb| {
            cbb.every_voxel().map(move |index| VoxelAddress {
                node: cbb.node,
                chunk: cbb.chunk,
                index,
            })
        })
    }
}

// translated_position should be the object position in the node coordinates of the chunk.
// node can be easily factored out if it stops being convienent.
impl ChunkBoundingBox {
    pub fn get_chunk_bounding_box(
        node: NodeId,
        chunk: Vertex,
        translated_position: na::Vector4<f64>,
        radius: f64,
        dimension: u8,
    ) -> Option<Self> {
        let euclidean_position = {
            let temp = chunk.node_to_chunk() * translated_position;
            temp.xyz() / temp[3]
        };

        let mut min_xyz = na::Vector3::<u32>::new(0_u32, 0_u32, 0_u32);
        let mut max_xyz = na::Vector3::<u32>::new(0_u32, 0_u32, 0_u32);

        // It's important to note that euclidean_position is measured as chunk lengths, and radius is measured in absolute units.
        // By coincidence, an absolute unit is approximately a chunk's diameter, and only because of that there is no unit conversion here.

        // verify at least one box corner is within the chunk
        if euclidean_position
            .iter()
            .all(|n| n + radius > 0_f64 && n - radius < 1_f64)
        {
            min_xyz.x =
                ((euclidean_position.x - radius).max(0_f64) * dimension as f64).floor() as u32;
            max_xyz.x =
                ((euclidean_position.x + radius).min(1_f64) * dimension as f64).ceil() as u32;

            min_xyz.y =
                ((euclidean_position.y - radius).max(0_f64) * dimension as f64).floor() as u32;
            max_xyz.y =
                ((euclidean_position.y + radius).min(1_f64) * dimension as f64).ceil() as u32;

            min_xyz.z =
                ((euclidean_position.z - radius).max(0_f64) * dimension as f64).floor() as u32;
            max_xyz.z =
                ((euclidean_position.z + radius).min(1_f64) * dimension as f64).ceil() as u32;
            Some(ChunkBoundingBox {
                node,
                chunk,
                min_xyz,
                max_xyz,
                dimension,
            })
        } else {
            None
        }
    }

    pub fn every_voxel(&self) -> impl Iterator<Item = u32> + '_ {
        let lwm = (self.dimension as u32) + 2;
        (self.min_xyz[2]..self.max_xyz[2]).flat_map(move |z| {
            (self.min_xyz[1]..self.max_xyz[1]).flat_map(move |y| {
                (self.min_xyz[0]..self.max_xyz[0])
                    .map(move |x| (x + 1) + lwm * (y + 1) + lwm.pow(2) * (z + 1))
            })
        })
    }
}

impl fmt::Debug for BoundingBox {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Bounding Box")
            .field("Chunk components", &self.bounding_boxes)
            .finish()
    }
}

impl fmt::Debug for ChunkBoundingBox {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Chunk Bounding Box")
            .field("chunk", &self.chunk)
            .field("spanning from", &self.min_xyz.data)
            .field("spanning to", &self.max_xyz.data)
            .finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{graph::Graph, proto::Position, traversal::ensure_nearby};

    const CHUNK_SIZE: u8 = 12_u8; // might want to test with multiple values in the future.
    static CHUNK_SIZE_F: f64 = CHUNK_SIZE as f64;

    fn proper_chunks_generic(x: f64, y: f64, z: f64, expected_chunks: usize) {
        let mut graph = Graph::new();
        ensure_nearby(&mut graph, &Position::origin(), 4.0);
        let central_chunk = Vertex::B; // arbitrary vertex
        let position = central_chunk.chunk_to_node() * na::Vector4::new(x, y, z, 1.0);

        let bb = BoundingBox::create_aabb(
            NodeId::ROOT,
            position,
            1.6 / CHUNK_SIZE_F,
            &graph,
            CHUNK_SIZE,
        );

        assert_eq!(bb.bounding_boxes.len(), expected_chunks);
    }

    // place a small bounding box near the center of the node. There should be exactly 20 chunks in contact.
    #[test]
    fn proper_chunks_20() {
        proper_chunks_generic(
            0.4 / CHUNK_SIZE_F,
            0.4 / CHUNK_SIZE_F,
            0.4 / CHUNK_SIZE_F,
            20,
        );
    }

    // place a small bounding box in the center of a chunk. There should be exactly 1 chunk in contact.
    #[test]
    fn proper_chunks_1() {
        proper_chunks_generic(0.5, 0.5, 0.5, 1)
    }

    // place a small bounding box next to a dodecaherdral vertex. There should be exactly 8 chunks in contact.
    #[test]
    fn proper_chunks_8() {
        proper_chunks_generic(
            1.0 - 0.4 / CHUNK_SIZE_F,
            1.0 - 0.4 / CHUNK_SIZE_F,
            1.0 - 0.4 / CHUNK_SIZE_F,
            8,
        );
    }

    // place a small bounding box next to a dodecaherdral vertex. All chunks in contact should be of the same index
    #[test]
    fn proper_vertex_ids_corner() {
        let mut graph = Graph::new();
        ensure_nearby(&mut graph, &Position::origin(), 4.0);

        let central_chunk = Vertex::C; // arbitrary vertex
        let position = central_chunk.chunk_to_node()
            * na::Vector4::new(
                1.0 - 0.4 / CHUNK_SIZE_F,
                1.0 - 0.4 / CHUNK_SIZE_F,
                1.0 - 0.4 / CHUNK_SIZE_F,
                1.0,
            );

        let bb = BoundingBox::create_aabb(
            NodeId::ROOT,
            position,
            2.0 / CHUNK_SIZE_F,
            &graph,
            CHUNK_SIZE,
        );

        for cbb in bb.bounding_boxes {
            assert!(cbb.chunk == central_chunk);
        }
    }

    // place a small bounding box on the center of a dodecaherdral face. There should be exactly 10 chunks in contact.
    #[test]
    fn proper_chunks_10() {
        proper_chunks_generic(
            1.0 - 0.5 / CHUNK_SIZE_F,
            0.25 / CHUNK_SIZE_F,
            0.25 / CHUNK_SIZE_F,
            10,
        );
    }

    // place a small bounding box right between the center of a dodecaherdral face and the node center. There should be exactly 5 chunks in contact.
    #[test]
    fn proper_chunks_5() {
        proper_chunks_generic(0.4 / CHUNK_SIZE_F, 0.4 / CHUNK_SIZE_F, 0.5, 5);
    }

    // place bounding boxes in a variety of places with a variety of sizes and make sure the amount of voxels contained within are roughly what you would
    // expect in a euclidean bounding box of the same radius.
    #[test]
    fn reasonable_voxel_count() {
        let margin_of_error = 1.5; // 1.5 times more voxels than what would be expected.
        let radi_to_test = CHUNK_SIZE; // higher number means more test precision. Try to keep it a divisor of CHUNK_SIZE_F.

        let mut graph = Graph::new();
        ensure_nearby(&mut graph, &Position::origin(), 4.0);

        let central_chunk = Vertex::B; // arbitrary vertex

        for x in 0..CHUNK_SIZE {
            for r in 1..radi_to_test {
                let radius = (r as f64) / (radi_to_test as f64);
                let x_f64 = x as f64;
                // Getting the correct estimation for the number of voxels can be tricky
                let expected_voxel_count = (radius * 2.0 * CHUNK_SIZE_F).powf(3.0); // value to display
                let minimum_expected_voxel_count =
                    ((((radius * CHUNK_SIZE_F) - 1_f64).powf(3.0) / margin_of_error).floor()
                        * 8_f64) as i32;
                let maximum_expected_voxel_count =
                    ((((radius * CHUNK_SIZE_F) + 1_f64).powf(3.0) * margin_of_error).ceil()
                        * 20_f64) as i32;

                let position = central_chunk.chunk_to_node()
                    * na::Vector4::new(
                        x_f64 / CHUNK_SIZE_F,
                        x_f64 / CHUNK_SIZE_F,
                        x_f64 / CHUNK_SIZE_F,
                        1.0,
                    );

                let bb =
                    BoundingBox::create_aabb(NodeId::ROOT, position, radius, &graph, CHUNK_SIZE);

                let actual_voxels = bb.every_voxel_address().count() as i32;

                println!(
                    "actual_voxels for reasonable_voxel_count: {} vs {}(min), {}(expected), {}(max)",
                    actual_voxels, minimum_expected_voxel_count, expected_voxel_count, maximum_expected_voxel_count
                );
                println!(
                    "\tSpans {} chunks; {} voxels per chunk",
                    bb.bounding_boxes.len(),
                    actual_voxels / (bb.bounding_boxes.len() as i32)
                );
                println!("\tx_f64 is {} radius is {}", x_f64, radius);
                assert!(actual_voxels >= minimum_expected_voxel_count);
                assert!(actual_voxels <= maximum_expected_voxel_count);
            }
        }
    }

    // ensures that chunk_from_location has expected behavior
    #[test]
    fn chunk_from_location_proper_chunk() {
        for central_chunk in Vertex::iter() {
            let chunk_coords = na::Vector3::new(1_u32, 1_u32, 1_u32);

            let position = central_chunk.chunk_to_node()
                * na::Vector4::new(
                    (chunk_coords[0] as f64) / CHUNK_SIZE_F,
                    (chunk_coords[1] as f64) / CHUNK_SIZE_F,
                    (chunk_coords[2] as f64) / CHUNK_SIZE_F,
                    1.0,
                );

            assert_eq!(central_chunk, chunk_from_location(position));
        }
    }

    // places a bounding box at a certain [x,y,z], and verifies that the voxel at [x, y, z] is contained within the bounding box.
    #[test]
    fn internal_coordinates() {
        let mut graph = Graph::new();
        ensure_nearby(&mut graph, &Position::origin(), 4.0);

        let central_chunk = Vertex::B; // arbitrary vertex
        let chunk_coords = na::Vector3::new(2_u32, 6_u32, 9_u32);

        let position = central_chunk.chunk_to_node()
            * na::Vector4::new(
                (chunk_coords[0] as f64) / CHUNK_SIZE_F,
                (chunk_coords[1] as f64) / CHUNK_SIZE_F,
                (chunk_coords[2] as f64) / CHUNK_SIZE_F,
                1.0,
            );

        let bb = BoundingBox::create_aabb(
            NodeId::ROOT,
            position,
            3.0 / CHUNK_SIZE_F,
            &graph,
            CHUNK_SIZE,
        );

        println!("{:?}", bb);

        let lwm = (CHUNK_SIZE + 2_u8) as u32;

        let expected_index =
            chunk_coords[0] + 1 + (chunk_coords[1] + 1) * lwm + (chunk_coords[2] + 1) * lwm.pow(2);

        for address in bb.every_voxel_address() {
            if expected_index == address.index {
                return;
            }
        }
        panic!();
    }

    #[test]
    fn internal_coordinates_tiny() {
        let mut graph = Graph::new();
        ensure_nearby(&mut graph, &Position::origin(), 4.0);

        let central_chunk = Vertex::B; // arbitrary vertex
        let chunk_coords = na::Vector3::new(1_u32, 1_u32, 1_u32);

        let tiny_chunk_size = 1_u8;

        let position = central_chunk.chunk_to_node()
            * na::Vector4::new(
                (chunk_coords[0] as f64) / 1.0_f64,
                (chunk_coords[1] as f64) / 1.0_f64,
                (chunk_coords[2] as f64) / 1.0_f64,
                1.0,
            );

        let bb = BoundingBox::create_aabb(NodeId::ROOT, position, 0.3, &graph, 1);

        println!("{:?}", bb);

        let lwm = (tiny_chunk_size + 2_u8) as u32;

        //let expected_index = chunk_coords[0] + 1 + (chunk_coords[1] + 1) * lwm + (chunk_coords[2] + 1)*lwm.pow(2);
        let expected_index = chunk_coords[0] + chunk_coords[1] * lwm + chunk_coords[2] * lwm.pow(2);

        for address in bb.every_voxel_address() {
            if expected_index == address.index {
                return;
            }
        }
        panic!();
    }
}
