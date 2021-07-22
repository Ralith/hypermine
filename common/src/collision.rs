use crate::node::{DualGraph};
use crate::{
    dodeca::{Vertex},
    graph::NodeId,
};

#[allow(dead_code)]
/*
The set of voxels that a collision body covers within a chunk
*/
#[derive(PartialEq, Clone, Debug, Copy)]
pub struct ChunkBoundingBox {
    node: NodeId,
    chunk: Vertex,
    min_xyz: na::Vector3<i32>,
    max_xyz: na::Vector3<i32>,
    dimension: u8,
}

pub struct VoxelAddress {
    node: NodeId,
    chunk: Vertex,
    index: i32,
}

/*
The set of voxels that a collision body covers.
*/
pub struct BoundingBox {
    bounding_boxes: Vec<ChunkBoundingBox>,
}

// Does not figure out which chunk it is in automatically
// The pattern I use to unwrap the results of neighbor is kind of awkward.
impl BoundingBox {
    pub fn create_aabb(
        start_node: NodeId,
        start_chunk: Vertex,
        position: na::Vector4<f64>,
        radius: f64,
        graph: &DualGraph,
        dimension: u8,
    ) -> Self {
        assert!(
            radius <= 1.0,
            "Error: the radius of a bounding box may not exceed 1 absolute unit."
        );
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
        for side in sides.iter() {
            if let Some(node) = graph.neighbor(start_node, *side) {
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

        if let Some(opposite_node) = graph.opposing_node(start_node, sides) {
            let opposite_position =
                sides[0].reflection() * sides[1].reflection() * sides[2].reflection() * position;

            // get BBs for the chunks sharing an edge
            for side in sides.iter() {
                let node = graph.neighbor(opposite_node, *side);
                let translated_position = side.reflection() * opposite_position;

                Self::add_sub_bb(
                    &mut bounding_boxes,
                    ChunkBoundingBox::get_chunk_bounding_box(
                        node.unwrap(),
                        Vertex::from_sides(sides[0], sides[1], sides[2]).unwrap(), // pretty sure this is pointless
                        translated_position,
                        radius,
                        dimension,
                    ),
                );
            }

            // get BB for chunk sharing only a vertex.
            Self::add_sub_bb(
                &mut bounding_boxes,
                ChunkBoundingBox::get_chunk_bounding_box(
                    opposite_node,
                    Vertex::from_sides(sides[0], sides[1], sides[2]).unwrap(),
                    opposite_position,
                    radius,
                    dimension,
                ),
            );
        }

        BoundingBox { bounding_boxes }
    }

    // adds a sub-bounding box to a list if it exists
    pub fn add_sub_bb(list: &mut Vec<ChunkBoundingBox>, result: Option<ChunkBoundingBox>) {
        if let Some(x) = result {
            list.push(x);
        }
    }

    pub fn every_voxel_address<'a>(&'a self) -> impl Iterator<Item = VoxelAddress> + 'a {
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
        let euclidean_position =
            (chunk.chunk_to_node().try_inverse().unwrap() * translated_position).xyz();
        let mut min_xyz = na::Vector3::<i32>::new(0_i32, 0_i32, 0_i32);
        let mut max_xyz = na::Vector3::<i32>::new(0_i32, 0_i32, 0_i32);

        // verify at least one box corner is within the chunk
        if euclidean_position
            .iter()
            .all(|n| n + radius > 0_f64 && n - radius < 1_f64)
        {
            min_xyz.x = (euclidean_position.x.min(0_f64) * dimension as f64).floor() as i32;
            max_xyz.x = (euclidean_position.x.max(1_f64) * dimension as f64).ceil() as i32;

            min_xyz.y = (euclidean_position.y.min(0_f64) * dimension as f64).floor() as i32;
            max_xyz.y = (euclidean_position.y.max(1_f64) * dimension as f64).ceil() as i32;

            min_xyz.z = (euclidean_position.z.min(0_f64) * dimension as f64).floor() as i32;
            max_xyz.z = (euclidean_position.z.max(1_f64) * dimension as f64).ceil() as i32;

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

    // returns the index (single number) of every voxel contained inside
    pub fn every_voxel<'b>(&'b self) -> impl Iterator<Item = i32> + 'b {
        (self.min_xyz[2]..self.max_xyz[2]).flat_map(move |z| {
            (self.min_xyz[1]..self.max_xyz[1]).flat_map(move |y| {
                (self.min_xyz[0]..self.max_xyz[0]).map(move |x| {
                    x + (self.dimension as i32) * y + (self.dimension as i32).pow(2) * z
                })
            })
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::node::{DualGraph, Node};
    use crate::Chunks;
    use crate::{graph::Graph, proto::Position, traversal::ensure_nearby};
    use approx::*;

    const CHUNK_SIZE: u8 = 12; // might want to test with multiple values in the future.
    static CHUNK_SIZE_F: f64 = CHUNK_SIZE as f64;

    // place a small bounding box near the center of the node. There should be exactly 20 chunks in contact.
    #[test]
    fn proper_chunks_20() {
        let mut graph = Graph::new();
        ensure_nearby(&mut graph, &Position::origin(), 4.0);

        let central_chunk = Vertex::A; // arbitrary vertex
        let position = central_chunk.chunk_to_node()
            * na::Vector4::new(
                0.4 / CHUNK_SIZE_F,
                0.4 / CHUNK_SIZE_F,
                0.4 / CHUNK_SIZE_F,
                1.0,
            );

        let bb = BoundingBox::create_aabb(
            NodeId::ROOT,
            central_chunk,
            position,
            2.0 / CHUNK_SIZE_F,
            &graph,
            CHUNK_SIZE,
        );

        assert_eq!(bb.bounding_boxes.len(), 20);
    }

    // place a small bounding box in the center of a chunk. There should be exactly 1 chunk in contact.
    #[test]
    fn proper_chunks_1() {
        let mut graph = Graph::new();
        ensure_nearby(&mut graph, &Position::origin(), 4.0);

        let central_chunk = Vertex::A; // arbitrary vertex
        let position = central_chunk.chunk_to_node() * na::Vector4::new(0.5, 0.5, 0.5, 1.0);

        let bb = BoundingBox::create_aabb(
            NodeId::ROOT,
            central_chunk,
            position,
            1.0 / CHUNK_SIZE_F,
            &graph,
            CHUNK_SIZE,
        );

        assert_eq!(bb.bounding_boxes.len(), 1);
    }

    // place a small bounding box next to a dodecaherdral vertex. There should be exactly 8 chunks in contact.
    #[test]
    fn proper_chunks_8() {
        let mut graph = Graph::new();
        ensure_nearby(&mut graph, &Position::origin(), 4.0);

        let central_chunk = Vertex::K; // arbitrary vertex
        let position = central_chunk.chunk_to_node()
            * na::Vector4::new(
                1.0 - 0.4 / CHUNK_SIZE_F,
                1.0 - 0.4 / CHUNK_SIZE_F,
                1.0 - 0.4 / CHUNK_SIZE_F,
                1.0,
            );

        let bb = BoundingBox::create_aabb(
            NodeId::ROOT,
            central_chunk,
            position,
            2.0 / CHUNK_SIZE_F,
            &graph,
            CHUNK_SIZE,
        );

        assert_eq!(bb.bounding_boxes.len(), 8);
    }

    // place a small bounding box on the center of a dodecaherdral face. There should be exactly 10 chunks in contact.
    #[test]
    fn proper_chunks_10() {
        let mut graph = Graph::new();
        ensure_nearby(&mut graph, &Position::origin(), 4.0);

        let central_chunk = Vertex::D; // arbitrary vertex
        let position = central_chunk.chunk_to_node()
            * na::Vector4::new(
                1.0 - 0.4 / CHUNK_SIZE_F,
                1.0 - 0.4 / CHUNK_SIZE_F,
                0.4 / CHUNK_SIZE_F,
                1.0,
            );

        let bb = BoundingBox::create_aabb(
            NodeId::ROOT,
            central_chunk,
            position,
            2.0 / CHUNK_SIZE_F,
            &graph,
            CHUNK_SIZE,
        );

        assert_eq!(bb.bounding_boxes.len(), 10);
    }

    // place a small bounding box right between the center of a dodecaherdral face and the node center. There should be exactly 5 chunks in contact.
    #[test]
    fn proper_chunks_5() {
        let mut graph = Graph::new();
        ensure_nearby(&mut graph, &Position::origin(), 4.0);

        let central_chunk = Vertex::E; // arbitrary vertex
        let position = central_chunk.chunk_to_node()
            * na::Vector4::new(0.4 / CHUNK_SIZE_F, 0.4 / CHUNK_SIZE_F, 0.5, 1.0);

        let bb = BoundingBox::create_aabb(
            NodeId::ROOT,
            central_chunk,
            position,
            2.0 / CHUNK_SIZE_F,
            &graph,
            CHUNK_SIZE,
        );

        assert_eq!(bb.bounding_boxes.len(), 5);
    }

    // place bounding boxes in a variety of places with a variety of sizes and make sure the amount of voxels contained within are roughly what you would
    // expect in a euclidean bounding box of the same radius.
    #[test]
    fn reasonable_voxel_count() {
        let margin_of_error = 3.0; // three times more voxels than what would be expected.
        let radi_to_test = 6; // higher number means more test precision.

        let mut graph = Graph::new();
        ensure_nearby(&mut graph, &Position::origin(), 4.0);

        let central_chunk = Vertex::B; // arbitrary vertex

        for x in 0..CHUNK_SIZE {
            for r in 1..radi_to_test {
                let radius = (r / radi_to_test) as f64;
                let x_f64 = x as f64;
                let expected_voxel_count = (radius * 2.0).powf(3.0);
                let position = central_chunk.chunk_to_node()
                    * na::Vector4::new(
                        x_f64 / CHUNK_SIZE_F,
                        x_f64 / CHUNK_SIZE_F,
                        x_f64 / CHUNK_SIZE_F,
                        1.0,
                    );

                let bb = BoundingBox::create_aabb(
                    NodeId::ROOT,
                    central_chunk,
                    position,
                    radius,
                    &graph,
                    CHUNK_SIZE,
                );

                let mut actual_voxels = 0;
                for address in bb.every_voxel_address() {
                    actual_voxels += 1;
                }

                assert!(actual_voxels <= (expected_voxel_count * margin_of_error).ceil() as i32);
                assert!(actual_voxels >= (expected_voxel_count / margin_of_error).floor() as i32);
            }
        }
    }

    // places a bounding box at a certain [x,y,z], and verifies that the voxel at [x, y, z] is contained within the bounding box.
    #[test]
    fn internal_coordinates() {
        let mut graph = Graph::new();
        ensure_nearby(&mut graph, &Position::origin(), 4.0);

        let central_chunk = Vertex::B; // arbitrary vertex
        let chunk_coords = na::Vector3::new(2_i32, 6_i32, 9_i32);

        let position = central_chunk.chunk_to_node()
            * na::Vector4::new(
                (chunk_coords[0] as f64) / CHUNK_SIZE_F,
                (chunk_coords[1] as f64) / CHUNK_SIZE_F,
                (chunk_coords[2] as f64) / CHUNK_SIZE_F,
                1.0,
            );

        let bb = BoundingBox::create_aabb(
            NodeId::ROOT,
            central_chunk,
            position,
            3.0 / CHUNK_SIZE_F,
            &graph,
            CHUNK_SIZE,
        );

        let expected_index = chunk_coords[0] * (CHUNK_SIZE as i32).pow(2)
            + chunk_coords[1] * (CHUNK_SIZE as i32)
            + chunk_coords[2];

        for address in bb.every_voxel_address() {
            if expected_index == address.index {
                return;
            }
        }
        panic!();
    }
}
