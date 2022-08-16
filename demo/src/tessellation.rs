use enum_map::{enum_map, EnumMap};

use crate::{
    math,
    node_string::{NodeString, Side, Vertex},
};

// Order 4 pentagonal tiling
// Note: Despite being constants, they are not really configurable, as the rest of the code
// depends on them being set to their current values, NUM_SIDES = 5 and ORDER = 4
const NUM_SIDES: usize = 5;
const ORDER: usize = 4;

pub struct Tessellation {
    vertex_sides: EnumMap<Vertex, (Side, Side)>,
    reflection_vectors: EnumMap<Side, na::Vector3<f64>>,
    reflections: EnumMap<Side, na::Matrix3<f64>>,
    vertices: EnumMap<Vertex, na::Vector3<f64>>,
    voxel_to_hyperboloid: EnumMap<Vertex, na::Matrix3<f64>>,
    hyperboloid_to_voxel: EnumMap<Vertex, na::Matrix3<f64>>,
    nodes: Vec<Node>,
    chunk_size: usize,
}

impl Tessellation {
    pub fn new(chunk_size: usize) -> Self {
        let side_angle = math::TAU / NUM_SIDES as f64;
        let order_angle = math::TAU / ORDER as f64;

        let cos_side_angle = side_angle.cos();
        let cos_order_angle = order_angle.cos();

        let reflection_r = ((1.0 + cos_order_angle) / (1.0 - cos_side_angle)).sqrt();
        let reflection_z = ((cos_side_angle + cos_order_angle) / (1.0 - cos_side_angle)).sqrt();

        let vertex_sides: EnumMap<Vertex, (Side, Side)> = enum_map! {
            Vertex::AB => (Side::A, Side::B),
            Vertex::BC => (Side::B, Side::C),
            Vertex::CD => (Side::C, Side::D),
            Vertex::DE => (Side::D, Side::E),
            Vertex::EA => (Side::E, Side::A),
        };

        let mut reflection_vectors: EnumMap<Side, na::Vector3<f64>> = EnumMap::default();
        let mut vertices: EnumMap<Vertex, na::Vector3<f64>> = EnumMap::default();
        let mut voxel_to_hyperboloid: EnumMap<Vertex, na::Matrix3<f64>> = EnumMap::default();

        for (side, reflection) in reflection_vectors.iter_mut() {
            let theta = side_angle * (side as usize) as f64;
            *reflection = na::Vector3::new(
                reflection_r * theta.cos(),
                reflection_r * theta.sin(),
                reflection_z,
            );
        }

        for (vertex, vertex_pos) in vertices.iter_mut() {
            *vertex_pos = math::normal(
                &reflection_vectors[vertex_sides[vertex].0],
                &reflection_vectors[vertex_sides[vertex].1],
            );
            *vertex_pos /= (-math::sqr(vertex_pos)).sqrt();
        }

        for (vertex, mat) in voxel_to_hyperboloid.iter_mut() {
            let reflector0 = &reflection_vectors[vertex_sides[vertex].0];
            let reflector1 = &reflection_vectors[vertex_sides[vertex].1];
            let origin = na::Vector3::new(0.0, 0.0, 1.0);
            *mat = na::Matrix3::from_columns(&[
                -reflector0 * reflector0.z,
                -reflector1 * reflector1.z,
                origin + reflector0 * reflector0.z + reflector1 * reflector1.z,
            ]);
        }

        Tessellation {
            vertex_sides,
            reflection_vectors,
            reflections: reflection_vectors.map(|_, v| math::reflection(&v)),
            vertices,
            voxel_to_hyperboloid,
            hyperboloid_to_voxel: voxel_to_hyperboloid.map(|_, m| m.try_inverse().unwrap()),
            nodes: Vec::new(),
            chunk_size,
        }
    }

    pub fn reflection(&self, side: Side) -> &na::Matrix3<f64> {
        &self.reflections[side]
    }

    // TODO: This function is inefficient and should be replaced with a graph datastructure
    pub fn voxel_to_hyperboloid(&self, ns: &NodeString, vert: Vertex) -> na::Matrix3<f64> {
        let mut transform = na::Matrix3::identity();
        for &side in ns.iter() {
            transform *= self.reflection(side);
        }
        transform *= self.voxel_to_hyperboloid[vert];
        transform
    }

    pub fn chunk_data(&self, node: NodeHandle, vertex: Vertex) -> ChunkData {
        ChunkData {
            chunk_size: self.chunk_size,
            data: &self.nodes[node.index].chunks[vertex].data,
        }
    }

    fn link_nodes(&mut self, node0: NodeHandle, node1: NodeHandle, side: Side) {
        self.nodes[node0.index].neighbors[side] = Some(node1);
        self.nodes[node1.index].neighbors[side] = Some(node0);
    }
}

struct Node {
    neighbors: EnumMap<Side, Option<NodeHandle>>,
    chunks: EnumMap<Vertex, Chunk>, // Per vertex
}

impl Node {
    fn new(chunk_size: usize) -> Node {
        Node {
            neighbors: EnumMap::default(),
            chunks: enum_map! { _ => Chunk::new(chunk_size) },
        }
    }
}

struct Chunk {
    data: Vec<u8>,
}

impl Chunk {
    fn new(chunk_size: usize) -> Chunk {
        Chunk {
            data: (0..chunk_size * chunk_size).map(|_| 1).collect(),
        }
    }
}

#[derive(Copy, Clone)]
pub struct NodeHandle {
    index: usize,
}

#[derive(Copy, Clone)]
pub struct ChunkData<'a> {
    chunk_size: usize,
    data: &'a Vec<u8>,
}

impl<'a> ChunkData<'a> {
    pub fn chunk_size(&self) -> usize {
        self.chunk_size
    }

    pub fn get(&self, x: usize, y: usize) -> u8 {
        self.data[x * self.chunk_size + y]
    }
}
