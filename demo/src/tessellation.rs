use crate::{
    math,
    node_string::{NodeString, Side, Vertex},
};

// Order 4 pentagonal tiling
// Note: Despite being constants, it is undefined behavior to change these values, as certain hardcoded unsafe code
// depends on them being set to their current values, NUM_SIDES = 5 and ORDER = 4
const NUM_SIDES: usize = 5;
const ORDER: usize = 4;

pub struct Tessellation {
    reflection_vectors: [na::Vector3<f64>; NUM_SIDES],
    reflections: [na::Matrix3<f64>; NUM_SIDES],
    vertices: [na::Vector3<f64>; NUM_SIDES],
    voxel_to_hyperboloid: [na::Matrix3<f64>; NUM_SIDES],
    hyperboloid_to_voxel: [na::Matrix3<f64>; NUM_SIDES],
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

        let mut reflection_vectors = [na::Vector3::default(); NUM_SIDES];
        let mut vertices = [na::Vector3::default(); NUM_SIDES];
        let mut voxel_to_hyperboloid = [na::Matrix3::default(); NUM_SIDES];

        for (i, reflection) in reflection_vectors.iter_mut().enumerate() {
            let theta = side_angle * i as f64;
            *reflection = na::Vector3::new(
                reflection_r * theta.cos(),
                reflection_r * theta.sin(),
                reflection_z,
            );
        }

        for (i, vertex) in vertices.iter_mut().enumerate() {
            *vertex = math::normal(
                &reflection_vectors[(i + 1) % NUM_SIDES],
                &reflection_vectors[(i) % NUM_SIDES],
            );
            *vertex /= (-math::sqr(vertex)).sqrt();
        }

        for (i, mat) in voxel_to_hyperboloid.iter_mut().enumerate() {
            let reflector0 = &reflection_vectors[(i) % NUM_SIDES];
            let reflector1 = &reflection_vectors[(i + 1) % NUM_SIDES];
            let origin = na::Vector3::new(0.0, 0.0, 1.0);
            *mat = na::Matrix3::from_columns(&[
                -reflector0 * reflector0.z,
                -reflector1 * reflector1.z,
                origin + reflector0 * reflector0.z + reflector1 * reflector1.z,
            ]);
        }

        Tessellation {
            reflection_vectors,
            reflections: reflection_vectors.map(|v| math::reflection(&v)),
            vertices,
            voxel_to_hyperboloid,
            hyperboloid_to_voxel: voxel_to_hyperboloid.map(|m| m.try_inverse().unwrap()),
            nodes: Vec::new(),
            chunk_size,
        }
    }

    pub fn reflection(&self, side: Side) -> &na::Matrix3<f64> {
        unsafe { self.reflections.get_unchecked(side as usize) }
    }

    // TODO: This function is inefficient and should be replaced with a graph datastructure
    pub fn voxel_to_hyperboloid(&self, ns: &NodeString, vert: Vertex) -> na::Matrix3<f64> {
        let mut transform = na::Matrix3::identity();
        for &side in ns.iter() {
            transform *= self.reflection(side);
        }
        transform *= unsafe { self.voxel_to_hyperboloid.get_unchecked(vert as usize) };
        transform
    }

    pub fn chunk_data(&self, node: NodeHandle, vertex: Vertex) -> ChunkData {
        ChunkData {
            chunk_size: self.chunk_size,
            data: &self.nodes[node.index].chunks[vertex as usize].data,
        }
    }

    fn link_nodes(&mut self, node0: NodeHandle, node1: NodeHandle, side: Side) {
        self.nodes[node0.index].neighbors[side as usize] = Some(node1);
        self.nodes[node1.index].neighbors[side as usize] = Some(node0);
    }
}

struct Node {
    neighbors: [Option<NodeHandle>; NUM_SIDES],
    chunks: [Chunk; 5], // Per vertex
}

impl Node {
    fn new(chunk_size: usize) -> Node {
        Node {
            neighbors: [None; NUM_SIDES],
            chunks: [0; 5].map(|_| Chunk::new(chunk_size)),
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

    pub fn value(&self, x: usize, y: usize) -> u8 {
        self.data[x * self.chunk_size + y]
    }
}
