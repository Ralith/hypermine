use std::collections::HashSet;

use enum_map::{enum_map, EnumMap};
use rand::Rng;

use crate::{
    math::HyperboloidVector,
    penta::{Side, Vertex},
};

pub struct Tessellation {
    nodes: Vec<Node>,
    root: NodeHandle,
    chunk_size: usize,
}

impl Tessellation {
    pub fn new(chunk_size: usize) -> Self {
        let mut root_node = Node::new(chunk_size);
        root_node.down = *Side::A.normal();
        root_node.is_near_ground = true;
        Tessellation {
            nodes: vec![root_node],
            root: NodeHandle { index: 0 },
            chunk_size,
        }
    }

    pub fn chunk_data(&self, node: NodeHandle, vertex: Vertex) -> ChunkData {
        ChunkData {
            chunk_size: self.chunk_size,
            data: &self.nodes[node.index].chunks[vertex].data,
        }
    }

    pub fn mut_chunk_data(&mut self, node: NodeHandle, vertex: Vertex) -> MutChunkData {
        MutChunkData {
            chunk_size: self.chunk_size,
            data: &mut self.nodes[node.index].chunks[vertex].data,
        }
    }

    pub fn root(&self) -> NodeHandle {
        self.root
    }

    pub fn parity(&self, node: NodeHandle) -> u32 {
        self.get_node(node).is_odd as u32
    }

    pub fn down(&self, node: NodeHandle) -> &na::Vector3<f32> {
        &self.get_node(node).down
    }

    pub fn get_chunk_data(&self, node: NodeHandle, vertex: Vertex) -> ChunkData {
        ChunkData {
            chunk_size: self.chunk_size,
            data: &self.get_node(node).chunks[vertex].data,
        }
    }

    pub fn get_neighbor(&self, node: NodeHandle, side: Side) -> Option<NodeHandle> {
        self.get_node(node).neighbors[side]
    }

    pub fn get_voxel_at_pos(
        &mut self,
        node: NodeHandle,
        pos: na::Vector3<f32>,
    ) -> Option<(MutChunkData, [usize; 2])> {
        let mut current_node = node;
        let mut current_pos = pos;
        for _ in 0..5 {
            for vertex in Vertex::iter() {
                let mut voxel_pos = vertex.penta_to_voxel() * current_pos;
                voxel_pos /= voxel_pos.z;
                let integer_voxel_pos =
                    voxel_pos.xy().map(|x| (x * self.chunk_size as f32).floor());
                if integer_voxel_pos
                    .iter()
                    .all(|&x| x >= 0.0 && (x as usize) < self.chunk_size)
                {
                    return Some((
                        self.mut_chunk_data(current_node, vertex),
                        [integer_voxel_pos.x as usize, integer_voxel_pos.y as usize],
                    ));
                }
            }

            for side in Side::iter() {
                if current_pos.mip(side.normal()) > 0.0 {
                    if let Some(new_node) = self.get_neighbor(current_node, side) {
                        current_node = new_node;
                        current_pos = side.reflection() * current_pos;
                    } else {
                        return None;
                    }
                }
            }
        }
        None
    }

    pub fn get_nearby(
        &self,
        node: NodeHandle,
        transform: na::Matrix3<f32>,
        distance: u32,
    ) -> Vec<NodeHandleWithContext> {
        let mut visited = HashSet::new();
        let mut result = vec![NodeHandleWithContext { node, transform }];
        let mut frontier_start: usize = 0;

        for _ in 0..distance {
            let frontier_end = result.len();
            for i in frontier_start..frontier_end {
                let handle_with_context = result[i];
                for side in Side::iter() {
                    if let Some(neighbor) = self.get_neighbor(handle_with_context.node, side) {
                        if visited.insert(neighbor) {
                            result.push(NodeHandleWithContext {
                                node: neighbor,
                                transform: handle_with_context.transform * side.reflection(),
                            })
                        }
                    }
                }
            }
            frontier_start = frontier_end;
        }

        result
    }

    // TODO: Can we avoid code duplication with get_nearby?
    pub fn ensure_nearby(
        &mut self,
        node: NodeHandle,
        transform: na::Matrix3<f32>,
        distance: u32,
    ) -> Vec<NodeHandleWithContext> {
        let mut visited = HashSet::new();
        visited.insert(node);

        let mut result = vec![NodeHandleWithContext { node, transform }];

        let mut frontier_start: usize = 0;

        for _ in 0..distance {
            let frontier_end = result.len();
            for i in frontier_start..frontier_end {
                let handle_with_context = result[i];
                for side in Side::iter() {
                    let neighbor = self.ensure_neighbor(handle_with_context.node, side);
                    if visited.insert(neighbor) {
                        result.push(NodeHandleWithContext {
                            node: neighbor,
                            transform: handle_with_context.transform * side.reflection(),
                        });
                    }
                }
            }
            frontier_start = frontier_end;
        }

        result
    }

    fn ensure_neighbor(&mut self, node: NodeHandle, side: Side) -> NodeHandle {
        let actual_node = self.get_node(node);

        if let Some(neighbor) = actual_node.neighbors[side] {
            return neighbor;
        }

        if actual_node.is_child[side] {
            return self.add_child(node, side);
        }

        let parent_side = actual_node.parent.unwrap();
        let parent_neighbor =
            self.ensure_neighbor(actual_node.neighbors[parent_side].unwrap(), side);

        let neighbor = self.ensure_neighbor(parent_neighbor, parent_side);
        self.link_nodes(node, neighbor, side);
        neighbor
    }

    fn add_child(&mut self, node: NodeHandle, side: Side) -> NodeHandle {
        let child = self.add_node();
        self.get_node_mut(child).is_child[side] = false;
        self.get_node_mut(child).parent = Some(side);
        self.get_node_mut(child).is_odd = !self.get_node(node).is_odd;
        self.link_nodes(node, child, side);

        if self.get_node(node).is_near_ground && side.is_neighbor(Side::A) {
            // Rather than applying an ostensibly no-op reflection, creating an unstable equilibrium, just do nothing.
            self.get_node_mut(child).down = self.get_node(node).down;
            self.get_node_mut(child).is_near_ground = true;
        } else {
            self.get_node_mut(child).down = side.reflection() * self.get_node(node).down;
            self.get_node_mut(child).is_near_ground =
                self.get_node(node).is_near_ground && side == Side::A;
        }

        // Higher-priority sides and already-pruned sides need to be pruned from the child
        for candidate_side in Side::iter().filter(|&s| side.is_neighbor(s)) {
            if candidate_side < side || !self.get_node(node).is_child[candidate_side] {
                self.get_node_mut(child).is_child[candidate_side] = false;
            }
        }

        child
    }

    fn link_nodes(&mut self, node0: NodeHandle, node1: NodeHandle, side: Side) {
        self.nodes[node0.index].neighbors[side] = Some(node1);
        self.nodes[node1.index].neighbors[side] = Some(node0);
    }

    fn add_node(&mut self) -> NodeHandle {
        let node = Node::new(self.chunk_size);
        self.nodes.push(node);
        NodeHandle {
            index: self.nodes.len() - 1,
        }
    }

    fn get_node_mut(&mut self, node: NodeHandle) -> &mut Node {
        &mut self.nodes[node.index]
    }

    fn get_node(&self, node: NodeHandle) -> &Node {
        &self.nodes[node.index]
    }
}

struct Node {
    neighbors: EnumMap<Side, Option<NodeHandle>>,
    chunks: EnumMap<Vertex, Chunk>, // Per vertex
    is_child: EnumMap<Side, bool>,
    parent: Option<Side>,
    is_odd: bool,
    down: na::Vector3<f32>,
    is_near_ground: bool,
}

impl Node {
    fn new(chunk_size: usize) -> Node {
        Node {
            neighbors: EnumMap::default(),
            chunks: enum_map! { _ => Chunk::new(chunk_size) },
            is_child: enum_map! { _ => true },
            parent: None,
            is_odd: false,
            down: na::Vector3::zeros(),
            is_near_ground: false,
        }
    }
}

struct Chunk {
    data: Vec<u8>,
}

impl Chunk {
    fn new(chunk_size: usize) -> Chunk {
        Chunk {
            data: (0..chunk_size * chunk_size)
                .map(|_| (rand::thread_rng().gen::<f32>() < 0.02) as u8)
                .collect(),
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct NodeHandle {
    index: usize,
}

#[derive(Copy, Clone)]
pub struct NodeHandleWithContext {
    pub node: NodeHandle,
    pub transform: na::Matrix3<f32>,
}

#[derive(Copy, Clone)]
pub struct ChunkData<'a> {
    chunk_size: usize,
    data: &'a Vec<u8>,
}

pub struct MutChunkData<'a> {
    chunk_size: usize,
    data: &'a mut Vec<u8>,
}

impl<'a> ChunkData<'a> {
    pub fn chunk_size(&self) -> usize {
        self.chunk_size
    }

    pub fn get(&self, coords: [usize; 2]) -> u8 {
        assert!(coords[0] < self.chunk_size);
        assert!(coords[1] < self.chunk_size);
        self.data[coords[0] + coords[1] * self.chunk_size]
    }
}

impl<'a> MutChunkData<'a> {
    pub fn chunk_size(&self) -> usize {
        self.chunk_size
    }

    pub fn get(&self, coords: [usize; 2]) -> u8 {
        assert!(coords[0] < self.chunk_size);
        assert!(coords[1] < self.chunk_size);
        self.data[coords[0] + coords[1] * self.chunk_size]
    }

    pub fn set(&mut self, coords: [usize; 2], val: u8) {
        assert!(coords[0] < self.chunk_size);
        assert!(coords[1] < self.chunk_size);
        self.data[coords[0] + coords[1] * self.chunk_size] = val;
    }
}

#[cfg(test)]
mod test {
    use crate::penta::Side;

    use super::Tessellation;

    #[test]
    fn test_ensure_neighbor() {
        let mut tessellation = Tessellation::new(12);
        let root = tessellation.root();
        let a = tessellation.ensure_neighbor(root, Side::A);
        let ab = tessellation.ensure_neighbor(a, Side::B);
        let _b = tessellation.ensure_neighbor(ab, Side::A);
        print_graph(&tessellation);
    }

    #[test]
    fn test_ensure_nearby() {
        let mut tessellation = Tessellation::new(12);
        let root = tessellation.root();
        println!(
            "{}",
            tessellation
                .ensure_nearby(root, na::Matrix3::identity(), 3)
                .len()
        );
        print_graph(&tessellation);
    }

    fn print_graph(tessellation: &Tessellation) {
        for (i, node) in tessellation.nodes.iter().enumerate() {
            println!("{}: {:?}", i, node.neighbors.map(|_, o| o.map(|h| h.index)));
        }
    }
}
