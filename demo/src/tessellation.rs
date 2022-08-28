use std::collections::{hash_map, HashMap};

use enum_map::{enum_map, EnumMap};

use crate::penta::{Side, Vertex};

pub struct Tessellation {
    nodes: Vec<Node>,
    root: NodeHandle,
    chunk_size: usize,
}

impl Tessellation {
    pub fn new(chunk_size: usize) -> Self {
        Tessellation {
            nodes: vec![Node::new(chunk_size)],
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

    pub fn root(&self) -> NodeHandle {
        self.root
    }

    pub fn parity(&self, node: NodeHandle) -> u32 {
        if self.get_node(node).is_odd {
            1
        } else {
            0
        }
    }

    pub fn get_chunk_data(&self, node: NodeHandle, vertex: Vertex) -> ChunkData {
        ChunkData {
            chunk_size: self.chunk_size,
            data: &self.get_node(node).chunks[vertex].data,
        }
    }

    pub fn ensure_nearby(
        &mut self,
        node: NodeHandle,
        transform: na::Matrix3<f64>,
        distance: u32,
    ) -> Vec<NodeHandleWithContext> {
        let mut visited = HashMap::new();
        visited.insert(node, ());

        let mut result = vec![NodeHandleWithContext { node, transform }];

        let mut frontier_start: usize = 0;

        for _ in 0..distance {
            let frontier_end = result.len();
            for i in frontier_start..frontier_end {
                let handle_with_context = result[i];
                for side in Side::iter() {
                    let neighbor = self.ensure_neighbor(handle_with_context.node, side);
                    if let hash_map::Entry::Vacant(e) = visited.entry(neighbor) {
                        e.insert(());
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
}

impl Node {
    fn new(chunk_size: usize) -> Node {
        Node {
            neighbors: EnumMap::default(),
            chunks: enum_map! { _ => Chunk::new(chunk_size) },
            is_child: enum_map! { _ => true },
            parent: None,
            is_odd: false,
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
                .map(|i| match i % 6 {
                    0..=1 => 1,
                    _ => 0,
                })
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
    pub transform: na::Matrix3<f64>,
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
