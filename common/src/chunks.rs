use std::ops::{Index, IndexMut};

use crate::dodeca::Vertex;

/// A table of chunks contained by a single node
///
/// Each chunk is 1/8 of a cube whose vertices are at the centers of nodes.
#[derive(Debug, Copy, Clone, Default)]
pub struct Chunks<T> {
    values: [T; 20],
}

impl<T> Index<Vertex> for Chunks<T> {
    type Output = T;
    fn index(&self, v: Vertex) -> &T {
        &self.values[v as usize]
    }
}

impl<T> IndexMut<Vertex> for Chunks<T> {
    fn index_mut(&mut self, v: Vertex) -> &mut T {
        &mut self.values[v as usize]
    }
}
