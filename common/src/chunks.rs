use std::ops::{Index, IndexMut};

use crate::dodeca::Vertex;

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
