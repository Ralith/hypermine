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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_index() {
        let chunks: Chunks<Option<isize>> = Chunks {
            values: [
                Some(0),
                Some(1),
                Some(2),
                Some(3),
                Some(4),
                Some(5),
                Some(6),
                Some(7),
                Some(8),
                Some(9),
                Some(10),
                Some(11),
                Some(12),
                Some(13),
                Some(14),
                Some(15),
                Some(16),
                Some(17),
                Some(18),
                Some(19),
            ],
        };

        // Alphabetical order of vertices should index into array in order
        assert!(chunks.index(Vertex::A).unwrap() == 0);
        assert!(chunks.index(Vertex::B).unwrap() == 1);
        assert!(chunks.index(Vertex::C).unwrap() == 2);
        assert!(chunks.index(Vertex::D).unwrap() == 3);
        assert!(chunks.index(Vertex::E).unwrap() == 4);
        assert!(chunks.index(Vertex::F).unwrap() == 5);
        assert!(chunks.index(Vertex::G).unwrap() == 6);
        assert!(chunks.index(Vertex::H).unwrap() == 7);
        assert!(chunks.index(Vertex::I).unwrap() == 8);
        assert!(chunks.index(Vertex::J).unwrap() == 9);
        assert!(chunks.index(Vertex::K).unwrap() == 10);
        assert!(chunks.index(Vertex::L).unwrap() == 11);
        assert!(chunks.index(Vertex::M).unwrap() == 12);
        assert!(chunks.index(Vertex::N).unwrap() == 13);
        assert!(chunks.index(Vertex::O).unwrap() == 14);
        assert!(chunks.index(Vertex::P).unwrap() == 15);
        assert!(chunks.index(Vertex::Q).unwrap() == 16);
        assert!(chunks.index(Vertex::R).unwrap() == 17);
        assert!(chunks.index(Vertex::S).unwrap() == 18);
        assert!(chunks.index(Vertex::T).unwrap() == 19);
    }

    #[test]
    fn test_index_mut() {
        let mut chunks: Chunks<Option<isize>> = Chunks {
            values: [
                Some(0),
                Some(1),
                Some(2),
                Some(3),
                Some(4),
                Some(5),
                Some(6),
                Some(7),
                Some(8),
                Some(9),
                Some(10),
                Some(11),
                Some(12),
                Some(13),
                Some(14),
                Some(15),
                Some(16),
                Some(17),
                Some(18),
                Some(19),
            ],
        };

        let x = chunks.index_mut(Vertex::A);
        *x = None;

        match chunks.index(Vertex::A) {
            Some(_) => assert!(false),
            None => assert!(true),
        };
    }
}
