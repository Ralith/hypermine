use enum_map::{enum_map, Enum, EnumMap};
use lazy_static::lazy_static;

lazy_static! {
    static ref NEIGHBORHOOD: EnumMap<Side, EnumMap<Side, bool>> = enum_map! {
        Side::A => enum_map! { Side::E | Side::B => true, _ => false },
        Side::B => enum_map! { Side::A | Side::C => true, _ => false },
        Side::C => enum_map! { Side::B | Side::D => true, _ => false },
        Side::D => enum_map! { Side::C | Side::E => true, _ => false },
        Side::E => enum_map! { Side::D | Side::A => true, _ => false },
    };
}

#[derive(Debug, Clone, Copy, Eq, PartialEq, Ord, PartialOrd, Hash, Enum)]
pub enum Side {
    A = 0,
    B = 1,
    C = 2,
    D = 3,
    E = 4,
}

#[derive(Debug, Clone, Copy, Eq, PartialEq, Ord, PartialOrd, Hash, Enum)]
pub enum Vertex {
    AB = 0,
    BC = 1,
    CD = 2,
    DE = 3,
    EA = 4,
}

impl Vertex {
    pub fn iter() -> impl ExactSizeIterator<Item = Self> {
        use Vertex::*;
        [AB, BC, CD, DE, EA].iter().cloned()
    }
}

fn adjacent(a: Side, b: Side) -> bool {
    NEIGHBORHOOD[a][b]
}

#[derive(Debug, Clone, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct NodeString {
    path: Vec<Side>,
}

impl NodeString {
    pub fn new() -> Self {
        Self { path: vec![] }
    }

    pub fn append(&mut self, new_segment: Side) {
        let mut insertion_point = self.path.len();

        for (index, &segment) in self.path.iter().enumerate().rev() {
            if segment == new_segment {
                self.path.remove(index);
                return;
            }

            if !adjacent(segment, new_segment) {
                break;
            }

            if new_segment < segment {
                insertion_point = index;
            }
        }

        self.path.insert(insertion_point, new_segment);
    }

    pub fn backtrack(&mut self) {
        self.path.pop();
    }

    pub fn has_child(&self, new_segment: Side) -> bool {
        for &segment in self.path.iter().rev() {
            if segment == new_segment {
                return false;
            }

            if !adjacent(segment, new_segment) {
                return true;
            }

            if new_segment < segment {
                return false;
            }
        }
        true
    }

    pub fn len(&self) -> usize {
        self.path.len()
    }

    pub fn is_empty(&self) -> bool {
        self.path.is_empty()
    }

    pub fn iter(&self) -> std::slice::Iter<Side> {
        self.path.iter()
    }
}

impl Default for NodeString {
    fn default() -> Self {
        NodeString::new()
    }
}
