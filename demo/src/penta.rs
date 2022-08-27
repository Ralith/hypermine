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

impl Side {
    pub fn iter() -> impl ExactSizeIterator<Item = Self> {
        use Side::*;
        [A, B, C, D, E].iter().cloned()
    }

    pub fn is_neighbor(&self, side: Side) -> bool {
        NEIGHBORHOOD[*self][side]
    }
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
