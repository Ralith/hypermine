use std::ops::{Index, IndexMut};

use serde::{Deserialize, Serialize};

/// Represents a particular axis in a voxel grid
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CoordAxis {
    X = 0,
    Y = 1,
    Z = 2,
}

/// Trying to convert a `usize` to a `CoordAxis` returns this struct if the provided
/// `usize` is out-of-bounds
#[derive(Debug, Clone, Copy)]
pub struct CoordAxisOutOfBounds;

impl CoordAxis {
    /// Iterates through the the axes in ascending order
    pub fn iter() -> impl ExactSizeIterator<Item = Self> {
        [Self::X, Self::Y, Self::Z].into_iter()
    }

    /// Returns the pair axes orthogonal to the current axis
    pub fn other_axes(self) -> [Self; 2] {
        match self {
            Self::X => [Self::Y, Self::Z],
            Self::Y => [Self::Z, Self::X],
            Self::Z => [Self::X, Self::Y],
        }
    }
}

impl TryFrom<usize> for CoordAxis {
    type Error = CoordAxisOutOfBounds;

    fn try_from(value: usize) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Self::X),
            1 => Ok(Self::Y),
            2 => Ok(Self::Z),
            _ => Err(CoordAxisOutOfBounds),
        }
    }
}

/// Represents a direction in a particular axis. This struct is meant to be used with a coordinate axis,
/// so when paired with the X-axis, it represents the postitive X-direction when set to Plus and the
/// negative X-direction when set to Minus.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CoordDirection {
    Plus = 1,
    Minus = -1,
}

impl CoordDirection {
    /// Iterates through the two possible coordinate directions
    pub fn iter() -> impl ExactSizeIterator<Item = Self> {
        [CoordDirection::Plus, CoordDirection::Minus].into_iter()
    }
}

/// Coordinates for a discrete voxel within a chunk, not including margins
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct Coords(pub [u8; 3]);

impl Coords {
    /// Returns the array index in `VoxelData` corresponding to these coordinates
    pub fn to_index(&self, chunk_size: u8) -> usize {
        let chunk_size_with_margin = chunk_size as usize + 2;
        (self.0[0] as usize + 1)
            + (self.0[1] as usize + 1) * chunk_size_with_margin
            + (self.0[2] as usize + 1) * chunk_size_with_margin.pow(2)
    }
}

impl Index<CoordAxis> for Coords {
    type Output = u8;

    fn index(&self, coord_axis: CoordAxis) -> &u8 {
        self.0.index(coord_axis as usize)
    }
}

impl IndexMut<CoordAxis> for Coords {
    fn index_mut(&mut self, coord_axis: CoordAxis) -> &mut u8 {
        self.0.index_mut(coord_axis as usize)
    }
}
