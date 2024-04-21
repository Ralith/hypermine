use std::ops::{Index, IndexMut};

use serde::{Deserialize, Serialize};

use crate::dodeca::Side;

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
pub enum CoordSign {
    Plus = 1,
    Minus = -1,
}

impl CoordSign {
    /// Iterates through the two possible coordinate directions
    pub fn iter() -> impl ExactSizeIterator<Item = Self> {
        [CoordSign::Plus, CoordSign::Minus].into_iter()
    }
}

impl std::ops::Mul for CoordSign {
    type Output = CoordSign;

    fn mul(self, rhs: Self) -> Self::Output {
        match self == rhs {
            true => CoordSign::Plus,
            false => CoordSign::Minus,
        }
    }
}

impl std::ops::MulAssign for CoordSign {
    fn mul_assign(&mut self, rhs: Self) {
        *self = *self * rhs;
    }
}

/// Coordinates for a discrete voxel within a chunk, not including margins
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct Coords(pub [u8; 3]);

impl Coords {
    /// Returns the array index in `VoxelData` corresponding to these coordinates
    pub fn to_index(self, chunk_size: u8) -> usize {
        let chunk_size_with_margin = chunk_size as usize + 2;
        (self.0[0] as usize + 1)
            + (self.0[1] as usize + 1) * chunk_size_with_margin
            + (self.0[2] as usize + 1) * chunk_size_with_margin.pow(2)
    }

    /// Returns the x, y, or z coordinate that would correspond to the voxel meeting the chunk boundary in the direction of `sign`
    pub fn edge_coord(chunk_size: u8, sign: CoordSign) -> u8 {
        match sign {
            CoordSign::Plus => chunk_size - 1,
            CoordSign::Minus => 0,
        }
    }
}

impl Index<CoordAxis> for Coords {
    type Output = u8;

    #[inline]
    fn index(&self, coord_axis: CoordAxis) -> &u8 {
        self.0.index(coord_axis as usize)
    }
}

impl IndexMut<CoordAxis> for Coords {
    #[inline]
    fn index_mut(&mut self, coord_axis: CoordAxis) -> &mut u8 {
        self.0.index_mut(coord_axis as usize)
    }
}

/// Represents one of the six main directions within a chunk: positive or negative x, y, and z.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ChunkDirection {
    pub axis: CoordAxis,
    pub sign: CoordSign,
}

impl ChunkDirection {
    pub const PLUS_X: Self = ChunkDirection {
        axis: CoordAxis::X,
        sign: CoordSign::Plus,
    };
    pub const PLUS_Y: Self = ChunkDirection {
        axis: CoordAxis::Y,
        sign: CoordSign::Plus,
    };
    pub const PLUS_Z: Self = ChunkDirection {
        axis: CoordAxis::Z,
        sign: CoordSign::Plus,
    };
    pub const MINUS_X: Self = ChunkDirection {
        axis: CoordAxis::X,
        sign: CoordSign::Minus,
    };
    pub const MINUS_Y: Self = ChunkDirection {
        axis: CoordAxis::Y,
        sign: CoordSign::Minus,
    };
    pub const MINUS_Z: Self = ChunkDirection {
        axis: CoordAxis::Z,
        sign: CoordSign::Minus,
    };

    pub fn iter() -> impl ExactSizeIterator<Item = ChunkDirection> {
        [
            Self::PLUS_X,
            Self::PLUS_Y,
            Self::PLUS_Z,
            Self::MINUS_X,
            Self::MINUS_Y,
            Self::MINUS_Z,
        ]
        .into_iter()
    }
}

/// Represents one of the 6 possible permutations a chunk's axes can have, useful for comparing the canonical sides of one chunk to an adjacent chunk.
/// This is analogous to a 3x3 rotation/reflection matrix with a restricted domain.
/// Note that it may make sense to define a more general `ChunkOrientation` class that takes three `ChunkDirection`s, to represent
/// any cube rotation/reflection, but no use exists for it yet, so it has not yet been implemented.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ChunkAxisPermutation {
    axes: [CoordAxis; 3],
}

impl ChunkAxisPermutation {
    pub const IDENTITY: Self = ChunkAxisPermutation {
        axes: [CoordAxis::X, CoordAxis::Y, CoordAxis::Z],
    };

    /// Constructs a `ChunkAxisPermutation` that, when left-multiplying a set of coordinates, moves from `from`'s reference
    /// frame to `to`'s reference frame, where `from` and `to` are represented as three dodeca sides incident to a vertex
    /// that determine the orientation of a chunk.
    pub fn from_permutation(from: [Side; 3], to: [Side; 3]) -> Self {
        assert!(from[0] != from[1] && from[0] != from[2] && from[1] != from[2]);
        assert!(to[0] != to[1] && to[0] != to[2] && to[1] != to[2]);
        ChunkAxisPermutation {
            axes: from.map(|f| {
                CoordAxis::try_from(
                    to.iter()
                        .position(|&t| f == t)
                        .expect("from and to must have same set of sides"),
                )
                .unwrap()
            }),
        }
    }
}

impl Index<CoordAxis> for ChunkAxisPermutation {
    type Output = CoordAxis;

    fn index(&self, index: CoordAxis) -> &Self::Output {
        &self.axes[index as usize]
    }
}

impl std::ops::Mul<Coords> for ChunkAxisPermutation {
    type Output = Coords;

    fn mul(self, rhs: Coords) -> Self::Output {
        let mut result = Coords([0; 3]);
        for axis in CoordAxis::iter() {
            result[self[axis]] = rhs[axis];
        }
        result
    }
}

impl std::ops::Mul<ChunkDirection> for ChunkAxisPermutation {
    type Output = ChunkDirection;

    fn mul(self, rhs: ChunkDirection) -> Self::Output {
        ChunkDirection {
            axis: self[rhs.axis],
            sign: rhs.sign,
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::dodeca::Vertex;

    use super::*;

    fn coords_to_vector3(coords: Coords) -> na::Vector3<i32> {
        na::Vector3::new(
            coords[CoordAxis::X] as i32,
            coords[CoordAxis::Y] as i32,
            coords[CoordAxis::Z] as i32,
        )
    }

    fn coord_axis_to_vector3(coord_axis: CoordAxis) -> na::Vector3<i32> {
        let mut vector = na::Vector3::new(0, 0, 0);
        vector[coord_axis as usize] = 1;
        vector
    }

    fn chunk_direction_to_vector3(chunk_direction: ChunkDirection) -> na::Vector3<i32> {
        let mut vector = na::Vector3::new(0, 0, 0);
        vector[chunk_direction.axis as usize] = chunk_direction.sign as i32;
        vector
    }

    fn chunk_axis_permutation_to_matrix3(
        chunk_axis_permutation: ChunkAxisPermutation,
    ) -> na::Matrix3<i32> {
        na::Matrix::from_columns(&chunk_axis_permutation.axes.map(coord_axis_to_vector3))
    }

    // Helper function to return all permutations as a list of ordered triples
    fn get_all_permutations() -> Vec<(usize, usize, usize)> {
        let mut permutations = vec![];
        for i in 0..3 {
            for j in 0..3 {
                if j == i {
                    continue;
                }
                for k in 0..3 {
                    if k == i || k == j {
                        continue;
                    }
                    permutations.push((i, j, k));
                }
            }
        }
        permutations
    }

    #[test]
    fn test_chunk_axis_permutation() {
        let sides = Vertex::A.canonical_sides();

        let example_coords = Coords([3, 5, 9]);

        for (i, j, k) in get_all_permutations() {
            let permutation = ChunkAxisPermutation::from_permutation(
                [sides[0], sides[1], sides[2]],
                [sides[i], sides[j], sides[k]],
            );

            // Test that the permutation goes in the expected direction
            assert_eq!(
                permutation * example_coords,
                Coords([
                    example_coords.0[i],
                    example_coords.0[j],
                    example_coords.0[k]
                ])
            );

            // Test that the multiplication operations are consistent with matrix multiplication
            assert_eq!(
                coords_to_vector3(permutation * example_coords),
                chunk_axis_permutation_to_matrix3(permutation) * coords_to_vector3(example_coords)
            );
            for chunk_direction in ChunkDirection::iter() {
                assert_eq!(
                    chunk_direction_to_vector3(permutation * chunk_direction),
                    chunk_axis_permutation_to_matrix3(permutation)
                        * chunk_direction_to_vector3(chunk_direction)
                )
            }
        }
    }
}
