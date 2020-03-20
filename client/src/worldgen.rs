use crate::sim::VoxelData;
use common::{
    cursor::{Cursor, Dir},
    world::{Material, SUBDIVISION_FACTOR},
    dodeca::Side,
};

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum NodeState {
    RootSky,
    Sky,
    DeepSky,
    Land,
    DeepLand,
}
use NodeState::*;

impl NodeState {
    pub const ROOT: Self = RootSky;

    /// If a vertex is at the junction of 8 different types of nodes,
    /// which should it manifest?
    fn precedence(self) -> u8 {
        match self {
            RootSky => 3,
            DeepLand => 2,
            Land => 1,
            Sky | DeepSky => 0,
        }
    }

    pub fn child(self, i: Side) -> Self {
        match (self, i) {
            (RootSky, _) => {
                match i {
                    _ if i.adjacent_to(Side::A) => Land,
                    Side::A => Sky,
                    _ => DeepLand
                }
            },
            (_, Side::A) => match self {
                Sky => Land,
                Land => Sky,
                _ => self,
            },
            _ if i.adjacent_to(Side::A) => self,
            (Sky, _) => DeepSky,
            (Land, _) => DeepLand,
            _ => self,
        }
    }

    pub fn voxels(incidents: Vec<NodeState>, cursor: Cursor) -> VoxelData {
        assert_eq!(incidents.len(), 8);
        let first = incidents.get(0).cloned();

        // cubes with homogeneous incidents can't be seen,
        // they're incased in things that can be seen
        if incidents
                .iter()
                .fold(first, |a, x| a.filter(|a| a == x))
                .is_some()
        {
            return VoxelData::Empty;
        }

        const GAP: usize = 0;
        match incidents.iter().max_by(|a, b| a.precedence().cmp(&b.precedence())).unwrap() {
            Land => {
                let mut data = empty_chunk_data();
                for z in GAP..(SUBDIVISION_FACTOR - GAP) {
                    for y in GAP..(SUBDIVISION_FACTOR - GAP) {
                        for x in GAP..((SUBDIVISION_FACTOR / 2) - GAP) {
                            data[padded_chunk_index(x, y, z)] = match (z + y)/2 % 3 {
                                0 => Material::Stone,
                                1 => Material::Dirt,
                                _ => Material::Sand,
                            };
                        }
                    }
                }
                VoxelData::Dense(data)
            },
            DeepLand => {
                let mut data = empty_chunk_data();
                for z in GAP..(SUBDIVISION_FACTOR - GAP) {
                    for y in GAP..(SUBDIVISION_FACTOR - GAP) {
                        for x in GAP..((SUBDIVISION_FACTOR) - GAP) {
                            data[padded_chunk_index(x, y, z)] = Material::Stone;
                        }
                    }
                }
                VoxelData::Dense(data)
            }
            RootSky | Sky | DeepSky => VoxelData::Empty,
        }
    }
}

#[inline]
fn empty_chunk_data() -> Box<[Material]> {
    (0..(SUBDIVISION_FACTOR + 2).pow(3))
        .map(|_| Material::Void)
        .collect::<Vec<_>>()
        .into_boxed_slice()
}

#[inline]
fn padded_chunk_index(x: usize, y: usize, z: usize) -> usize {
    (x + 1) + (y + 1) * (SUBDIVISION_FACTOR + 2) + (z + 1) * (SUBDIVISION_FACTOR + 2).pow(2)
}

