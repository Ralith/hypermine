use crate::sim::VoxelData;
use common::{
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
    pub const ROOT: Self = Land;

    /// If a vertex is at the junction of 8 different types of nodes,
    /// which should it manifest?
    pub fn child(self, i: Side) -> Self {
        match (self, i) {
            /*
            (RootSky, _) => {
                match i {
                    _ if i.adjacent_to(Side::A) => Land,
                    Side::A => Sky,
                    _ => DeepLand
                }
            },*/
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

    pub fn fill_subchunk(self, voxels: &mut VoxelData, subchunk_offset: na::Vector3<usize>) {
        // half of SUBDIVISION_FACTOR, which is the width/height/depth of a subchunk.
        const GAP: usize = 0;
        match self {
            RootSky | Sky | DeepSky => {},
            filled => {
                match filled {
                    Land => {
                        for z in GAP..(SUBDIVISION_FACTOR/2 - GAP) {
                            for y in GAP..(SUBDIVISION_FACTOR/2 - GAP) {
                                for x in GAP..(SUBDIVISION_FACTOR/2 - GAP) {
                                    let i = subchunk_index(x, y, z, subchunk_offset);
                                    voxels.data_mut()[i] = match ((z + y)/2 + (x%3)) % 3 {
                                        0 => Material::Stone,
                                        1 => Material::Dirt,
                                        2 => Material::Sand,
                                        _ => unreachable!()
                                    };
                                }
                            }
                        }
                    },
                    DeepLand => {
                        for z in GAP..(SUBDIVISION_FACTOR/2 - GAP) {
                            for y in GAP..(SUBDIVISION_FACTOR/2 - GAP) {
                                for x in GAP..(SUBDIVISION_FACTOR/2 - GAP) {
                                    let i = subchunk_index(x, y, z, subchunk_offset);
                                    voxels.data_mut()[i] = Material::Stone;
                                }
                            }
                        }
                    }
                    _ => unreachable!(),
                };
            }
        }
    }
}

#[inline]
fn subchunk_index(x: usize, y: usize, z: usize, subchunk_offset: na::Vector3<usize>) -> usize {
    let v = na::Vector3::new(x, y, z)
        + na::Vector3::repeat(1)
        + (subchunk_offset * (SUBDIVISION_FACTOR/2));

    v.x
        + v.y * (SUBDIVISION_FACTOR + 2)
        + v.z * (SUBDIVISION_FACTOR + 2).pow(2)
}

