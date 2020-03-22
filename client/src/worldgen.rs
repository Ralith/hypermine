use crate::sim::VoxelData;
use common::{
    dodeca::Side,
    world::{Material, SUBDIVISION_FACTOR},
};

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum NodeState {
    RootSky,
    Hole(usize),
    Sky,
    DeepSky,
    Land,
    DeepLand,
}
use NodeState::*;

impl NodeState {
    pub const ROOT: Self = RootSky;

    /// What state comes after this state, from a given side?
    pub fn child(self, i: Side) -> Self {
        match (self, i) {
            (RootSky, _) => match i {
                _ if i.adjacent_to(Side::A) => Land,
                Side::A => Sky,
                Side::J => Hole(5),
                _ => DeepLand,
            },
            (Hole(0), _) => match i {
                Side::A | Side::J => Hole(5),
                _ => Land,
            },
            (Hole(n), _) => match i {
                Side::A | Side::J => Hole(n - 1),
                _ => DeepLand,
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

    pub fn write_subchunk_voxels(
        self,
        voxels: &mut VoxelData,
        subchunk_offset: na::Vector3<usize>,
    ) {
        // half of SUBDIVISION_FACTOR, which is the width/height/depth of a subchunk.
        const GAP: usize = 0;
        const SUB: usize = (SUBDIVISION_FACTOR + 2) / 2;
        match self {
            Land => {
                for z in GAP..(SUB - GAP) {
                    for y in GAP..(SUB - GAP) {
                        for x in GAP..(SUB - GAP) {
                            let i = subchunk_index(x, y, z, subchunk_offset);
                            voxels.data_mut()[i] = match ((z + y) / 2 + (x % 3)) % 3 {
                                0 => Material::Stone,
                                1 => Material::Dirt,
                                2 => Material::Sand,
                                _ => unreachable!(),
                            };
                        }
                    }
                }
            }
            _ if self.solid_material().is_some() => {
                for z in GAP..(SUB - GAP) {
                    for y in GAP..(SUB - GAP) {
                        for x in GAP..(SUB - GAP) {
                            let i = subchunk_index(x, y, z, subchunk_offset);
                            voxels.data_mut()[i] = self.solid_material().unwrap();
                        }
                    }
                }
            }
            _ => unreachable!(),
        }
    }

    pub fn solid_material(self) -> Option<Material> {
        match self {
            Hole(_) | RootSky | Sky | DeepSky => Some(Material::Void),
            DeepLand => Some(Material::Stone),
            _ => None,
        }
    }

    pub fn write_chunk(self, voxels: &mut VoxelData, subchunk_offset: na::Vector3<usize>) {
        if let Some(material) = self.solid_material() {
            match voxels {
                // who knows, maybe this voxel is surrounded by the same type of node.
                VoxelData::Uninitialized => {
                    *voxels = VoxelData::Solid(material);
                    return;
                }
                // why change the voxel? it's already filled with what we'd fill it with.
                VoxelData::Solid(voxel_mat) if *voxel_mat == material => return,
                _ => {}
            }
        }

        // sometimes there's just no way around writing into the subchunk.
        self.write_subchunk_voxels(voxels, subchunk_offset);
    }
}

#[inline]
fn subchunk_index(x: usize, y: usize, z: usize, subchunk_offset: na::Vector3<usize>) -> usize {
    let v = na::Vector3::new(x, y, z)
        //+ na::Vector3::repeat(1)
        + (subchunk_offset * ((SUBDIVISION_FACTOR + 2) / 2));

    v.x + v.y * (SUBDIVISION_FACTOR + 2) + v.z * (SUBDIVISION_FACTOR + 2).pow(2)
}
