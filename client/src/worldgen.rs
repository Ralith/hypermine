use crate::sim::VoxelData;
use common::{
    dodeca::Side,
    world::{Material, SUBDIVISION_FACTOR},
};

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum NodeState {
    RootSky,
    Sky,
    DeepSky,
    Land,
    DeepLand,
    Sea,
    DeepSea,
}
use NodeState::*;

impl NodeState {
    pub const ROOT: Self = RootSky;

    /// What state comes after this state, from a given side?
    pub fn child(mut self, i: Side) -> Self {
        if self == RootSky {self = Land};
        match (self, i) {
            (Land, Side::A) => Sky,
            (Land, Side::B) => Sea,
            (Land, _) if !i.adjacent_to(Side::A) => DeepLand,
            (Sea, Side::A) => Sky,
            (Sea, Side::B) => Land,
            (Sea, _) if !i.adjacent_to(Side::A) => DeepSea,
            (DeepLand, Side::B) => DeepSea,
            (DeepSea, Side::B) => DeepLand,
            (Sky, _) if !i.adjacent_to(Side::A) => DeepSky,
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
            Land | DeepLand => Some(Material::Stone),
            Sea | DeepSea => Some(Material::Sand),
            RootSky | Sky | DeepSky => Some(Material::Void),
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
