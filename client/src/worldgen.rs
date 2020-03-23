use crate::sim::{DualGraph, VoxelData};
use common::{
    dodeca::Side,
    graph::NodeId,
    world::{Material, SUBDIVISION_FACTOR},
};

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum NodeStateKind {
    RootSky,
    Hole(usize),
    Sky,
    DeepSky,
    Land,
    DeepLand,
}
use NodeStateKind::*;

impl NodeStateKind {
    pub const ROOT: Self = RootSky;

    /// What state comes after this state, from a given side?
    pub fn child_with_spice(self, spice: u64, side: Side) -> Self {
        match (self, side) {
            (RootSky, _) => match side {
                _ if side.adjacent_to(Side::A) => Land,
                Side::A => Sky,
                Side::J => Hole(5),
                _ => DeepLand,
            },
            (Hole(0), _) => match side {
                Side::A | Side::J => Hole(5),
                _ => Land,
            },
            (Hole(n), _) => match side {
                Side::A | Side::J => Hole(n - 1),
                _ => DeepLand,
            },
            (_, Side::A) => match self {
                Sky => Land,
                Land => Sky,
                _ => self,
            },
            _ if side.adjacent_to(Side::A) => self,
            (Sky, _) => DeepSky,
            (Land, _) => DeepLand,
            _ => self,
        }
    }

    pub fn solid_material(self) -> Option<Material> {
        match self {
            Hole(_) | RootSky | Sky | DeepSky => Some(Material::Void),
            DeepLand => Some(Material::Stone),
            _ => None,
        }
    }
}

pub struct NodeState {
    kind: NodeStateKind,
    spice: u64,
    scalar_field: i64,
}
impl NodeState {
    pub const ROOT: Self = Self {
        kind: NodeStateKind::ROOT,
        spice: 0,
        scalar_field: 0,
    };

    pub fn child(&self, graph: &DualGraph, node: NodeId, side: Side) -> Self {
        let node_length = graph.length(node);

        let spice = graph
            .descenders(node)
            // you have to factor in the side representations so that nodes
            // with length 1 still get interesting hashes
            .map(|(s, n)| hash(graph.get(n).as_ref().unwrap().spice, s as u64))
            // now we mix together all the local hashes of the neighbors for this new node's
            // unique hash.
            .fold(hash(0, node_length as u64), |acc, x| hash(acc, x));

        let mut d = graph
            .descenders(node)
            .map(|(s, n)| (s, graph.get(n).as_ref().unwrap().scalar_field));
        let scalar_field = match (d.next(), d.next()) {
            (Some(_), None) => {
                let parent_side = graph.parent(node).unwrap();
                let parent_node = graph.neighbor(node, parent_side).unwrap();
                let parent_scalar = graph.get(parent_node).as_ref().unwrap().scalar_field;
                parent_scalar
                    + match spice % 2 {
                        0 => -1,
                        1 => 1,
                        _ => unreachable!(),
                    }
            }
            (Some((a_side, a_scalar)), Some((b_side, b_scalar))) => {
                let ab_node = graph.neighbor(graph.neighbor(node, a_side).unwrap(), b_side).unwrap();
                let ab_scalar = graph.get(ab_node).as_ref().unwrap().scalar_field;
                a_scalar + (b_scalar - ab_scalar)
            }
            _ => unreachable!()
        };

        Self {
            kind: self.kind.clone().child_with_spice(spice, side),
            spice: spice,
            scalar_field,
        }
    }

    pub fn write_subchunk_voxels(
        &self,
        voxels: &mut VoxelData,
        subchunk_offset: na::Vector3<usize>,
    ) {
        // half of SUBDIVISION_FACTOR, which is the width/height/depth of a subchunk.
        const GAP: usize = 0;
        const SUB: usize = (SUBDIVISION_FACTOR + 2) / 2;
        match self.kind {
            Land => {
                for z in GAP..(SUB - GAP) {
                    for y in GAP..(SUB - GAP) {
                        for x in GAP..((self.scalar_field as usize).min(SUB).max(1) - GAP) {
                            let i = subchunk_index(x, y, z, subchunk_offset);
                            voxels.data_mut()[i] = match (self.spice) % 3 {
                                0 => Material::Stone,
                                1 => Material::Dirt,
                                2 => Material::Sand,
                                _ => unreachable!(),
                            };
                        }
                    }
                }
            }
            _ if self.kind.solid_material().is_some() => {
                for z in GAP..(SUB - GAP) {
                    for y in GAP..(SUB - GAP) {
                        for x in GAP..(SUB - GAP) {
                            let i = subchunk_index(x, y, z, subchunk_offset);
                            voxels.data_mut()[i] = self.kind.solid_material().unwrap();
                        }
                    }
                }
            }
            _ => unreachable!(),
        }
    }

    pub fn write_chunk(&self, voxels: &mut VoxelData, subchunk_offset: na::Vector3<usize>) {
        if let Some(material) = self.kind.solid_material() {
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

#[inline]
fn hash(a: u64, b: u64) -> u64 {
    use std::ops::BitXor;
    a.rotate_left(5).bitxor(b).wrapping_mul(0x517cc1b727220a95)
}
