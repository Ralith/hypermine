use crate::sim::{DualGraph, VoxelData};
use common::{
    dodeca::Side,
    graph::NodeId,
    world::{Material, SUBDIVISION_FACTOR},
    math,
};

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum NodeStateKind {
    Horosphere(na::Vector4<f64>, f64)
}
use NodeStateKind::*;
use common::dodeca::Vertex;

impl NodeStateKind {
    #[inline]
    pub fn root() -> Self {
        Horosphere(na::Vector4::new(1.0, 0.0, 0.0, 1.0), 1.0)
    }

    /// What state comes after this state, from a given side?
    pub fn child_with_spice(self, spice: u64, side: Side) -> Self {
        let Horosphere(v, factor) = self;
        let mut w: na::Vector4<f64> = side.reflection() * v;
        w[3] = (w[0]*w[0] + w[1]*w[1] + w[2]*w[2]).sqrt(); // Make sure center stays ideal point
        let norm = w.norm();
        Horosphere(w / norm, factor * norm)
    }

    pub fn solid_material(self) -> Option<Material> {
        match self {
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
    #[inline]
    pub fn root() -> Self {
        Self {
            kind: NodeStateKind::root(),
            spice: 0,
            scalar_field: 0,
        }
    }

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

    pub fn write_chunk_voxels(
        &self,
        cube_type: Vertex,
        voxels: &mut VoxelData,
    ) {
        if let Horosphere(center, factor) = self.kind {
            for z in 0..SUBDIVISION_FACTOR {
                for y in 0..SUBDIVISION_FACTOR {
                    for x in 0..SUBDIVISION_FACTOR {
                        let i = chunk_index(x, y, z);
                        let ix = (0.5 + (x as f64)) / (SUBDIVISION_FACTOR as f64);
                        let iy = (0.5 + (y as f64)) / (SUBDIVISION_FACTOR as f64);
                        let iz = (0.5 + (z as f64)) / (SUBDIVISION_FACTOR as f64);
                        voxels.data_mut()[i] = if is_inside_horosphere(&center, factor, ix, iy, iz, cube_type) {Material::Void} else {Material::Stone};
                    }
                }
            }
        }
    }

    pub fn write_chunk(&self, cube_type: Vertex, voxels: &mut VoxelData) {
        match voxels {
            VoxelData::Uninitialized => {
                self.write_chunk_voxels(cube_type, voxels);
            }
            _ => {}
        }
    }
}

#[inline]
fn chunk_index(x: usize, y: usize, z: usize) -> usize {
    let v = na::Vector3::new(x, y, z)
        + na::Vector3::repeat(1);

    v.x + v.y * (SUBDIVISION_FACTOR + 2) + v.z * (SUBDIVISION_FACTOR + 2).pow(2)
}

#[inline]
fn hash(a: u64, b: u64) -> u64 {
    use std::ops::BitXor;
    a.rotate_left(5).bitxor(b).wrapping_mul(0x517cc1b727220a95)
}

#[inline]
fn is_inside_horosphere(center: &na::Vector4<f64>, factor: f64, cx: f64, cy: f64, cz: f64, cube_type: Vertex) -> bool {
    let pos: na::Vector4<f64> = math::lorentz_normalize(&(cube_type.cube_to_node() * na::Vector4::new(cx, cy, cz, 1.0)));
    math::mip(&pos, center) * factor > -1.0
}