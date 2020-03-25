use crate::sim::{DualGraph, VoxelData};
use common::{
    math::{mip, lorentz_normalize, origin},
    dodeca::{Side, Vertex},
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
        }    }

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
    elev: i64,
    temp: i64,
    rain: i64,
}
impl NodeState {
    pub const ROOT: Self = Self {
        kind: NodeStateKind::ROOT,
        spice: 0,
        elev: 0,
        temp: 0,
        rain: 0,
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
            .map(|(s, n)| (s, graph.get(n).as_ref().unwrap()));
        let (elev, temp, rain) = match (d.next(), d.next()) {
            (Some(_), None) => {
                let parent_side = graph.parent(node).unwrap();
                let parent_node = graph.neighbor(node, parent_side).unwrap();
                let parent_state = graph.get(parent_node).as_ref().unwrap();
                (
                    parent_state.elev + (1 - (spice as i64 % 3)),
                    parent_state.temp + (1 - (spice as i64 % 3)),
                    parent_state.rain + (1 - (spice as i64 % 3)),
                )
            }
            (Some((a_side, a_state)), Some((b_side, b_state))) => {
                let ab_node = graph.neighbor(graph.neighbor(node, a_side).unwrap(), b_side).unwrap();
                let ab_state = graph.get(ab_node).as_ref().unwrap();
                (
                    a_state.elev + (b_state.elev - ab_state.elev),
                    a_state.temp + (b_state.temp - ab_state.temp),
                    a_state.rain + (b_state.rain - ab_state.rain),
                )
            }
            _ => unreachable!()
        };

                             
        Self {
            kind: self.kind.clone().child_with_spice(spice, side),
            spice,
            elev,
            temp,
            rain,
        }
    }

    pub fn write_subchunk(
        &self,
        voxels: &mut VoxelData,
        subchunk_offset: na::Vector3<usize>,
        vertex: Vertex,
        elevations: [f64; 8]
    ) {
        let vertex_transform = vertex.cube_to_node();

        // half of SUBDIVISION_FACTOR, which is the width/height/depth of a subchunk.
        const GAP: usize = 0;
        const SUB: usize = SUBDIVISION_FACTOR / 2;
        match self.kind {
            Land => {
                for z in GAP..(SUB - GAP) {
                    for y in GAP..(SUB - GAP) {
                        for x in GAP..(SUB - GAP) {
                            // chunk indexing
                            let a = subchunk_offset * SUB + na::Vector3::new(x, y, z);
                            let i = index(a);

                            // distance from surface
                            let point = na::Vector4::new(a.x, a.y, a.z, 0)
                                .map(|x| x as f64 + 1.5) / SUBDIVISION_FACTOR as f64;
                            let p = lorentz_normalize(&(vertex_transform * point));

                            if mip(&p, &*SURFACE).asinh() < trilerp(&elevations, point.xyz())/10.0 {
                                let z_border = a.z == SUBDIVISION_FACTOR-1 || a.z == 0;
                                let y_border = a.y == SUBDIVISION_FACTOR-1 || a.y == 0;
                                voxels.data_mut()[i] = if z_border && y_border {
                                    Material::Stone
                                } else if z_border || y_border {
                                    Material::Dirt
                                } else {
                                    Material::Sand
                                };
                            }
                        }
                    }
                }
            }
            _ if self.kind.solid_material().is_some() => {
                for z in GAP..(SUB - GAP) {
                    for y in GAP..(SUB - GAP) {
                        for x in GAP..(SUB - GAP) {
                            let i = index(subchunk_offset*SUB + na::Vector3::new(x, y, z));
                            voxels.data_mut()[i] = self.kind.solid_material().unwrap();
                        }
                    }
                }
            }
            _ => unreachable!(),
        }
    }
}

lazy_static::lazy_static! {
    static ref SURFACE: na::Vector4<f64> = {
        let n = Side::A.reflection().map(|x| x as f64) * origin() - origin();
        lorentz_normalize(&n)
    };
}

pub fn voxels(graph: &mut DualGraph, node: NodeId, cube: Vertex) -> VoxelData {

    // this is a bit cursed, but I don't want to collect into a vec because perf,
    // and I can't just return an iterator because then something still references graph.
    let elevations = {
        let mut e = cube
            .dual_vertices()
            .map(|(_, path)| path.fold(node, |node, side| graph.neighbor(node, side).unwrap()))
            .map(|n| graph.get(n).as_ref().unwrap().elev as f64);
        let e1 = e.next().unwrap();
        let e5 = e.next().unwrap();
        let e3 = e.next().unwrap();
        let e7 = e.next().unwrap();
        let e2 = e.next().unwrap();
        let e6 = e.next().unwrap();
        let e4 = e.next().unwrap();
        let e8 = e.next().unwrap();
        [e1, e2, e3, e4, e5, e6, e7, e8]
    };

    let mut voxels = VoxelData::Uninitialized;

    for ([x, y, z], path) in cube.dual_vertices() {
        let state = graph
            .get(path.fold(node, |node, side| graph.neighbor(node, side).unwrap()))
            .as_ref()
            .unwrap();
        let subchunk_offset = na::Vector3::new(x as usize, y as usize, z as usize);
        state.write_subchunk(&mut voxels, subchunk_offset, cube, elevations);
    }

    voxels
}

#[test]
fn elevation_grabbing() {
    let mut g = DualGraph::new();
    for (i, path) in Vertex::A.dual_vertices().map(|(_, p)| p).enumerate() {
        // this line could panic if the paths in dual_vertices weren't outlined like they were.
        path.fold(NodeId::ROOT, |node, side| {
            g.neighbor(node, side).unwrap_or_else(|| {
                let new_node = g.ensure_neighbor(node, side);
                let mut state = NodeState::ROOT;
                state.elev = i as i64;
                *g.get_mut(new_node) = Some(state);
                new_node
            })
        });
    }
    /* should create:
     *
     * CORNER                 :  ELEVATION
     *                        :
     * [false, false, false]) :  1
     * [true, false, false])  :  5
     * [false, true, false])  :  3
     * [true, true, false])   :  7
     * [false, false, true])  :  2
     * [true, false, true])   :  6 
     * [false, true, true])   :  4
     * [true, true, true])    :  8
     */
}

fn trilerp<N: na::RealField>(a: &[N], t: na::Vector3<N>) -> N {
    fn lerp<N: na::RealField>(a: &[N], t: N) -> N {
        a[0] * (N::one() - t) + a[1] * t
    }
    fn bilerp<N: na::RealField>(a: &[N], t: na::Vector2<N>) -> N {
        lerp(&[lerp(&a[0..2], t.x), lerp(&a[2..4], t.x)], t.y)
    }

    lerp(&[bilerp(&a[0..4], t.xy()), bilerp(&a[4..8], t.xy())], t.z)
}
#[test]
fn check_trilerp() {
    assert_eq!(1.0, trilerp(&[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], na::Vector3::new(0.0, 0.0, 0.0)));
    assert_eq!(1.0, trilerp(&[0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], na::Vector3::new(1.0, 0.0, 0.0)));
    assert_eq!(1.0, trilerp(&[0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0], na::Vector3::new(0.0, 1.0, 0.0)));
    assert_eq!(1.0, trilerp(&[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0], na::Vector3::new(1.0, 1.0, 0.0)));
    assert_eq!(1.0, trilerp(&[0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0], na::Vector3::new(0.0, 0.0, 1.0)));
    assert_eq!(1.0, trilerp(&[0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0], na::Vector3::new(1.0, 0.0, 1.0)));
    assert_eq!(1.0, trilerp(&[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0], na::Vector3::new(0.0, 1.0, 1.0)));
    assert_eq!(1.0, trilerp(&[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], na::Vector3::new(1.0, 1.0, 1.0)));
    
    assert_eq!(0.5, trilerp(&[0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0], na::Vector3::new(0.5, 0.5, 0.5)));
    assert_eq!(0.5, trilerp(&[0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0], na::Vector3::new(0.5, 0.5, 0.5)));

    assert_eq!(4.5, trilerp(&[1.0, 5.0, 3.0, 7.0, 2.0, 6.0, 4.0, 8.0], na::Vector3::new(0.5, 0.5, 0.5)));
}

fn index(mut v: na::Vector3<usize>) -> usize {
    v += na::Vector3::repeat(1);

    v.x + v.y * (SUBDIVISION_FACTOR + 2) + v.z * (SUBDIVISION_FACTOR + 2).pow(2)
}

#[inline]
fn hash(a: u64, b: u64) -> u64 {
    use std::ops::BitXor;
    a.rotate_left(5).bitxor(b).wrapping_mul(0x517cc1b727220a95)
}
