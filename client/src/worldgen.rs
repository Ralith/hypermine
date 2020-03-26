use crate::sim::{DualGraph, VoxelData};
use common::{
    dodeca::{Side, Vertex},
    graph::NodeId,
    math::{lorentz_normalize, mip, origin},
    world::{Material, SUBDIVISION_FACTOR},
};

// chunk filling constants
const GAP: usize = 0;
const SUB: usize = SUBDIVISION_FACTOR / 2;

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
    pub fn child(self, side: Side) -> Self {
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
    surface: Surface,
    spice: u64,
    max_elevation: i64,
    temp: i64,
    rain: i64,
}
impl NodeState {
    pub fn root() -> Self {
        Self {
            kind: NodeStateKind::ROOT,
            surface: Surface::at_root(),
            spice: 0,
            max_elevation: 0,
            temp: 0,
            rain: 0,
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
            .fold(hash(0, node_length as u64), hash);

        let mut d = graph
            .descenders(node)
            .map(|(s, n)| (s, graph.get(n).as_ref().unwrap()));
        let (max_elevation, temp, rain) = match (d.next(), d.next()) {
            (Some(_), None) => {
                let parent_side = graph.parent(node).unwrap();
                let parent_node = graph.neighbor(node, parent_side).unwrap();
                let parent_state = graph.get(parent_node).as_ref().unwrap();
                (
                    parent_state.max_elevation + (1 - (spice as i64 % 30) / 10),
                    parent_state.temp + (1 - (spice as i64 % 15) / 5),
                    parent_state.rain + (1 - (spice as i64 % 90) / 30),
                )
            }
            (Some((a_side, a_state)), Some((b_side, b_state))) => {
                let ab_node = graph
                    .neighbor(graph.neighbor(node, a_side).unwrap(), b_side)
                    .unwrap();
                let ab_state = graph.get(ab_node).as_ref().unwrap();
                (
                    a_state.max_elevation + (b_state.max_elevation - ab_state.max_elevation),
                    a_state.temp + (b_state.temp - ab_state.temp),
                    a_state.rain + (b_state.rain - ab_state.rain),
                )
            }
            _ => unreachable!(),
        };

        Self {
            kind: self.kind.clone().child(side),
            surface: self.surface.reflect(side),
            spice,
            max_elevation,
            temp,
            rain,
        }
    }

    pub fn write_subchunk(
        &self,
        voxels: &mut VoxelData,
        subchunk_offset: na::Vector3<usize>,
        cube: Vertex,
        max_elevations: [f64; 8],
    ) {
        match self.kind {
            Sky | Land => {
                for z in GAP..(SUB - GAP) {
                    for y in GAP..(SUB - GAP) {
                        for x in GAP..(SUB - GAP) {
                            let p = absolute_subchunk_coords(x, y, z, subchunk_offset);
                            let q = relative_subchunk_coords(x, y, z, subchunk_offset);

                            let (voxel_mat, elevation_boost) = match self.rain {
                                r if r > 2 => (Material::Dirt, 3.0),
                                r if r < 1 => (Material::Stone, -3.0),
                                _ => (Material::Sand, -1.0),
                            };

                            // maximum max_elevation for this voxel according to the max_elevations
                            // of the incident nodes that dictate the content of this chunk
                            let max_e = trilerp(&max_elevations, p) - elevation_boost;

                            if self.surface.voxel_elevation(q, cube) < max_e / -10.0 {
                                voxels.data_mut()[index(p)] = voxel_mat;
                            }
                        }
                    }
                }
            }
            _ if self.kind.solid_material().is_some() => {
                for z in GAP..(SUB - GAP) {
                    for y in GAP..(SUB - GAP) {
                        for x in GAP..(SUB - GAP) {
                            let i = index(absolute_subchunk_coords(x, y, z, subchunk_offset));
                            voxels.data_mut()[i] = self.kind.solid_material().unwrap();
                        }
                    }
                }
            }
            _ => unreachable!(),
        }
    }
}

pub fn voxels(graph: &mut DualGraph, node: NodeId, cube: Vertex) -> VoxelData {
    let max_elevations = chunk_incident_max_elevations(graph, node, cube);

    let mut voxels = VoxelData::Uninitialized;

    for ([x, y, z], path) in cube.dual_vertices() {
        let state = graph
            .get(path.fold(node, |node, side| graph.neighbor(node, side).unwrap()))
            .as_ref()
            .unwrap();
        let subchunk_offset = na::Vector3::new(x as usize, y as usize, z as usize);
        state.write_subchunk(&mut voxels, subchunk_offset, cube, max_elevations);
    }

    voxels
}

fn chunk_incident_max_elevations(graph: &DualGraph, node: NodeId, cube: Vertex) -> [f64; 8] {
    let mut e = cube
        .dual_vertices()
        .map(|(_, path)| path.fold(node, |node, side| graph.neighbor(node, side).unwrap()))
        .map(|n| graph.get(n).as_ref().unwrap().max_elevation as f64);

    // this is a bit cursed, but I don't want to collect into a vec because perf,
    // and I can't just return an iterator because then something still references graph.
    [
        e.next().unwrap(),
        e.next().unwrap(),
        e.next().unwrap(),
        e.next().unwrap(),
        e.next().unwrap(),
        e.next().unwrap(),
        e.next().unwrap(),
        e.next().unwrap(),
    ]
}

/// Keeps track of the canonical surface wrt. the NodeState this is stored in
pub struct Surface {
    normal: na::Vector4<f64>,
}

impl Surface {
    /// A Vector pointing up from the surface at the root node.
    fn at_root() -> Self {
        let n = Side::A.reflection().map(|x| x as f64) * origin() - origin();
        Self {
            normal: lorentz_normalize(&n),
        }
    }

    /// Returns a new Surface relative to a node on a certain side of this one
    fn reflect(&self, side: Side) -> Self {
        Self {
            normal: lorentz_normalize(&(side.reflection() * self.normal)),
        }
    }

    /// The max_elevation of a single voxel wrt. this Surface
    fn voxel_elevation(&self, voxel: na::Vector3<f64>, cube: Vertex) -> f64 {
        let pos = lorentz_normalize(&(cube.cube_to_node() * voxel.push(1.0)));
        mip(&pos, &self.normal).asinh()
    }
}

fn trilerp<N: na::RealField>(&[v000, v001, v010, v011, v100, v101, v110, v111]: &[N; 8], t: na::Vector3<N>) -> N {
    fn lerp<N: na::RealField>(v0: N, v1: N, t: N) -> N {
        v0 * (N::one() - t) + v1 * t
    }
    fn bilerp<N: na::RealField>(v00: N, v01: N, v10: N, v11: N, t: na::Vector2<N>) -> N {
        lerp(lerp(v00, v01, t.x), lerp(v10, v11, t.x), t.y)
    }

    lerp(bilerp(v000, v100, v010, v110, t.xy()), bilerp(v001, v101, v011, v111, t.xy()), t.z)
}

/// Turns an x, y, z, index into the voxel data of a subchunk into a
/// coordinate in a unit chunk where the length of each side is one, relative to the canonical node.
fn absolute_subchunk_coords(
    x: usize,
    y: usize,
    z: usize,
    subchunk_offset: na::Vector3<usize>,
) -> na::Vector3<f64> {
    (subchunk_offset * SUB + na::Vector3::new(x, y, z)).map(|x| x as f64 + 0.5)
        / SUBDIVISION_FACTOR as f64
}

/// Turns an x, y, z, index into the voxel data of a subchunk into a
/// coordinate in a unit chunk where the length of each side is one, relative to the node corresponding to the subchunk.
fn relative_subchunk_coords(
    x: usize,
    y: usize,
    z: usize,
    subchunk_offset: na::Vector3<usize>,
) -> na::Vector3<f64> {
    let asc = absolute_subchunk_coords(x, y, z, subchunk_offset);
    na::Vector3::new(
        subchunk_offset[0] as f64 + (1.0 - 2.0 * subchunk_offset[0] as f64) * asc[0],
        subchunk_offset[1] as f64 + (1.0 - 2.0 * subchunk_offset[1] as f64) * asc[1],
        subchunk_offset[2] as f64 + (1.0 - 2.0 * subchunk_offset[2] as f64) * asc[2],
    )
}

fn index(v: na::Vector3<f64>) -> usize {
    chunk_coords_to_index_with_margin(v.map(|x| (x * SUBDIVISION_FACTOR as f64) as usize))
}

fn chunk_coords_to_index_with_margin(mut v: na::Vector3<usize>) -> usize {
    v += na::Vector3::repeat(1);

    // LWM = Length (of cube sides) With Margins
    const LWM: usize = SUBDIVISION_FACTOR + 2;
    v.x + v.y * LWM + v.z * LWM.pow(2)
}

fn hash(a: u64, b: u64) -> u64 {
    use std::ops::BitXor;
    a.rotate_left(5)
        .bitxor(b)
        .wrapping_mul(0x517c_c1b7_2722_0a95)
}

#[cfg(test)]
mod test {
    use super::*;
    use approx::*;

    #[test]
    fn check_surface_flipped() {
        let root = Surface::at_root();
        assert_abs_diff_eq!(
            root.voxel_elevation(na::Vector3::x(), Vertex::A),
            root.voxel_elevation(na::Vector3::x(), Vertex::J) * -1.0,
            epsilon = 1e-5
        );
    }

    #[test]
    fn chunk_indexing_origin() {
        // (0, 0, 0) in localized coords
        let origin_index = 1 + (SUBDIVISION_FACTOR + 2) + (SUBDIVISION_FACTOR + 2).pow(2);

        // simple sanity check
        assert_eq!(
            chunk_coords_to_index_with_margin(na::Vector3::repeat(0)),
            origin_index
        );
    }

    #[test]
    fn chunk_indexing_normalized() {
        let origin_index = 1 + (SUBDIVISION_FACTOR + 2) + (SUBDIVISION_FACTOR + 2).pow(2);
        // (0.5, 0.5, 0.5) in localized coords
        let center_index = chunk_coords_to_index_with_margin(na::Vector3::repeat(SUB));
        // the point farthest from the origin, (1, 1, 1) in localized coords
        let anti_index = chunk_coords_to_index_with_margin(na::Vector3::repeat(SUBDIVISION_FACTOR));

        assert_eq!(index(na::Vector3::repeat(0.0)), origin_index);
        assert_eq!(index(na::Vector3::repeat(0.5)), center_index);
        assert_eq!(index(na::Vector3::repeat(1.0)), anti_index);
    }

    #[test]
    fn chunk_indexing_absolute() {
        let origin_index = 1 + (SUBDIVISION_FACTOR + 2) + (SUBDIVISION_FACTOR + 2).pow(2);
        // (0.5, 0.5, 0.5) in localized coords
        let center_index = chunk_coords_to_index_with_margin(na::Vector3::repeat(SUB));
        // the point farthest from the origin, (1, 1, 1) in localized coords
        let anti_index = chunk_coords_to_index_with_margin(na::Vector3::repeat(SUBDIVISION_FACTOR));

        assert_eq!(
            index(absolute_subchunk_coords(0, 0, 0, na::zero())),
            origin_index
        );

        // biggest possible index in subchunk closest to origin still isn't the center
        assert!(
            index(absolute_subchunk_coords(
                SUB - 1,
                SUB - 1,
                SUB - 1,
                na::zero()
            )) < center_index
        );
        // but the first chunk in the subchunk across from that is
        assert_eq!(
            index(absolute_subchunk_coords(0, 0, 0, na::Vector3::repeat(1))),
            center_index
        );

        // biggest possible index in subchunk closest to anti_origin is still not quite
        // the anti_origin
        assert!(
            index(absolute_subchunk_coords(
                SUB - 1,
                SUB - 1,
                SUB - 1,
                na::Vector3::repeat(1)
            )) < anti_index
        );

        // one is added in the chunk indexing so this works out fine, the
        // domain is still SUBDIVISION_FACTOR because 0 is included.
        assert_eq!(
            index(absolute_subchunk_coords(
                SUB - 1,
                SUB - 1,
                SUB - 1,
                na::Vector3::repeat(1)
            )),
            chunk_coords_to_index_with_margin(na::Vector3::repeat(SUBDIVISION_FACTOR - 1))
        );
    }

    #[test]
    fn check_chunk_incident_max_elevations() {
        let mut g = DualGraph::new();
        for (i, path) in Vertex::A.dual_vertices().map(|(_, p)| p).enumerate() {
            let new_node = path.fold(NodeId::ROOT, |node, side| g.ensure_neighbor(node, side));

            // assigning state
            *g.get_mut(new_node) = Some({
                let mut state = NodeState::root();
                state.max_elevation = i as i64 + 1;
                state
            });
        }

        let max_elevations = chunk_incident_max_elevations(&g, NodeId::ROOT, Vertex::A);
        for (i, max_elevation) in max_elevations.iter().cloned().enumerate() {
            assert_abs_diff_eq!(max_elevation, (i + 1) as f64, epsilon = 1e-8);
        }

        // see corresponding test for trilerp
        let center_max_elevation = trilerp(&max_elevations, na::Vector3::repeat(0.5));
        assert_abs_diff_eq!(center_max_elevation, 4.5, epsilon = 1e-8);

        let mut checked_center = false;
        for ([x, y, z], _path) in Vertex::A.dual_vertices() {
            let subchunk_offset = na::Vector3::new(x as usize, y as usize, z as usize);
            for z in GAP..(SUB - GAP) {
                for y in GAP..(SUB - GAP) {
                    for x in GAP..(SUB - GAP) {
                        let a = subchunk_offset * SUB + na::Vector3::new(x, y, z);
                        let center = na::Vector3::repeat(SUB);
                        if a == center {
                            checked_center = true;
                            let c = center.map(|x| x as f64) / SUBDIVISION_FACTOR as f64;
                            let center_max_elevation = trilerp(&max_elevations, c);
                            assert_abs_diff_eq!(center_max_elevation, 4.5, epsilon = 1e-8);
                        }
                    }
                }
            }
        }

        if !checked_center {
            panic!("Never checked trilerping center max_elevation!");
        }
    }

    #[test]
    fn check_trilerp() {
        assert_abs_diff_eq!(
            1.0,
            trilerp(
                &[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                na::Vector3::new(0.0, 0.0, 0.0)
            ),
            epsilon = 1e-8,
        );
        assert_abs_diff_eq!(
            1.0,
            trilerp(
                &[0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                na::Vector3::new(1.0, 0.0, 0.0)
            ),
            epsilon = 1e-8,
        );
        assert_abs_diff_eq!(
            1.0,
            trilerp(
                &[0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                na::Vector3::new(0.0, 1.0, 0.0)
            ),
            epsilon = 1e-8,
        );
        assert_abs_diff_eq!(
            1.0,
            trilerp(
                &[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                na::Vector3::new(1.0, 1.0, 0.0)
            ),
            epsilon = 1e-8,
        );
        assert_abs_diff_eq!(
            1.0,
            trilerp(
                &[0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                na::Vector3::new(0.0, 0.0, 1.0)
            ),
            epsilon = 1e-8,
        );
        assert_abs_diff_eq!(
            1.0,
            trilerp(
                &[0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                na::Vector3::new(1.0, 0.0, 1.0)
            ),
            epsilon = 1e-8,
        );
        assert_abs_diff_eq!(
            1.0,
            trilerp(
                &[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
                na::Vector3::new(0.0, 1.0, 1.0)
            ),
            epsilon = 1e-8,
        );
        assert_abs_diff_eq!(
            1.0,
            trilerp(
                &[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                na::Vector3::new(1.0, 1.0, 1.0)
            ),
            epsilon = 1e-8,
        );

        assert_abs_diff_eq!(
            0.5,
            trilerp(
                &[0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0],
                na::Vector3::new(0.5, 0.5, 0.5)
            ),
            epsilon = 1e-8,
        );
        assert_abs_diff_eq!(
            0.5,
            trilerp(
                &[0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
                na::Vector3::new(0.5, 0.5, 0.5)
            ),
            epsilon = 1e-8,
        );

        assert_abs_diff_eq!(
            4.5,
            trilerp(
                &[1.0, 5.0, 3.0, 7.0, 2.0, 6.0, 4.0, 8.0],
                na::Vector3::new(0.5, 0.5, 0.5)
            ),
            epsilon = 1e-8,
        );
    }

    #[test]
    fn check_surface_on_plane() {
        assert_abs_diff_eq!(
            Surface::at_root().voxel_elevation(
                na::Vector3::new(0.5, 0.3, 0.9), // The first 0.5 is important, the plane is the midplane of the cube in Side::A direction
                Vertex::from_sides(Side::A, Side::B, Side::C).unwrap()
            ),
            0.0,
            epsilon = 1e-8,
        );
    }

    #[test]
    fn check_voxel_elevation_consistency() {
        // A cube corner should have the same elevation seen from different cubes
        assert_abs_diff_eq!(
            Surface::at_root().voxel_elevation(
                na::Vector3::new(0.0, 0.0, 0.0),
                Vertex::from_sides(Side::A, Side::B, Side::C).unwrap()
            ),
            Surface::at_root().voxel_elevation(
                na::Vector3::new(0.0, 0.0, 0.0),
                Vertex::from_sides(Side::F, Side::H, Side::J).unwrap()
            ),
            epsilon = 1e-8,
        );

        // The same corner should have the same elevation when represented from the same cube at different corners
        assert_abs_diff_eq!(
            Surface::at_root().voxel_elevation(
                na::Vector3::new(1.0, 0.0, 0.0),
                Vertex::from_sides(Side::A, Side::B, Side::C).unwrap()
            ),
            Surface::at_root().reflect(Side::A).voxel_elevation(
                na::Vector3::new(0.0, 0.0, 0.0),
                Vertex::from_sides(Side::A, Side::B, Side::C).unwrap()
            ),
            epsilon = 1e-8,
        );

        // Corners of midplane cubes separated by the midplane should have the same elevation with a different sign
        assert_abs_diff_eq!(
            Surface::at_root().voxel_elevation(
                na::Vector3::new(0.0, 0.0, 0.0),
                Vertex::from_sides(Side::A, Side::B, Side::C).unwrap()
            ),
            -Surface::at_root().voxel_elevation(
                na::Vector3::new(1.0, 0.0, 0.0),
                Vertex::from_sides(Side::A, Side::B, Side::C).unwrap()
            ),
            epsilon = 1e-8,
        );

        // Corners of midplane cubes not separated by the midplane should have the same elevation
        assert_abs_diff_eq!(
            Surface::at_root().voxel_elevation(
                na::Vector3::new(0.0, 0.0, 0.0),
                Vertex::from_sides(Side::A, Side::B, Side::C).unwrap()
            ),
            Surface::at_root().voxel_elevation(
                na::Vector3::new(0.0, 0.0, 1.0),
                Vertex::from_sides(Side::A, Side::B, Side::C).unwrap()
            ),
            epsilon = 1e-8,
        );
    }
}
