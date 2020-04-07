use crate::sim::{DualGraph, VoxelData};
use common::{
    dodeca::{Side, Vertex},
    graph::NodeId,
    math::{lorentz_normalize, mip, origin},
    world::{Material, SUBDIVISION_FACTOR},
};

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum NodeStateKind {
    RootSky,
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
}

pub struct NodeState {
    kind: NodeStateKind,
    surface: Surface,
    spice: u64,
    enviro: EnviroFactors,
}
impl NodeState {
    pub fn root() -> Self {
        Self {
            kind: NodeStateKind::ROOT,
            surface: Surface::at_root(),
            spice: 0,
            enviro: EnviroFactors {
                max_elevation: 0,
                temperature: 3,
                rainfall: 0,
            },
        }
    }

    pub fn child(&self, graph: &DualGraph, node: NodeId, side: Side) -> Self {
        let node_length = graph.length(node);

        let spice = graph
            .descenders(node)
            // you have to factor in the side representations so that nodes
            // with length 1 still get interesting hashes
            .map(|(s, n)| hash(graph.get(n).as_ref().unwrap().state.spice, s as u64))
            // now we mix together all the local hashes of the neighbors for this new node's
            // unique hash.
            .fold(hash(0, node_length as u64), hash);

        let mut d = graph
            .descenders(node)
            .map(|(s, n)| (s, &graph.get(n).as_ref().unwrap().state));
        let enviro = match (d.next(), d.next()) {
            (Some(_), None) => {
                let parent_side = graph.parent(node).unwrap();
                let parent_node = graph.neighbor(node, parent_side).unwrap();
                let parent_state = &graph.get(parent_node).as_ref().unwrap().state;
                EnviroFactors::varied_from(parent_state.enviro, spice)
            }
            (Some((a_side, a_state)), Some((b_side, b_state))) => {
                let ab_node = graph
                    .neighbor(graph.neighbor(node, a_side).unwrap(), b_side)
                    .unwrap();
                let ab_state = &graph.get(ab_node).as_ref().unwrap().state;
                EnviroFactors::continue_from(a_state.enviro, b_state.enviro, ab_state.enviro)
            }
            _ => unreachable!(),
        };

        let child_kind = self.kind.clone().child(side);

        Self {
            kind: child_kind,
            surface: match child_kind {
                Land => Surface::at_root(),
                Sky => Surface::opposite_root(),
                _ => self.surface.reflect(side),
            },
            spice,
            enviro,
        }
    }
}

pub fn voxels(graph: &DualGraph, node: NodeId, chunk: Vertex) -> VoxelData {
    let enviros = chunk_incident_enviro_factors(graph, node, chunk).unwrap();

    let mut voxels = VoxelData::Solid(Material::Void);

    let state = &graph
        .get(node)
        .as_ref()
        .expect("must only be called on populated nodes")
        .state;
    for z in 0..SUBDIVISION_FACTOR {
        for y in 0..SUBDIVISION_FACTOR {
            for x in 0..SUBDIVISION_FACTOR {
                let coords = na::Vector3::new(x, y, z);
                let center = voxel_center(coords);
                let cube_coords = center * 0.5;

                let elev = trilerp(&enviros.max_elevations, cube_coords);
                let rain = trilerp(&enviros.rainfalls, cube_coords);
                let temp = trilerp(&enviros.temperatures, cube_coords);

                let mut voxel_mat = Material::Dirt;
                let max_e = elev;

                if rain > 2.5 {
                    voxel_mat = Material::Leaves;
                }
                if rain < -0.5 {
                    voxel_mat = Material::Stone;
                }

                //peaks should roughly tend to be snow-covered, and valleys should roughly be watery.
                let temp_mod = (temp + 0.5_f64).rem_euclid(7_f64);
                if temp_mod <= 1.0 {
                    voxel_mat = Material::Snow;
                }

                if temp_mod >= 3.0 && temp_mod <= 4.0 {
                    voxel_mat = Material::Water;
                }

                if temp_mod < 0_f64 {
                    //should not happen.
                    voxel_mat = Material::Wood;
                }

                if state.surface.voxel_elevation(center, chunk) < max_e / -10.0 {
                    voxels.data_mut()[index(coords)] = voxel_mat;
                }
            }
        }
    }

    // Planting trees on dirt. Trees consist of a block of wood and a block of leaves.
    // The leaf block is on the opposite face of the wood block as the dirt block.
    let loc = na::Vector3::repeat(2);
    let voxel_of_interest_index = index(loc);
    let neighbor_data = voxel_neighbors(loc, &mut voxels);

    let num_void_neighbors = neighbor_data
        .iter()
        .filter(|n| n.material == Material::Void)
        .count();

    // Only plant a tree if there is exactly one adjacent dirt block.
    if num_void_neighbors == 5 {
        for i in neighbor_data.iter() {
            if i.material == Material::Dirt {
                voxels.data_mut()[voxel_of_interest_index] = Material::Wood;
                let leaf_location = index(i.coords_opposing);
                voxels.data_mut()[leaf_location] = Material::Leaves;
            }
        }
    }

    voxels
}

pub struct NeighborData {
    coords_opposing: na::Vector3<u8>,
    material: Material,
}

fn voxel_neighbors(coords: na::Vector3<u8>, voxels: &mut VoxelData) -> [NeighborData; 6] {
    [
        neighbor(coords, -1, 0, 0, voxels),
        neighbor(coords, 1, 0, 0, voxels),
        neighbor(coords, 0, -1, 0, voxels),
        neighbor(coords, 0, 1, 0, voxels),
        neighbor(coords, 0, 0, -1, voxels),
        neighbor(coords, 0, 0, 1, voxels),
    ]
}

pub fn neighbor(w: na::Vector3<u8>, x: i8, y: i8, z: i8, voxels: &mut VoxelData) -> NeighborData {
    let coords = na::Vector3::new(
        (w.x as i8 + x) as u8,
        (w.y as i8 + y) as u8,
        (w.z as i8 + z) as u8,
    );
    let coords_opposing = na::Vector3::new(
        (w.x as i8 - x) as u8,
        (w.y as i8 - y) as u8,
        (w.z as i8 - z) as u8,
    );
    let material = voxels.data()[index(coords)];

    NeighborData {
        coords_opposing,
        material,
    }
}

#[derive(Copy, Clone)]
struct EnviroFactors {
    max_elevation: i64,
    temperature: i64,
    rainfall: i64,
}
impl EnviroFactors {
    fn varied_from(parent: Self, spice: u64) -> Self {
        Self {
            max_elevation: parent.max_elevation
                + (1 - ((spice % 30) / 10) as i64)
                + (3 - parent.temperature.rem_euclid(7)),
            temperature: parent.temperature + (1 - ((spice % 78) / 26) as i64),
            rainfall: parent.rainfall + (1 - ((spice % 90) / 30) as i64),
        }
    }
    fn continue_from(a: Self, b: Self, ab: Self) -> Self {
        Self {
            max_elevation: a.max_elevation + (b.max_elevation - ab.max_elevation),
            temperature: a.temperature + (b.temperature - ab.temperature),
            rainfall: a.rainfall + (b.rainfall - ab.rainfall),
        }
    }
}
impl Into<(f64, f64, f64)> for EnviroFactors {
    fn into(self) -> (f64, f64, f64) {
        (
            self.max_elevation as f64,
            self.temperature as f64,
            self.rainfall as f64,
        )
    }
}
struct ChunkIncidentEnviroFactors {
    max_elevations: [f64; 8],
    #[allow(dead_code)]
    temperatures: [f64; 8],
    rainfalls: [f64; 8],
}

/// Returns the max_elevation values for the nodes that are incident to this chunk,
/// sorted and converted to f64 for use in functions like trilerp.
fn chunk_incident_enviro_factors(
    graph: &DualGraph,
    node: NodeId,
    cube: Vertex,
) -> Option<ChunkIncidentEnviroFactors> {
    let mut i = cube
        .dual_vertices()
        .map(|(_, path)| {
            path.fold(node, |node, side| {
                graph
                    .neighbor(node, side)
                    .expect("must only be called on chunks surrounded by populated nodes")
            })
        })
        .map(|n| {
            graph
                .get(n)
                .as_ref()
                .expect("must only be called on chunks surrounded by populated nodes")
                .state
                .enviro
        });

    // this is a bit cursed, but I don't want to collect into a vec because perf,
    // and I can't just return an iterator because then something still references graph.
    let (e1, t1, r1) = i.next()?.into();
    let (e2, t2, r2) = i.next()?.into();
    let (e3, t3, r3) = i.next()?.into();
    let (e4, t4, r4) = i.next()?.into();
    let (e5, t5, r5) = i.next()?.into();
    let (e6, t6, r6) = i.next()?.into();
    let (e7, t7, r7) = i.next()?.into();
    let (e8, t8, r8) = i.next()?.into();

    Some(ChunkIncidentEnviroFactors {
        max_elevations: [e1, e2, e3, e4, e5, e6, e7, e8],
        temperatures: [t1, t2, t3, t4, t5, t6, t7, t8],
        rainfalls: [r1, r2, r3, r4, r5, r6, r7, r8],
    })
}

/// Keeps track of the canonical surface wrt. the NodeState this is stored in
pub struct Surface {
    normal: na::Vector4<f64>,
}
impl Surface {
    /// A Vector pointing up from the surface at the root node.
    fn at_root() -> Self {
        let n = Side::A.reflection().column(3).map(|x| x as f64) - origin();
        Self {
            normal: lorentz_normalize(&n),
        }
    }

    /// A vector pointing up from the surface from a node opposite the root node
    fn opposite_root() -> Self {
        let n = origin() - Side::A.reflection().column(3).map(|x| x as f64);
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
        let pos = lorentz_normalize(&(cube.chunk_to_node() * voxel.push(1.0)));
        mip(&pos, &self.normal).asinh()
    }
}

fn trilerp<N: na::RealField>(
    &[v000, v001, v010, v011, v100, v101, v110, v111]: &[N; 8],
    t: na::Vector3<N>,
) -> N {
    fn lerp<N: na::RealField>(v0: N, v1: N, t: N) -> N {
        v0 * (N::one() - t) + v1 * t
    }
    fn bilerp<N: na::RealField>(v00: N, v01: N, v10: N, v11: N, t: na::Vector2<N>) -> N {
        lerp(lerp(v00, v01, t.x), lerp(v10, v11, t.x), t.y)
    }

    lerp(
        bilerp(v000, v100, v010, v110, t.xy()),
        bilerp(v001, v101, v011, v111, t.xy()),
        t.z,
    )
}

/// Location of the center of a voxel in a unit chunk
fn voxel_center(voxel: na::Vector3<u8>) -> na::Vector3<f64> {
    voxel.map(|x| f64::from(x) + 0.5) / f64::from(SUBDIVISION_FACTOR)
}

fn index(v: na::Vector3<u8>) -> usize {
    let v = v.map(|x| usize::from(x) + 1);

    // LWM = Length (of cube sides) With Margins
    const LWM: usize = SUBDIVISION_FACTOR as usize + 2;
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
    use crate::sim::Node;
    use approx::*;
    use common::Chunks;

    #[test]
    fn check_surface_flipped() {
        let root = Surface::at_root();
        assert_abs_diff_eq!(
            root.voxel_elevation(na::Vector3::x() * 2.0, Vertex::A),
            root.voxel_elevation(na::Vector3::x() * 2.0, Vertex::J) * -1.0,
            epsilon = 1e-5
        );
    }

    #[test]
    fn chunk_indexing_origin() {
        // (0, 0, 0) in localized coords
        let origin_index = 1
            + (usize::from(SUBDIVISION_FACTOR) + 2)
            + (usize::from(SUBDIVISION_FACTOR) + 2).pow(2);

        // simple sanity check
        assert_eq!(index(na::Vector3::repeat(0)), origin_index);
    }

    #[test]
    fn chunk_indexing_absolute() {
        let origin_index = 1
            + (usize::from(SUBDIVISION_FACTOR) + 2)
            + (usize::from(SUBDIVISION_FACTOR) + 2).pow(2);
        // (0.5, 0.5, 0.5) in localized coords
        let center_index = index(na::Vector3::repeat(SUBDIVISION_FACTOR / 2));
        // the point farthest from the origin, (1, 1, 1) in localized coords
        let anti_index = index(na::Vector3::repeat(SUBDIVISION_FACTOR));

        assert_eq!(index(na::Vector3::new(0, 0, 0)), origin_index);

        // biggest possible index in subchunk closest to origin still isn't the center
        assert!(
            index(na::Vector3::new(
                SUBDIVISION_FACTOR / 2 - 1,
                SUBDIVISION_FACTOR / 2 - 1,
                SUBDIVISION_FACTOR / 2 - 1,
            )) < center_index
        );
        // but the first chunk in the subchunk across from that is
        assert_eq!(
            index(na::Vector3::new(
                SUBDIVISION_FACTOR / 2,
                SUBDIVISION_FACTOR / 2,
                SUBDIVISION_FACTOR / 2
            )),
            center_index
        );

        // biggest possible index in subchunk closest to anti_origin is still not quite
        // the anti_origin
        assert!(
            index(na::Vector3::new(
                SUBDIVISION_FACTOR - 1,
                SUBDIVISION_FACTOR - 1,
                SUBDIVISION_FACTOR - 1,
            )) < anti_index
        );

        // one is added in the chunk indexing so this works out fine, the
        // domain is still SUBDIVISION_FACTOR because 0 is included.
        assert_eq!(
            index(na::Vector3::new(
                SUBDIVISION_FACTOR - 1,
                SUBDIVISION_FACTOR - 1,
                SUBDIVISION_FACTOR - 1,
            )),
            index(na::Vector3::repeat(SUBDIVISION_FACTOR - 1))
        );
    }

    #[test]
    fn check_chunk_incident_max_elevations() {
        let mut g = DualGraph::new();
        for (i, path) in Vertex::A.dual_vertices().map(|(_, p)| p).enumerate() {
            let new_node = path.fold(NodeId::ROOT, |node, side| g.ensure_neighbor(node, side));

            // assigning state
            *g.get_mut(new_node) = Some(Node {
                state: {
                    let mut state = NodeState::root();
                    state.enviro.max_elevation = i as i64 + 1;
                    state
                },
                chunks: Chunks::default(),
            });
        }

        let enviros = chunk_incident_enviro_factors(&g, NodeId::ROOT, Vertex::A).unwrap();
        for (i, max_elevation) in enviros.max_elevations.iter().cloned().enumerate() {
            println!("{}, {}", i, max_elevation);
            assert_abs_diff_eq!(max_elevation, (i + 1) as f64, epsilon = 1e-8);
        }

        // see corresponding test for trilerp
        let center_max_elevation = trilerp(&enviros.max_elevations, na::Vector3::repeat(0.5));
        assert_abs_diff_eq!(center_max_elevation, 4.5, epsilon = 1e-8);

        let mut checked_center = false;
        let center = na::Vector3::repeat(SUBDIVISION_FACTOR / 2);
        'top: for z in 0..SUBDIVISION_FACTOR {
            for y in 0..SUBDIVISION_FACTOR {
                for x in 0..SUBDIVISION_FACTOR {
                    let a = na::Vector3::new(x, y, z);
                    if a == center {
                        checked_center = true;
                        let c = center.map(|x| x as f64) / SUBDIVISION_FACTOR as f64;
                        let center_max_elevation = trilerp(&enviros.max_elevations, c);
                        assert_abs_diff_eq!(center_max_elevation, 4.5, epsilon = 1e-8);
                        break 'top;
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
                na::Vector3::new(1.0, 0.3, 0.9), // The first 1.0 is important, the plane is the midplane of the cube in Side::A direction
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
                na::Vector3::new(1.0, 0.0, 0.0),
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
                na::Vector3::new(2.0, 0.0, 0.0),
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
                na::Vector3::new(0.0, 0.0, 2.0),
                Vertex::from_sides(Side::A, Side::B, Side::C).unwrap()
            ),
            epsilon = 1e-8,
        );
    }
}
