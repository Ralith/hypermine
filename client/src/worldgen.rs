use rand::{distributions::Uniform, Rng, SeedableRng};

use crate::sim::{DualGraph, VoxelData};
use common::{
    dodeca::{Side, Vertex},
    graph::NodeId,
    math::{lorentz_normalize, mip, origin},
    world::Material,
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
                max_elevation: -2,
                temperature: 0,
                rainfall: 0,
                slopeiness: 3,
                blockiness: 0,
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

pub fn voxels(graph: &DualGraph, node: NodeId, chunk: Vertex, dimension: u8) -> VoxelData {
    let enviros = chunk_incident_enviro_factors(graph, node, chunk).unwrap();

    let mut voxels = VoxelData::Solid(Material::Void);

    let state = &graph
        .get(node)
        .as_ref()
        .expect("must only be called on populated nodes")
        .state;

    // Determine whether this chunk might contain a boundary between solid and void
    let mut me_min = enviros.max_elevations[0];
    let mut me_max = enviros.max_elevations[0];
    for &me in &enviros.max_elevations[1..] {
        me_min = me_min.min(me);
        me_max = me_max.max(me);
    }
    // Maximum difference between elevations at the center of a chunk and any other point in the chunk
    // TODO: Compute what this actually is, current value is a guess! Real one must be > 0.6
    // empirically.
    const ELEVATION_MARGIN: f64 = 0.7;
    let center_elevation = state.surface.elevation(na::Vector3::repeat(0.5), chunk);
    let threshold = 10.0;
    if center_elevation - ELEVATION_MARGIN > me_max / threshold {
        // The whole chunk is underground
        // TODO: More accurate VoxelData
        return VoxelData::Solid(Material::Stone);
    }

    if center_elevation + ELEVATION_MARGIN < me_min / threshold {
        // The whole chunk is above ground
        return VoxelData::Solid(Material::Void);
    }

    for z in 0..dimension {
        for y in 0..dimension {
            for x in 0..dimension {
                let coords = na::Vector3::new(x, y, z);
                let center = voxel_center(dimension, coords);
                let cube_coords = center * 0.5;

                let rain = triserp(&enviros.rainfalls, cube_coords, 0.2, 0.8);
                let temp = triserp(&enviros.temperatures, cube_coords, 0.2, 0.8);
                let slope = triserp(&enviros.slopeinesses, cube_coords, 0.2, 0.8);

                let block = triserp(&enviros.blockinesses, cube_coords, 0.2, 0.8);
                // map real number block to interval (0, 0.25), biased towards 0
                let triserp_threshold = 2.0f64.powf(block)/(8.0+2.0f64.powf(block)) * 0.25;
                let elev = triserp(&enviros.max_elevations, cube_coords, triserp_threshold, 1.0 - triserp_threshold);

                let mut voxel_mat;
                let max_e;

                if temp < -2.0 {
                    if rain < -2.0 {
                        voxel_mat = Material::Gravelstone;
                    } else if rain < 2.0 {
                        voxel_mat = Material::Stone;
                    } else {
                        voxel_mat = Material::Greystone;
                    }
                } else if temp < 2.0 {
                    if rain < -2.0 {
                        voxel_mat = Material::Graveldirt;
                    } else if rain < 2.0 {
                        voxel_mat = Material::Dirt;
                    } else {
                        voxel_mat = Material::Grass;
                    }
                } else if rain < -2.0 {
                    voxel_mat = Material::Redsand;
                } else if rain < 2.0 {
                    voxel_mat = Material::Sand;
                } else {
                    voxel_mat = Material::Flowergrass;
                }

                //peaks should roughly tend to be snow-covered
                let slope_mod = (slope + 0.5_f64).rem_euclid(7_f64);
                if slope_mod <= 1_f64 {
                    if temp < 0.0 {
                        voxel_mat = Material::Snow;
                        max_e = elev + 0.25;
                    } else {
                        max_e = elev;
                    }
                } else if (slope_mod >= 3_f64) && (slope_mod <= 4_f64) {
                    voxel_mat = match voxel_mat {
                        Material::Flowergrass => Material::Bigflowergrass,
                        Material::Greystone => Material::Blackstone,
                        _ => voxel_mat,
                    };
                    max_e = elev - 0.25;
                } else {
                    max_e = elev;
                }

                if state.surface.elevation(center, chunk) < max_e / threshold {
                    voxels.data_mut(dimension)[index(dimension, coords)] = voxel_mat;
                }
            }
        }
    }

    // Planting trees on dirt. Trees consist of a block of wood and a block of leaves.
    // The leaf block is on the opposite face of the wood block as the dirt block.
    let loc = na::Vector3::repeat(4);
    let voxel_of_interest_index = index(dimension, loc);
    let neighbor_data = voxel_neighbors(dimension, loc, &mut voxels);

    let num_void_neighbors = neighbor_data
        .iter()
        .filter(|n| n.material == Material::Void)
        .count();

    // Only plant a tree if there is exactly one adjacent dirt block.
    if num_void_neighbors == 5 {
        for i in neighbor_data.iter() {
            if (i.material == Material::Dirt)
                || (i.material == Material::Grass)
                || (i.material == Material::Flowergrass)
            {
                voxels.data_mut(dimension)[voxel_of_interest_index] = Material::Wood;
                let leaf_location = index(dimension, i.coords_opposing);
                voxels.data_mut(dimension)[leaf_location] = Material::Leaves;
            }
        }
    }

    voxels
}

pub struct NeighborData {
    coords_opposing: na::Vector3<u8>,
    material: Material,
}

fn voxel_neighbors(dim: u8, coords: na::Vector3<u8>, voxels: &mut VoxelData) -> [NeighborData; 6] {
    [
        neighbor(dim, coords, -1, 0, 0, voxels),
        neighbor(dim, coords, 1, 0, 0, voxels),
        neighbor(dim, coords, 0, -1, 0, voxels),
        neighbor(dim, coords, 0, 1, 0, voxels),
        neighbor(dim, coords, 0, 0, -1, voxels),
        neighbor(dim, coords, 0, 0, 1, voxels),
    ]
}

pub fn neighbor(
    dimension: u8,
    w: na::Vector3<u8>,
    x: i8,
    y: i8,
    z: i8,
    voxels: &mut VoxelData,
) -> NeighborData {
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
    let material = voxels.get(index(dimension, coords));

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
    slopeiness: i64,
    blockiness: i64,
}
impl EnviroFactors {
    fn varied_from(parent: Self, spice: u64) -> Self {
        let mut rng = rand_pcg::Pcg64Mcg::seed_from_u64(spice);
        let plus_or_minus_one = Uniform::new_inclusive(-1, 1);
        let slopeiness = parent.slopeiness + rng.sample(&plus_or_minus_one);
        Self {
            slopeiness,
            max_elevation: parent.max_elevation
                + ((3 - parent.slopeiness.rem_euclid(7)) + (3 - slopeiness.rem_euclid(7)))
                + rng.sample(&plus_or_minus_one),
            temperature: parent.temperature + rng.sample(&plus_or_minus_one),
            rainfall: parent.rainfall + rng.sample(&plus_or_minus_one),
            blockiness: parent.blockiness + rng.sample(&plus_or_minus_one),
        }
    }
    fn continue_from(a: Self, b: Self, ab: Self) -> Self {
        Self {
            max_elevation: a.max_elevation + (b.max_elevation - ab.max_elevation),
            temperature: a.temperature + (b.temperature - ab.temperature),
            rainfall: a.rainfall + (b.rainfall - ab.rainfall),
            slopeiness: a.slopeiness + (b.slopeiness - ab.slopeiness),
            blockiness: a.blockiness + (b.blockiness - ab.blockiness),
        }
    }
}
impl Into<(f64, f64, f64, f64, f64)> for EnviroFactors {
    fn into(self) -> (f64, f64, f64, f64, f64) {
        (
            self.max_elevation as f64,
            self.temperature as f64,
            self.rainfall as f64,
            self.slopeiness as f64,
            self.blockiness as f64,
        )
    }
}
struct ChunkIncidentEnviroFactors {
    max_elevations: [f64; 8],
    temperatures: [f64; 8],
    rainfalls: [f64; 8],
    slopeinesses: [f64; 8],
    blockinesses: [f64; 8],
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
    let (e1, t1, r1, h1, b1) = i.next()?.into();
    let (e2, t2, r2, h2, b2) = i.next()?.into();
    let (e3, t3, r3, h3, b3) = i.next()?.into();
    let (e4, t4, r4, h4, b4) = i.next()?.into();
    let (e5, t5, r5, h5, b5) = i.next()?.into();
    let (e6, t6, r6, h6, b6) = i.next()?.into();
    let (e7, t7, r7, h7, b7) = i.next()?.into();
    let (e8, t8, r8, h8, b8) = i.next()?.into();

    Some(ChunkIncidentEnviroFactors {
        max_elevations: [e1, e2, e3, e4, e5, e6, e7, e8],
        temperatures: [t1, t2, t3, t4, t5, t6, t7, t8],
        rainfalls: [r1, r2, r3, r4, r5, r6, r7, r8],
        slopeinesses: [h1, h2, h3, h4, h5, h6, h7, h8],
        blockinesses: [b1, b2, b3, b4, b5, b6, b7, b8],
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

    /// Distance from a point in a chunk to the surface
    fn elevation(&self, chunk_coord: na::Vector3<f64>, chunk: Vertex) -> f64 {
        let pos = lorentz_normalize(&(chunk.chunk_to_node() * chunk_coord.push(1.0)));
        mip(&self.normal, &pos).asinh()
    }
}

fn triserp<N: na::RealField>(
    &[v000, v001, v010, v011, v100, v101, v110, v111]: &[N; 8],
    t: na::Vector3<N>,
    threshold1: N,
    threshold2: N,
) -> N {
    fn serp<N: na::RealField>(v0: N, v1: N, t: N, threshold1: N, threshold2: N) -> N {
        let out: N;
        if t < threshold1 {
            out = v0;
        } else if t < threshold2 {
            let s = (t - threshold1) / (threshold2 - threshold1);
            // S-shaped curve that in practice is barely different to a straight line
            out = na::convert::<_, N>(2.0) * (v0 - v1) * s * s * s
                + na::convert::<_, N>(3.0) * (v1 - v0) * s * s
                + v0;
        } else {
            out = v1;
        }
        out
    }
    fn biserp<N: na::RealField>(
        v00: N,
        v01: N,
        v10: N,
        v11: N,
        t: na::Vector2<N>,
        threshold1: N,
        threshold2: N,
    ) -> N {
        serp(
            serp(v00, v01, t.x, threshold1, threshold2),
            serp(v10, v11, t.x, threshold1, threshold2),
            t.y,
            threshold1,
            threshold2,
        )
    }

    serp(
        biserp(v000, v100, v010, v110, t.xy(), threshold1, threshold2),
        biserp(v001, v101, v011, v111, t.xy(), threshold1, threshold2),
        t.z,
        threshold1,
        threshold2,
    )
}

/// Location of the center of a voxel in a unit chunk
fn voxel_center(dimension: u8, voxel: na::Vector3<u8>) -> na::Vector3<f64> {
    voxel.map(|x| f64::from(x) + 0.5) / f64::from(dimension)
}

fn index(dimension: u8, v: na::Vector3<u8>) -> usize {
    let v = v.map(|x| usize::from(x) + 1);

    // LWM = Length (of cube sides) With Margins
    let lwm = usize::from(dimension) + 2;
    v.x + v.y * lwm + v.z * lwm.pow(2)
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

    const CHUNK_SIZE: u8 = 12;

    #[test]
    fn check_surface_flipped() {
        let root = Surface::at_root();
        assert_abs_diff_eq!(
            root.elevation(na::Vector3::x() * 2.0, Vertex::A),
            root.elevation(na::Vector3::x() * 2.0, Vertex::J) * -1.0,
            epsilon = 1e-5
        );
    }

    #[test]
    fn chunk_indexing_origin() {
        // (0, 0, 0) in localized coords
        let origin_index = 1 + (usize::from(CHUNK_SIZE) + 2) + (usize::from(CHUNK_SIZE) + 2).pow(2);

        // simple sanity check
        assert_eq!(index(CHUNK_SIZE, na::Vector3::repeat(0)), origin_index);
    }

    #[test]
    fn chunk_indexing_absolute() {
        let origin_index = 1 + (usize::from(CHUNK_SIZE) + 2) + (usize::from(CHUNK_SIZE) + 2).pow(2);
        // (0.5, 0.5, 0.5) in localized coords
        let center_index = index(CHUNK_SIZE, na::Vector3::repeat(CHUNK_SIZE / 2));
        // the point farthest from the origin, (1, 1, 1) in localized coords
        let anti_index = index(CHUNK_SIZE, na::Vector3::repeat(CHUNK_SIZE));

        assert_eq!(index(CHUNK_SIZE, na::Vector3::new(0, 0, 0)), origin_index);

        // biggest possible index in subchunk closest to origin still isn't the center
        assert!(
            index(
                CHUNK_SIZE,
                na::Vector3::new(CHUNK_SIZE / 2 - 1, CHUNK_SIZE / 2 - 1, CHUNK_SIZE / 2 - 1,)
            ) < center_index
        );
        // but the first chunk in the subchunk across from that is
        assert_eq!(
            index(
                CHUNK_SIZE,
                na::Vector3::new(CHUNK_SIZE / 2, CHUNK_SIZE / 2, CHUNK_SIZE / 2)
            ),
            center_index
        );

        // biggest possible index in subchunk closest to anti_origin is still not quite
        // the anti_origin
        assert!(
            index(
                CHUNK_SIZE,
                na::Vector3::new(CHUNK_SIZE - 1, CHUNK_SIZE - 1, CHUNK_SIZE - 1,)
            ) < anti_index
        );

        // one is added in the chunk indexing so this works out fine, the
        // domain is still CHUNK_SIZE because 0 is included.
        assert_eq!(
            index(
                CHUNK_SIZE,
                na::Vector3::new(CHUNK_SIZE - 1, CHUNK_SIZE - 1, CHUNK_SIZE - 1,)
            ),
            index(CHUNK_SIZE, na::Vector3::repeat(CHUNK_SIZE - 1))
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
        let center = na::Vector3::repeat(CHUNK_SIZE / 2);
        'top: for z in 0..CHUNK_SIZE {
            for y in 0..CHUNK_SIZE {
                for x in 0..CHUNK_SIZE {
                    let a = na::Vector3::new(x, y, z);
                    if a == center {
                        checked_center = true;
                        let c = center.map(|x| x as f64) / CHUNK_SIZE as f64;
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
            Surface::at_root().elevation(
                na::Vector3::new(1.0, 0.3, 0.9), // The first 1.0 is important, the plane is the midplane of the cube in Side::A direction
                Vertex::from_sides(Side::A, Side::B, Side::C).unwrap()
            ),
            0.0,
            epsilon = 1e-8,
        );
    }

    #[test]
    fn check_elevation_consistency() {
        // A cube corner should have the same elevation seen from different cubes
        assert_abs_diff_eq!(
            Surface::at_root().elevation(
                na::Vector3::new(0.0, 0.0, 0.0),
                Vertex::from_sides(Side::A, Side::B, Side::C).unwrap()
            ),
            Surface::at_root().elevation(
                na::Vector3::new(0.0, 0.0, 0.0),
                Vertex::from_sides(Side::F, Side::H, Side::J).unwrap()
            ),
            epsilon = 1e-8,
        );

        // The same corner should have the same elevation when represented from the same cube at different corners
        assert_abs_diff_eq!(
            Surface::at_root().elevation(
                na::Vector3::new(1.0, 0.0, 0.0),
                Vertex::from_sides(Side::A, Side::B, Side::C).unwrap()
            ),
            Surface::at_root().reflect(Side::A).elevation(
                na::Vector3::new(1.0, 0.0, 0.0),
                Vertex::from_sides(Side::A, Side::B, Side::C).unwrap()
            ),
            epsilon = 1e-8,
        );

        // Corners of midplane cubes separated by the midplane should have the same elevation with a different sign
        assert_abs_diff_eq!(
            Surface::at_root().elevation(
                na::Vector3::new(0.0, 0.0, 0.0),
                Vertex::from_sides(Side::A, Side::B, Side::C).unwrap()
            ),
            -Surface::at_root().elevation(
                na::Vector3::new(2.0, 0.0, 0.0),
                Vertex::from_sides(Side::A, Side::B, Side::C).unwrap()
            ),
            epsilon = 1e-8,
        );

        // Corners of midplane cubes not separated by the midplane should have the same elevation
        assert_abs_diff_eq!(
            Surface::at_root().elevation(
                na::Vector3::new(0.0, 0.0, 0.0),
                Vertex::from_sides(Side::A, Side::B, Side::C).unwrap()
            ),
            Surface::at_root().elevation(
                na::Vector3::new(0.0, 0.0, 2.0),
                Vertex::from_sides(Side::A, Side::B, Side::C).unwrap()
            ),
            epsilon = 1e-8,
        );
    }
}
