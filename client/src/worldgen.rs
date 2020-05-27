use rand::{distributions::Uniform, Rng, SeedableRng};

use crate::sim::{DualGraph, VoxelData};
use common::{
    dodeca::{Side, Vertex},
    graph::NodeId,
    world::Material,
    Plane,
};

#[derive(Clone, Copy, PartialEq, Debug)]
enum NodeStateKind {
    Sky,
    DeepSky,
    Land,
    DeepLand,
}
use NodeStateKind::*;

impl NodeStateKind {
    const ROOT: Self = Land;

    /// What state comes after this state, from a given side?
    fn child(self, side: Side) -> Self {
        match (self, side) {
            (Sky, Side::A) => Land,
            (Land, Side::A) => Sky,
            (Sky, _) if !side.adjacent_to(Side::A) => DeepSky,
            (Land, _) if !side.adjacent_to(Side::A) => DeepLand,
            _ => self,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
enum NodeStateRoad {
    East,
    DeepEast,
    West,
    DeepWest,
}
use NodeStateRoad::*;

impl NodeStateRoad {
    const ROOT: Self = West;

    /// What state comes after this state, from a given side?
    fn child(self, side: Side) -> Self {
        match (self, side) {
            (East, Side::B) => West,
            (West, Side::B) => East,
            (East, _) if !side.adjacent_to(Side::B) => DeepEast,
            (West, _) if !side.adjacent_to(Side::B) => DeepWest,
            _ => self,
        }
    }
}

pub struct NodeState {
    kind: NodeStateKind,
    surface: Plane<f64>,
    road_state: NodeStateRoad,
    spice: u64,
    enviro: EnviroFactors,
}
impl NodeState {
    pub fn root() -> Self {
        Self {
            kind: NodeStateKind::ROOT,
            surface: Plane::from(Side::A),
            road_state: NodeStateRoad::ROOT,
            spice: 0,
            enviro: EnviroFactors {
                max_elevation: -2,
                temperature: 0,
                rainfall: 0,
                slopeiness: 3,
                blockiness: 0,
                flatness: 25,
            },
        }
    }

    pub fn child(&self, graph: &DualGraph, node: NodeId, side: Side) -> Self {
        let spice = graph
            .descenders(node)
            // you have to factor in the side representations so that nodes
            // with length 1 still get interesting hashes
            .map(|(s, n)| hash(graph.get(n).as_ref().unwrap().state.spice, s as u64))
            // now we mix together all the local hashes of the neighbors for this new node's
            // unique hash.
            .fold(0, hash);

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
        let child_road = self.road_state.clone().child(side);

        Self {
            kind: child_kind,
            surface: match child_kind {
                Land => Plane::from(Side::A),
                Sky => -Plane::from(Side::A),
                _ => side * self.surface,
            },
            road_state: child_road,
            spice,
            enviro,
        }
    }
}

/// Data needed to generate a chunk
pub struct ChunkParams {
    /// Number of voxels along an edge
    dimension: u8,
    /// Which vertex of the containing node this chunk lies against
    chunk: Vertex,
    env: ChunkIncidentEnviroFactors,
    /// Reference plane for the terrain surface
    surface: Plane<f64>,
    /// Whether this chunk contains a segment of the road
    is_road: bool,
    /// Whether this chunk contains a section of the road's supports
    is_road_support: bool,
}

impl ChunkParams {
    /// Extract data necessary to generate a chunk
    ///
    /// Returns `None` if an unpopulated node is needed.
    pub fn new(dimension: u8, graph: &DualGraph, node: NodeId, chunk: Vertex) -> Option<Self> {
        let state = &graph.get(node).as_ref()?.state;
        Some(Self {
            dimension,
            chunk,
            env: chunk_incident_enviro_factors(graph, node, chunk)?,
            surface: state.surface,
            is_road: state.kind == Sky
                && ((state.road_state == East) || (state.road_state == West)),
            is_road_support: ((state.kind == Land) || (state.kind == DeepLand))
                && ((state.road_state == East) || (state.road_state == West)),
        })
    }

    pub fn chunk(&self) -> Vertex {
        self.chunk
    }

    fn generate_terrain(&self, center: na::Vector3<f64>) -> Material {
        let cube_coords = center * 0.5;

        let rain = trilerp(&self.env.rainfalls, cube_coords);
        let temp = trilerp(&self.env.temperatures, cube_coords);
        let slope = trilerp(&self.env.slopeinesses, cube_coords);
        let flat = trilerp(&self.env.flatness, cube_coords);

        // block is a real number, threshold is in (0, 0.2) and biased towards 0
        // This causes the level of terrain bumpiness to vary over space.
        let block = trilerp(&self.env.blockinesses, cube_coords);
        let threshold = 2.0f64.powf(block) / (4.0 + 2.0f64.powf(block)) * 0.2;
        let elev_raw = trilerp(&self.env.max_elevations, cube_coords);
        let terracing_scale = 5.0; // This is not wavelength in number of blocks
        let elev_floor = (elev_raw / terracing_scale).floor();
        let elev_rem = elev_raw / terracing_scale - elev_floor;
        let elev = terracing_scale * elev_floor + serp(0.0, terracing_scale, elev_rem, threshold);

        let mut voxel_mat;
        let max_e;

        // Nine basic terrain types based on combinations of
        // low/medium/high temperature and humidity.
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

        // Additional adjustments alter both block material and elevation
        // for a bit of extra variety.
        if flat <= 30.0 {
            let slope_mod = (slope + 0.5_f64).rem_euclid(7_f64);
            // peaks should roughly tend to be snow-covered
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
        } else {
            max_e = elev;
        }

        let voxel_elevation = self.surface.distance_to_chunk(self.chunk, &center);
        if voxel_elevation >= max_e / ELEVATION_SCALE {
            voxel_mat = Material::Void;
        }

        voxel_mat
    }

    fn generate_road(&self, center: na::Vector3<f64>) -> Option<Material> {
        let plane = -Plane::from(Side::B);

        let horizontal_distance = plane.distance_to_chunk(self.chunk, &center);
        let elevation = self.surface.distance_to_chunk(self.chunk, &center);
        if horizontal_distance > 0.3 {
            return None;
        }
        if elevation < 0.075 {
            // Surface
            Some(if horizontal_distance < 0.15 {
                // Inner
                Material::WhiteBrick
            } else {
                // Outer
                Material::GreyBrick
            })
        } else if elevation < 0.9 {
            Some(Material::Void) // Tunnel
        } else {
            None
        }
    }

    fn generate_road_support(
        &self,
        center: na::Vector3<f64>,
        coords: na::Vector3<u8>,
    ) -> Option<Material> {
        let plane = -Plane::from(Side::B);
        let horizontal_distance = plane.distance_to_chunk(self.chunk, &center);

        if horizontal_distance < 0.3 && self.trussing_at(coords) {
            Some(Material::WoodPlanks)
        } else {
            None
        }
    }

    /// Make a truss-shaped template
    fn trussing_at(&self, coords: na::Vector3<u8>) -> bool {
        // Generates planar diagonals, but corner is offset
        let mut criteria_met = 0_u32;
        let x = coords[0];
        let y = coords[1];
        let z = coords[2];
        let offset = 2 * self.dimension / 3;

        // straight lines.
        criteria_met += u32::from(x == offset);
        criteria_met += u32::from(y == offset);
        criteria_met += u32::from(z == offset);

        // main diagonal
        criteria_met += u32::from(x == y);
        criteria_met += u32::from(y == z);
        criteria_met += u32::from(x == z);

        criteria_met >= 2
    }

    /// Generate voxels making up the chunk
    pub fn generate_voxels(&self) -> VoxelData {
        // Determine whether this chunk might contain a boundary between solid and void
        let mut me_min = self.env.max_elevations[0];
        let mut me_max = self.env.max_elevations[0];
        for &me in &self.env.max_elevations[1..] {
            me_min = me_min.min(me);
            me_max = me_max.max(me);
        }
        // Maximum difference between elevations at the center of a chunk and any other point in the chunk
        // TODO: Compute what this actually is, current value is a guess! Real one must be > 0.6
        // empirically.
        const ELEVATION_MARGIN: f64 = 0.7;
        let center_elevation = self
            .surface
            .distance_to_chunk(self.chunk, &na::Vector3::repeat(0.5));
        if (center_elevation - ELEVATION_MARGIN > me_max / ELEVATION_SCALE)
            && !(self.is_road || self.is_road_support)
        {
            // The whole chunk is above ground and not part of the road
            return VoxelData::Solid(Material::Void);
        }

        if (center_elevation + ELEVATION_MARGIN < me_min / ELEVATION_SCALE) && !self.is_road {
            // The whole chunk is underground
            // TODO: More accurate VoxelData
            return VoxelData::Solid(Material::Stone);
        }

        let mut voxels = VoxelData::Solid(Material::Void);

        for z in 0..self.dimension {
            for y in 0..self.dimension {
                for x in 0..self.dimension {
                    let coords = na::Vector3::new(x, y, z);
                    let center = voxel_center(self.dimension, coords);

                    // road generation
                    let mat = if self.is_road {
                        self.generate_road(center)
                    } else if self.is_road_support {
                        self.generate_road_support(center, coords)
                    } else {
                        None
                    };
                    let mat = mat.unwrap_or_else(|| self.generate_terrain(center));
                    if mat != Material::Void {
                        voxels.data_mut(self.dimension)[index(self.dimension, coords)] = mat;
                    }
                }
            }
        }

        // TODO: Don't generate detailed data for solid chunks with no neighboring voids

        // Planting trees on dirt, grass, or flowers. Trees consist of a block of wood
        // and a block of leaves. The leaf block is on the opposite face of the
        // wood block as the ground block.
        if self.dimension > 2 {
            let loc = na::Vector3::repeat(self.dimension / 2);
            let voxel_of_interest_index = index(self.dimension, loc);
            let neighbor_data = voxel_neighbors(self.dimension, loc, &mut voxels);

            let num_void_neighbors = neighbor_data
                .iter()
                .filter(|n| n.material == Material::Void)
                .count();

            // Only plant a tree if there is exactly one adjacent block of dirt, grass, or flowers.
            if num_void_neighbors == 5 {
                for i in neighbor_data.iter() {
                    if (i.material == Material::Dirt)
                        || (i.material == Material::Grass)
                        || (i.material == Material::Flowergrass)
                    {
                        voxels.data_mut(self.dimension)[voxel_of_interest_index] = Material::Wood;
                        let leaf_location = index(self.dimension, i.coords_opposing);
                        voxels.data_mut(self.dimension)[leaf_location] = Material::Leaves;
                    }
                }
            }
        }

        voxels
    }
}

const ELEVATION_SCALE: f64 = 10.0;

struct NeighborData {
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

fn neighbor(
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
    flatness: i64,
}
impl EnviroFactors {
    fn varied_from(parent: Self, spice: u64) -> Self {
        let mut rng = rand_pcg::Pcg64Mcg::seed_from_u64(spice);
        let plus_or_minus_one = Uniform::new_inclusive(-1, 1);
        let flatness = (parent.flatness + rng.sample(&plus_or_minus_one))
            .max(0)
            .min(40);
        let slopeiness = parent.slopeiness + rng.sample(&plus_or_minus_one);
        Self {
            slopeiness,
            flatness,
            max_elevation: parent.max_elevation
                + ((((3 - parent.slopeiness.rem_euclid(7)) as f64)
                    * (1.0 - (((parent.flatness as f64) - 20.0) / 10.0).tanh())
                    + ((3 - slopeiness.rem_euclid(7)) as f64)
                        * (1.0 - (((flatness as f64) - 20.0) / 10.0).tanh()))
                    as i64)
                    * rng.sample(&plus_or_minus_one),
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
            flatness: a.flatness + (b.flatness - ab.flatness),
        }
    }
}
impl Into<(f64, f64, f64, f64, f64, f64)> for EnviroFactors {
    fn into(self) -> (f64, f64, f64, f64, f64, f64) {
        (
            self.max_elevation as f64,
            self.temperature as f64,
            self.rainfall as f64,
            self.slopeiness as f64,
            self.blockiness as f64,
            self.flatness as f64,
        )
    }
}
struct ChunkIncidentEnviroFactors {
    max_elevations: [f64; 8],
    temperatures: [f64; 8],
    rainfalls: [f64; 8],
    slopeinesses: [f64; 8],
    blockinesses: [f64; 8],
    flatness: [f64; 8],
}

/// Returns the max_elevation values for the nodes that are incident to this chunk,
/// sorted and converted to f64 for use in functions like trilerp.
///
/// Returns `None` if not all incident nodes are populated.
fn chunk_incident_enviro_factors(
    graph: &DualGraph,
    node: NodeId,
    cube: Vertex,
) -> Option<ChunkIncidentEnviroFactors> {
    let mut i = cube
        .dual_vertices()
        .map(|(_, mut path)| path.try_fold(node, |node, side| graph.neighbor(node, side)))
        .filter_map(|node| Some(graph.get(node?).as_ref()?.state.enviro));

    // this is a bit cursed, but I don't want to collect into a vec because perf,
    // and I can't just return an iterator because then something still references graph.
    let (e1, t1, r1, h1, b1, f1) = i.next()?.into();
    let (e2, t2, r2, h2, b2, f2) = i.next()?.into();
    let (e3, t3, r3, h3, b3, f3) = i.next()?.into();
    let (e4, t4, r4, h4, b4, f4) = i.next()?.into();
    let (e5, t5, r5, h5, b5, f5) = i.next()?.into();
    let (e6, t6, r6, h6, b6, f6) = i.next()?.into();
    let (e7, t7, r7, h7, b7, f7) = i.next()?.into();
    let (e8, t8, r8, h8, b8, f8) = i.next()?.into();

    Some(ChunkIncidentEnviroFactors {
        max_elevations: [e1, e2, e3, e4, e5, e6, e7, e8],
        temperatures: [t1, t2, t3, t4, t5, t6, t7, t8],
        rainfalls: [r1, r2, r3, r4, r5, r6, r7, r8],
        slopeinesses: [h1, h2, h3, h4, h5, h6, h7, h8],
        blockinesses: [b1, b2, b3, b4, b5, b6, b7, b8],
        flatness: [f1, f2, f3, f4, f5, f6, f7, f8],
    })
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

// serp interpolates between two values v0 and v1 over the interval [0, 1] by yielding
// v0 for [0, threshold], v1 for [1-threshold, 1], and linear interpolation in between
// such that the overall shape is an S-shaped piecewise function.
// threshold should be between 0 and 0.5.
fn serp<N: na::RealField>(v0: N, v1: N, t: N, threshold: N) -> N {
    if t < threshold {
        v0
    } else if t < (N::one() - threshold) {
        let s = (t - threshold) / ((N::one() - threshold) - threshold);
        v0 * (N::one() - s) + v1 * s
    } else {
        v1
    }
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
                na::Vector3::new(0.0, 0.0, 0.0),
            ),
            epsilon = 1e-8,
        );
        assert_abs_diff_eq!(
            1.0,
            trilerp(
                &[0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                na::Vector3::new(1.0, 0.0, 0.0),
            ),
            epsilon = 1e-8,
        );
        assert_abs_diff_eq!(
            1.0,
            trilerp(
                &[0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                na::Vector3::new(0.0, 1.0, 0.0),
            ),
            epsilon = 1e-8,
        );
        assert_abs_diff_eq!(
            1.0,
            trilerp(
                &[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                na::Vector3::new(1.0, 1.0, 0.0),
            ),
            epsilon = 1e-8,
        );
        assert_abs_diff_eq!(
            1.0,
            trilerp(
                &[0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                na::Vector3::new(0.0, 0.0, 1.0),
            ),
            epsilon = 1e-8,
        );
        assert_abs_diff_eq!(
            1.0,
            trilerp(
                &[0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                na::Vector3::new(1.0, 0.0, 1.0),
            ),
            epsilon = 1e-8,
        );
        assert_abs_diff_eq!(
            1.0,
            trilerp(
                &[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
                na::Vector3::new(0.0, 1.0, 1.0),
            ),
            epsilon = 1e-8,
        );
        assert_abs_diff_eq!(
            1.0,
            trilerp(
                &[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                na::Vector3::new(1.0, 1.0, 1.0),
            ),
            epsilon = 1e-8,
        );

        assert_abs_diff_eq!(
            0.5,
            trilerp(
                &[0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0],
                na::Vector3::new(0.5, 0.5, 0.5),
            ),
            epsilon = 1e-8,
        );
        assert_abs_diff_eq!(
            0.5,
            trilerp(
                &[0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
                na::Vector3::new(0.5, 0.5, 0.5),
            ),
            epsilon = 1e-8,
        );

        assert_abs_diff_eq!(
            4.5,
            trilerp(
                &[1.0, 5.0, 3.0, 7.0, 2.0, 6.0, 4.0, 8.0],
                na::Vector3::new(0.5, 0.5, 0.5),
            ),
            epsilon = 1e-8,
        );
    }
}
