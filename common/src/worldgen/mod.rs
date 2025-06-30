use horosphere::{HorosphereChunk, HorosphereNode};
use plane::Plane;
use rand::{Rng, SeedableRng, distr::Uniform};
use rand_distr::Normal;
use terraingen::VoronoiInfo;

use crate::{
    dodeca::{Side, Vertex},
    graph::{Graph, NodeId},
    margins,
    math::{self, MVector},
    node::{ChunkId, VoxelData},
    world::Material,
};

mod horosphere;
mod plane;
mod terraingen;

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

use rand_pcg::Pcg64Mcg;

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

/// Contains a minimal amount of information about a node that can be deduced entirely from
/// the NodeState of its parents.
pub struct PartialNodeState {
    /// This becomes a real horosphere only if it doesn't interfere with another higher-priority horosphere.
    /// See `HorosphereNode::has_priority` for the definition of priority.
    candidate_horosphere: Option<HorosphereNode>,
}

impl PartialNodeState {
    pub fn new(graph: &Graph, node: NodeId) -> Self {
        Self {
            candidate_horosphere: HorosphereNode::new(graph, node),
        }
    }
}

/// Contains all information about a node used for world generation. Most world
/// generation logic uses this information as a starting point. The `NodeState` is deduced
/// from the `NodeState` of the node's parents, along with the `PartialNodeState` of the node
/// itself and its "peer" nodes (See `peer_traverser`).
pub struct NodeState {
    kind: NodeStateKind,
    surface: Plane,
    road_state: NodeStateRoad,
    enviro: EnviroFactors,
    horosphere: Option<HorosphereNode>,
}
impl NodeState {
    pub fn new(graph: &Graph, node: NodeId) -> Self {
        let mut parents = graph
            .parents(node)
            .map(|(s, n)| ParentInfo {
                node_id: n,
                side: s,
                node_state: graph.node_state(n),
            })
            .fuse();
        let parents = [parents.next(), parents.next(), parents.next()];

        let enviro = match (parents[0], parents[1]) {
            (None, None) => EnviroFactors {
                max_elevation: 0.0,
                temperature: 0.0,
                rainfall: 0.0,
                blockiness: 0.0,
            },
            (Some(parent), None) => {
                let spice = graph.hash_of(node) as u64;
                EnviroFactors::varied_from(parent.node_state.enviro, spice)
            }
            (Some(parent_a), Some(parent_b)) => {
                let ab_node = graph.neighbor(parent_a.node_id, parent_b.side).unwrap();
                let ab_state = &graph.node_state(ab_node);
                EnviroFactors::continue_from(
                    parent_a.node_state.enviro,
                    parent_b.node_state.enviro,
                    ab_state.enviro,
                )
            }
            _ => unreachable!(),
        };

        let kind = parents[0].map_or(NodeStateKind::ROOT, |p| p.node_state.kind.child(p.side));
        let road_state = parents[0].map_or(NodeStateRoad::ROOT, |p| {
            p.node_state.road_state.child(p.side)
        });

        let horosphere = graph
            .partial_node_state(node)
            .candidate_horosphere
            .filter(|h| h.should_generate(graph, node));

        Self {
            kind,
            surface: match kind {
                Land => Plane::from(Side::A),
                Sky => -Plane::from(Side::A),
                _ => parents[0].map(|p| p.side * p.node_state.surface).unwrap(),
            },
            road_state,
            enviro,
            horosphere,
        }
    }

    pub fn up_direction(&self) -> MVector<f32> {
        *self.surface.scaled_normal()
    }
}

#[derive(Clone, Copy)]
struct ParentInfo<'a> {
    node_id: NodeId,
    side: Side,
    node_state: &'a NodeState,
}

struct VoxelCoords {
    counter: u32,
    dimension: u8,
}

impl VoxelCoords {
    fn new(dimension: u8) -> Self {
        VoxelCoords {
            counter: 0,
            dimension,
        }
    }
}

impl Iterator for VoxelCoords {
    type Item = (u8, u8, u8);

    fn next(&mut self) -> Option<Self::Item> {
        let dim = u32::from(self.dimension);

        if self.counter == dim.pow(3) {
            return None;
        }

        let result = (
            (self.counter / dim.pow(2)) as u8,
            ((self.counter / dim) % dim) as u8,
            (self.counter % dim) as u8,
        );

        self.counter += 1;
        Some(result)
    }
}

/// Data needed to generate a chunk
pub struct ChunkParams {
    /// Number of voxels along an edge
    dimension: u8,
    /// Which vertex of the containing node this chunk lies against
    chunk: Vertex,
    /// Random quantities stored at the eight adjacent nodes, used for terrain generation
    env: ChunkIncidentEnviroFactors,
    /// Reference plane for the terrain surface
    surface: Plane,
    /// Whether this chunk contains a segment of the road
    is_road: bool,
    /// Whether this chunk contains a section of the road's supports
    is_road_support: bool,
    /// Random quantity used to seed terrain gen
    node_spice: u64,
    /// Horosphere to place in the chunk
    horosphere: Option<HorosphereChunk>,
}

impl ChunkParams {
    /// Extract data necessary to generate a chunk, generating new graph nodes if necessary
    pub fn new(graph: &mut Graph, chunk: ChunkId) -> Self {
        graph.ensure_node_state(chunk.node);
        let env = chunk_incident_enviro_factors(graph, chunk);
        let state = graph.node_state(chunk.node);
        Self {
            dimension: graph.layout().dimension(),
            chunk: chunk.vertex,
            env,
            surface: state.surface,
            is_road: state.kind == Sky
                && ((state.road_state == East) || (state.road_state == West)),
            is_road_support: ((state.kind == Land) || (state.kind == DeepLand))
                && ((state.road_state == East) || (state.road_state == West)),
            node_spice: graph.hash_of(chunk.node) as u64,
            horosphere: state
                .horosphere
                .as_ref()
                .map(|h| HorosphereChunk::new(h, chunk.vertex)),
        }
    }

    pub fn chunk(&self) -> Vertex {
        self.chunk
    }

    /// Generate voxels making up the chunk
    pub fn generate_voxels(&self) -> VoxelData {
        let mut voxels = VoxelData::Solid(Material::Void);
        let mut rng = rand_pcg::Pcg64Mcg::seed_from_u64(hash(self.node_spice, self.chunk as u64));

        self.generate_terrain(&mut voxels, &mut rng);

        if self.is_road {
            self.generate_road(&mut voxels);
        } else if self.is_road_support {
            self.generate_road_support(&mut voxels);
        }

        if let Some(horosphere) = &self.horosphere {
            horosphere.generate(&mut voxels, self.dimension);
        }

        // TODO: Don't generate detailed data for solid chunks with no neighboring voids

        self.generate_trees(&mut voxels, &mut rng);

        margins::initialize_margins(self.dimension, &mut voxels);
        voxels
    }

    /// Performs all terrain generation that can be done one voxel at a time and with
    /// only the containing chunk's surrounding nodes' envirofactors.
    fn generate_terrain(&self, voxels: &mut VoxelData, rng: &mut Pcg64Mcg) {
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
        const ELEVATION_MARGIN: f32 = 0.7;
        let center_elevation = self
            .surface
            .distance_to_chunk(self.chunk, &na::Vector3::repeat(0.5));
        if center_elevation - ELEVATION_MARGIN > me_max / TERRAIN_SMOOTHNESS {
            // The whole chunk is above ground
            *voxels = VoxelData::Solid(Material::Void);
            return;
        }
        if center_elevation + ELEVATION_MARGIN < me_min / TERRAIN_SMOOTHNESS {
            // The whole chunk is underground
            *voxels = VoxelData::Solid(Material::Dirt);
            return;
        }

        // Otherwise, the chunk might contain a solid/void boundary, so the full terrain generation
        // code should run.
        let normal = Normal::new(0.0, 0.03).unwrap();

        for (x, y, z) in VoxelCoords::new(self.dimension) {
            let coords = na::Vector3::new(x, y, z);
            let center = voxel_center(self.dimension, coords);
            let trilerp_coords = center.map(|x| (1.0 - x) * 0.5);

            let rain = trilerp(&self.env.rainfalls, trilerp_coords) + rng.sample(normal);
            let temp = trilerp(&self.env.temperatures, trilerp_coords) + rng.sample(normal);

            // elev is calculated in multiple steps. The initial value elev_pre_terracing
            // is used to calculate elev_pre_noise which is used to calculate elev.
            let elev_pre_terracing = trilerp(&self.env.max_elevations, trilerp_coords);
            let block = trilerp(&self.env.blockinesses, trilerp_coords);
            let voxel_elevation = self.surface.distance_to_chunk(self.chunk, &center);
            let strength = 0.4 / (1.0 + math::sqr(voxel_elevation));
            let terracing_small = terracing_diff(elev_pre_terracing, block, 5.0, strength, 2.0);
            let terracing_big = terracing_diff(elev_pre_terracing, block, 15.0, strength, -1.0);
            // Small and big terracing effects must not sum to more than 1,
            // otherwise the terracing fails to be (nonstrictly) monotonic
            // and the terrain gets trenches ringing around its cliffs.
            let elev_pre_noise = elev_pre_terracing + 0.6 * terracing_small + 0.4 * terracing_big;

            // initial value dist_pre_noise is the difference between the voxel's distance
            // from the guiding plane and the voxel's calculated elev value. It represents
            // how far from the terrain surface a voxel is.
            let dist_pre_noise = elev_pre_noise / TERRAIN_SMOOTHNESS - voxel_elevation;

            // adding noise allows interfaces between strata to be rough
            let elev = elev_pre_noise + TERRAIN_SMOOTHNESS * rng.sample(normal);

            // Final value of dist is calculated in this roundabout way for greater control
            // over how noise in elev affects dist.
            let dist = if dist_pre_noise > 0.0 {
                // The .max(0.0) keeps the top of the ground smooth
                // while still allowing the surface/general terrain interface to be rough
                (elev / TERRAIN_SMOOTHNESS - voxel_elevation).max(0.0)
            } else {
                // Distance not updated for updated elevation if distance was originally
                // negative. This ensures that no voxels that would have otherwise
                // been void are changed to a material---so no floating dirt blocks.
                dist_pre_noise
            };

            if dist >= 0.0 {
                let voxel_mat = VoronoiInfo::terraingen_voronoi(elev, rain, temp, dist);
                voxels.data_mut(self.dimension)[index(self.dimension, coords)] = voxel_mat;
            }
        }
    }

    /// Places a road along the guiding plane.
    fn generate_road(&self, voxels: &mut VoxelData) {
        let plane = -Plane::from(Side::B);

        for (x, y, z) in VoxelCoords::new(self.dimension) {
            let coords = na::Vector3::new(x, y, z);
            let center = voxel_center(self.dimension, coords);
            let horizontal_distance = plane.distance_to_chunk(self.chunk, &center);
            let elevation = self.surface.distance_to_chunk(self.chunk, &center);

            if horizontal_distance > 0.3 || elevation > 0.9 {
                continue;
            }

            let mut mat: Material = Material::Void;

            if elevation < 0.075 {
                if horizontal_distance < 0.15 {
                    // Inner
                    mat = Material::WhiteBrick;
                } else {
                    // Outer
                    mat = Material::GreyBrick;
                }
            }

            voxels.data_mut(self.dimension)[index(self.dimension, coords)] = mat;
        }
    }

    /// Fills the half-plane below the road with wooden supports.
    fn generate_road_support(&self, voxels: &mut VoxelData) {
        if voxels.is_solid() && voxels.get(0) != Material::Void {
            // There is guaranteed no void to fill with the road supports, so
            // nothing to do here.
            return;
        }

        let plane = -Plane::from(Side::B);

        for (x, y, z) in VoxelCoords::new(self.dimension) {
            let coords = na::Vector3::new(x, y, z);
            let center = voxel_center(self.dimension, coords);
            let horizontal_distance = plane.distance_to_chunk(self.chunk, &center);

            if horizontal_distance > 0.3 {
                continue;
            }

            let mat = if self.trussing_at(coords) {
                Material::WoodPlanks
            } else {
                Material::Void
            };

            if mat != Material::Void {
                voxels.data_mut(self.dimension)[index(self.dimension, coords)] = mat;
            }
        }
    }

    /// Make a truss-shaped template
    fn trussing_at(&self, coords: na::Vector3<u8>) -> bool {
        // Generates planar diagonals, but corner is offset
        let mut criteria_met = 0_u32;
        let x = coords[0];
        let y = coords[1];
        let z = coords[2];
        let offset = self.dimension / 3;

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

    /// Plants trees on dirt and grass. Trees consist of a block of wood
    /// and a block of leaves. The leaf block is on the opposite face of the
    /// wood block as the ground block.
    fn generate_trees(&self, voxels: &mut VoxelData, rng: &mut Pcg64Mcg) {
        if voxels.is_solid() {
            // No trees can be generated unless there's both land and air.
            return;
        }

        if self.dimension <= 4 {
            // The tree generation algorithm can crash when the chunk size is too small.
            return;
        }

        // margins are added to keep voxels outside the chunk from being read/written
        let random_position = Uniform::new(1, self.dimension - 1).unwrap();

        let rain = self.env.rainfalls[0];
        let tree_candidate_count =
            (u32::from(self.dimension - 2).pow(3) as f32 * (rain / 100.0).clamp(0.0, 0.5)) as usize;
        for _ in 0..tree_candidate_count {
            let loc = na::Vector3::from_fn(|_, _| rng.sample(random_position));
            let voxel_of_interest_index = index(self.dimension, loc);
            let neighbor_data = self.voxel_neighbors(loc, voxels);

            let num_void_neighbors = neighbor_data
                .iter()
                .filter(|n| n.material == Material::Void)
                .count();

            // Only plant a tree if there is exactly one adjacent block of dirt or grass
            if num_void_neighbors == 5 {
                for i in neighbor_data.iter() {
                    if (i.material == Material::Dirt)
                        || (i.material == Material::Grass)
                        || (i.material == Material::MudGrass)
                        || (i.material == Material::LushGrass)
                        || (i.material == Material::TanGrass)
                        || (i.material == Material::CoarseGrass)
                    {
                        voxels.data_mut(self.dimension)[voxel_of_interest_index] = Material::Wood;
                        let leaf_location = index(self.dimension, i.coords_opposing);
                        voxels.data_mut(self.dimension)[leaf_location] = Material::Leaves;
                    }
                }
            }
        }
    }

    /// Provides information on the type of material in a voxel's six neighbours
    fn voxel_neighbors(&self, coords: na::Vector3<u8>, voxels: &VoxelData) -> [NeighborData; 6] {
        [
            self.neighbor(coords, -1, 0, 0, voxels),
            self.neighbor(coords, 1, 0, 0, voxels),
            self.neighbor(coords, 0, -1, 0, voxels),
            self.neighbor(coords, 0, 1, 0, voxels),
            self.neighbor(coords, 0, 0, -1, voxels),
            self.neighbor(coords, 0, 0, 1, voxels),
        ]
    }

    fn neighbor(
        &self,
        w: na::Vector3<u8>,
        x: i8,
        y: i8,
        z: i8,
        voxels: &VoxelData,
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
        let material = voxels.get(index(self.dimension, coords));

        NeighborData {
            coords_opposing,
            material,
        }
    }
}

const TERRAIN_SMOOTHNESS: f32 = 10.0;

struct NeighborData {
    coords_opposing: na::Vector3<u8>,
    material: Material,
}

#[derive(Copy, Clone)]
struct EnviroFactors {
    max_elevation: f32,
    temperature: f32,
    rainfall: f32,
    blockiness: f32,
}
impl EnviroFactors {
    fn varied_from(parent: Self, spice: u64) -> Self {
        let mut rng = rand_pcg::Pcg64Mcg::seed_from_u64(spice);
        let unif = Uniform::new_inclusive(-1.0, 1.0).unwrap();
        let max_elevation = parent.max_elevation + rng.sample(Normal::new(0.0, 4.0).unwrap());

        Self {
            max_elevation,
            temperature: parent.temperature + rng.sample(unif),
            rainfall: parent.rainfall + rng.sample(unif),
            blockiness: parent.blockiness + rng.sample(unif),
        }
    }
    fn continue_from(a: Self, b: Self, ab: Self) -> Self {
        Self {
            max_elevation: a.max_elevation + (b.max_elevation - ab.max_elevation),
            temperature: a.temperature + (b.temperature - ab.temperature),
            rainfall: a.rainfall + (b.rainfall - ab.rainfall),
            blockiness: a.blockiness + (b.blockiness - ab.blockiness),
        }
    }
}
impl From<EnviroFactors> for (f32, f32, f32, f32) {
    fn from(envirofactors: EnviroFactors) -> Self {
        (
            envirofactors.max_elevation,
            envirofactors.temperature,
            envirofactors.rainfall,
            envirofactors.blockiness,
        )
    }
}
struct ChunkIncidentEnviroFactors {
    max_elevations: [f32; 8],
    temperatures: [f32; 8],
    rainfalls: [f32; 8],
    blockinesses: [f32; 8],
}

/// Returns the max_elevation values for the nodes that are incident to this chunk,
/// sorted and converted to f32 for use in functions like trilerp.
///
/// Returns `None` if not all incident nodes are populated.
fn chunk_incident_enviro_factors(graph: &mut Graph, chunk: ChunkId) -> ChunkIncidentEnviroFactors {
    let mut i = chunk.vertex.dual_vertices().map(|(_, path)| {
        let node = path.fold(chunk.node, |node, side| graph.ensure_neighbor(node, side));
        graph.ensure_node_state(node);
        graph.node_state(node).enviro
    });

    // this is a bit cursed, but I don't want to collect into a vec because perf,
    // and I can't just return an iterator because then something still references graph.
    let (e1, t1, r1, b1) = i.next().unwrap().into();
    let (e2, t2, r2, b2) = i.next().unwrap().into();
    let (e3, t3, r3, b3) = i.next().unwrap().into();
    let (e4, t4, r4, b4) = i.next().unwrap().into();
    let (e5, t5, r5, b5) = i.next().unwrap().into();
    let (e6, t6, r6, b6) = i.next().unwrap().into();
    let (e7, t7, r7, b7) = i.next().unwrap().into();
    let (e8, t8, r8, b8) = i.next().unwrap().into();

    ChunkIncidentEnviroFactors {
        max_elevations: [e1, e2, e3, e4, e5, e6, e7, e8],
        temperatures: [t1, t2, t3, t4, t5, t6, t7, t8],
        rainfalls: [r1, r2, r3, r4, r5, r6, r7, r8],
        blockinesses: [b1, b2, b3, b4, b5, b6, b7, b8],
    }
}

/// Linearly interpolate at interior and boundary of a cube given values at the eight corners.
fn trilerp<N: na::RealField + Copy>(
    &[v000, v001, v010, v011, v100, v101, v110, v111]: &[N; 8],
    t: na::Vector3<N>,
) -> N {
    fn lerp<N: na::RealField + Copy>(v0: N, v1: N, t: N) -> N {
        v0 * (N::one() - t) + v1 * t
    }
    fn bilerp<N: na::RealField + Copy>(v00: N, v01: N, v10: N, v11: N, t: na::Vector2<N>) -> N {
        lerp(lerp(v00, v01, t.x), lerp(v10, v11, t.x), t.y)
    }

    lerp(
        bilerp(v000, v100, v010, v110, t.xy()),
        bilerp(v001, v101, v011, v111, t.xy()),
        t.z,
    )
}

/// serp interpolates between two values v0 and v1 over the interval [0, 1] by yielding
/// v0 for [0, threshold], v1 for [1-threshold, 1], and linear interpolation in between
/// such that the overall shape is an S-shaped piecewise function.
/// threshold should be between 0 and 0.5.
fn serp<N: na::RealField + Copy>(v0: N, v1: N, t: N, threshold: N) -> N {
    if t < threshold {
        v0
    } else if t < (N::one() - threshold) {
        let s = (t - threshold) / ((N::one() - threshold) - threshold);
        v0 * (N::one() - s) + v1 * s
    } else {
        v1
    }
}

/// Intended to produce a number that is added to elev_raw.
/// block is a real number, threshold is in (0, strength) via a logistic function
/// scale controls wavelength and amplitude. It is not 1:1 to the number of blocks in a period.
/// strength represents extremity of terracing effect. Sensible values are in (0, 0.5).
/// The greater the value of limiter, the stronger the bias of threshold towards 0.
fn terracing_diff(elev_raw: f32, block: f32, scale: f32, strength: f32, limiter: f32) -> f32 {
    let threshold: f32 = strength / (1.0 + libm::powf(2.0, limiter - block));
    let elev_floor = libm::floorf(elev_raw / scale);
    let elev_rem = elev_raw / scale - elev_floor;
    scale * elev_floor + serp(0.0, scale, elev_rem, threshold) - elev_raw
}

/// Location of the center of a voxel in a unit chunk
fn voxel_center(dimension: u8, voxel: na::Vector3<u8>) -> na::Vector3<f32> {
    voxel.map(|x| f32::from(x) + 0.5) / f32::from(dimension)
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
    use approx::*;

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
        let mut g = Graph::new(1);
        for (i, path) in Vertex::A.dual_vertices().map(|(_, p)| p).enumerate() {
            let new_node = path.fold(NodeId::ROOT, |node, side| g.ensure_neighbor(node, side));

            // assigning state
            g.ensure_node_state(new_node);
            g[new_node].state.as_mut().unwrap().enviro.max_elevation = i as f32 + 1.0;
        }

        let enviros = chunk_incident_enviro_factors(&mut g, ChunkId::new(NodeId::ROOT, Vertex::A));
        for (i, max_elevation) in enviros.max_elevations.into_iter().enumerate() {
            println!("{i}, {max_elevation}");
            assert_abs_diff_eq!(max_elevation, (i + 1) as f32, epsilon = 1e-8);
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
                        let c = center.map(|x| x as f32) / CHUNK_SIZE as f32;
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

    #[test]
    fn check_voxel_iterable() {
        let dimension = 12;

        for (counter, (x, y, z)) in (VoxelCoords::new(dimension as u8)).enumerate() {
            let index = z as usize + y as usize * dimension + x as usize * dimension.pow(2);
            assert!(counter == index);
        }
    }
}
