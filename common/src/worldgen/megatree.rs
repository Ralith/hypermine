use crate::{
    dodeca::{Side, Vertex},
    graph::{Graph, NodeId},
    math::{MPoint, MVector},
    node::VoxelData,
    world::Material,
};
use rand::{RngExt, SeedableRng};
use rand_distr::Normal;
use rand_pcg::Pcg64Mcg;

use super::{EnviroFactors, NodeStateKind};
use super::{VoxelCoords, index, line::LineSegment, voxel_center};

/// Branch radius, in absolute hyperbolic distance units.
const BRANCH_RADIUS: f32 = 0.2;
/// Radius of the leaves ball at a terminal node.
const LEAVES_RADIUS: f32 = 0.5;
/// Initial growth budget for a newly seeded tree, in Sky propagation steps.
const INITIAL_REMAINING_GROWTH: f32 = 6.5;
/// Random variation in the growth budget consumed by each Sky node.
const GROWTH_COST_STANDARD_DEVIATION: f32 = 0.3;
/// Value mixed into each node's hash to seed generation. Chosen randomly.
const SEED: u64 = 13334231312061724180;

/// Megatree state for one graph node.
#[derive(Clone, Copy)]
pub struct MegatreeState {
    tree: Option<MegatreeNode>,
}

impl MegatreeState {
    pub fn new(graph: &Graph, node: NodeId, kind: NodeStateKind, enviro: EnviroFactors) -> Self {
        let tree = MegatreeNode::new(graph, node, kind, enviro);
        Self { tree }
    }
}

/// Megatree propagation information for one node.
///
/// `parent_side` identifies the incoming branch; `child_sides` records outgoing
/// propagation directions. Rendering uses both, while propagation uses children only.
#[derive(Clone, Copy)]
struct MegatreeNode {
    parent_side: Option<Side>,
    child_sides: u16,
    remaining_growth: f32,
}

impl MegatreeNode {
    fn new(
        graph: &Graph,
        node: NodeId,
        kind: NodeStateKind,
        enviro: EnviroFactors,
    ) -> Option<Self> {
        let spice = graph.hash_of(node) as u64;
        let mut rng = Pcg64Mcg::seed_from_u64(super::hash(spice, SEED));
        // `Graph::parents` returns every shallower neighbor and the shared side.
        // Parent states have already been finalized by `ensure_node_state`.
        let tree_parent = graph.parents(node).find_map(|(side, parent)| {
            graph.node_state(parent).megatree.tree.and_then(|state| {
                state
                    .has_child_side(side)
                    .then_some((side, state.remaining_growth))
            })
        });
        let Some((parent_side, parent_growth)) = tree_parent else {
            // Seed path: new trees start on Land and grow along the ground-plane axis.
            if kind != NodeStateKind::Land
                || rng.random::<f32>() >= tree_generation_probability(enviro)
            {
                return None;
            }
            return Some(Self {
                parent_side: None,
                child_sides: (1 << Side::A as usize) | (1 << Side::A.opposite() as usize),
                remaining_growth: INITIAL_REMAINING_GROWTH,
            });
        };

        // Continuation path: inherit growth and select children from the parent side.
        let remaining_growth = if matches!(kind, NodeStateKind::Sky | NodeStateKind::DeepSky) {
            parent_growth - 1.0
                + rng.sample(Normal::new(0.0, GROWTH_COST_STANDARD_DEVIATION).unwrap())
        } else {
            parent_growth
        };
        let mut result = Self {
            parent_side: Some(parent_side),
            child_sides: 0,
            remaining_growth,
        };
        if remaining_growth >= 0.0 {
            if kind != NodeStateKind::DeepLand {
                for side in Side::iter() {
                    if side != parent_side
                        && !side.adjacent_to(parent_side)
                        && rng.random::<f32>() < branch_probability(enviro)
                    {
                        result.add_child_side(side);
                    }
                }
            } else {
                result.add_child_side(parent_side.opposite());
            }
        }

        Some(result)
    }

    fn add_child_side(&mut self, side: Side) {
        self.child_sides |= 1 << side as usize;
    }

    fn has_child_side(self, side: Side) -> bool {
        self.child_sides & (1 << side as usize) != 0
    }
}

/// Megatree geometry needed to render one chunk.
pub struct MegatreeChunk {
    dimension: u8,
    chunk: Vertex,
    branches: Vec<LineSegment>,
    has_leaves: bool,
}

impl MegatreeChunk {
    pub fn new(state: MegatreeState, dimension: u8, chunk: Vertex) -> Option<Self> {
        state
            .tree
            .map(|tree| Self::from_tree(tree, dimension, chunk))
    }

    fn from_tree(megatree: MegatreeNode, dimension: u8, chunk: Vertex) -> Self {
        let branches = megatree
            .parent_side
            .into_iter()
            .chain(
                Side::iter().filter(move |&side| megatree.child_sides & (1 << side as usize) != 0),
            )
            .map(|side| {
                let center = MPoint::origin();
                let neighbor_center = side.reflection() * center;
                let edge_center = center.midpoint(&neighbor_center);
                LineSegment::new(center, edge_center)
            })
            .collect();
        Self {
            dimension,
            chunk,
            branches,
            has_leaves: megatree.child_sides == 0,
        }
    }

    pub fn generate(&self, voxels: &mut VoxelData) {
        if self.branches.is_empty() && !self.has_leaves {
            return;
        }

        for (x, y, z) in VoxelCoords::new(self.dimension) {
            let coords = na::Vector3::new(x, y, z);
            let voxel_index = index(self.dimension, coords);
            if voxels.get(voxel_index) != Material::Void {
                continue;
            }
            let center = voxel_center(self.dimension, coords);
            let point =
                MVector::from(self.chunk.chunk_to_node() * center.push(1.0)).normalized_point();

            let material = if self
                .branches
                .iter()
                .any(|branch| branch.distance_to(&point) <= BRANCH_RADIUS)
            {
                Material::Wood
            } else if self.has_leaves && point.distance(&MPoint::origin()) <= LEAVES_RADIUS {
                Material::Leaves
            } else {
                continue;
            };
            voxels.data_mut(self.dimension)[voxel_index] = material;
        }
    }
}

fn tree_generation_probability(enviro: EnviroFactors) -> f32 {
    (enviro.rainfall * 0.025).clamp(0.0, 1.0)
}

fn branch_probability(enviro: EnviroFactors) -> f32 {
    // Use a linear ramp with lower and upper temperature limits.
    const BRANCH_TEMPERATURE_MIN: f32 = -10.0;
    const BRANCH_TEMPERATURE_MAX: f32 = 10.0;
    ((enviro.temperature - BRANCH_TEMPERATURE_MIN)
        / (BRANCH_TEMPERATURE_MAX - BRANCH_TEMPERATURE_MIN))
        .clamp(0.0, 1.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::worldgen::WorldgenConfig;

    fn climate(rainfall: f32, temperature: f32) -> EnviroFactors {
        EnviroFactors {
            max_elevation: 0.0,
            rainfall,
            temperature,
            blockiness: 0.0,
        }
    }

    fn inherited_tree(kind: NodeStateKind) -> MegatreeNode {
        let mut graph = Graph::new(1);
        graph.ensure_node_state(NodeId::ROOT, &WorldgenConfig::default());
        let mut parent = MegatreeNode {
            parent_side: None,
            child_sides: 0,
            remaining_growth: 2.5,
        };
        parent.add_child_side(Side::B);
        graph[NodeId::ROOT].state.as_mut().unwrap().megatree.tree = Some(parent);
        let child = graph.ensure_neighbor(NodeId::ROOT, Side::B);
        MegatreeNode::new(&graph, child, kind, climate(0.0, 10.0)).unwrap()
    }

    #[test]
    fn sky_branches_avoid_parent_and_adjacent_faces() {
        // Sky branches choose every eligible side independently when the branch probability is 1.
        let tree = inherited_tree(NodeStateKind::DeepSky);
        let child_sides = tree.child_sides;
        assert_eq!(child_sides.count_ones(), 6);
        assert_eq!(child_sides & (1 << Side::B as usize), 0);
        assert!(
            Side::iter()
                .filter(|s| s.adjacent_to(Side::B))
                .all(|s| child_sides & (1 << s as usize) == 0)
        );
    }

    #[test]
    fn deep_land_always_continues_as_trunk() {
        // DeepLand always continues the trunk through the face opposite its incoming parent.
        let tree = inherited_tree(NodeStateKind::DeepLand);
        assert_eq!(tree.child_sides, 1 << Side::B.opposite() as usize);
    }

    #[test]
    fn land_seed_uses_side_a_as_ground_axis() {
        // A newly seeded Land tree begins along the ground-plane axis and its opposite face.
        let graph = Graph::new(1);
        let tree = MegatreeNode::new(
            &graph,
            NodeId::ROOT,
            NodeStateKind::Land,
            climate(100.0, 10.0),
        )
        .unwrap();
        assert_eq!(
            tree.child_sides,
            (1 << Side::A as usize) | (1 << Side::A.opposite() as usize)
        );
        assert_eq!(tree.parent_side, None);
    }

    #[test]
    fn rendering_fills_void_but_preserves_terrain() {
        // Megatree voxels preserve solid terrain and fill only void portions of mixed chunks.
        let tree = MegatreeNode {
            parent_side: Some(Side::B),
            child_sides: 0,
            remaining_growth: 2.5,
        };
        let chunk = MegatreeChunk::from_tree(tree, 12, Vertex::A);
        let mut reference = VoxelData::Solid(Material::Void);
        chunk.generate(&mut reference);
        let size = (12usize + 2).pow(3);
        let wood = (0..size)
            .find(|&i| reference.get(i) == Material::Wood)
            .unwrap();
        let leaves = (0..size)
            .find(|&i| reference.get(i) == Material::Leaves)
            .unwrap();

        let mut solid = VoxelData::Solid(Material::Dirt);
        chunk.generate(&mut solid);
        assert!(matches!(solid, VoxelData::Solid(Material::Dirt)));

        let mut mixed = VoxelData::Solid(Material::Void);
        mixed.data_mut(12)[wood] = Material::Dirt;
        mixed.data_mut(12)[leaves] = Material::Dirt;
        chunk.generate(&mut mixed);
        assert_eq!(mixed.get(wood), Material::Dirt);
        assert_eq!(mixed.get(leaves), Material::Dirt);
        assert!((0..size).any(|i| mixed.get(i) == Material::Wood));
        assert!((0..size).any(|i| mixed.get(i) == Material::Leaves));
    }
}
