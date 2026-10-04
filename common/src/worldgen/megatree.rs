use crate::{
    dodeca::Side,
    graph::{Graph, NodeId},
    math::MPoint,
};
use rand::{RngExt, SeedableRng};
use rand_pcg::Pcg64Mcg;

use super::{EnviroFactors, NodeStateKind, Plane, TERRAIN_SMOOTHNESS};

/// Node-center depth required to switch propagation into underground mode.
const UNDERGROUND_DEPTH_THRESHOLD: f32 = 0.75;
/// Estimated clearance above terrain required for a trunk to start branching.
const BRANCHING_HEIGHT: f32 = 0.75;
/// Tree spawn probability per precipitation unit at Land nodes.
const SPAWN_RATE: f32 = 0.025;
/// Lower and upper temperature limits for the linear branch-probability ramp.
const BRANCH_TEMPERATURE_MIN: f32 = -10.0;
const BRANCH_TEMPERATURE_MAX: f32 = 10.0;
/// Value mixed into each node's hash to seed generation. Chosen randomly.
const SEED: u64 = 13334231312061724180;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Growth {
    Trunk,
    Branching,
}

impl Growth {
    fn at_height(self, kind: NodeStateKind, height: f32) -> Self {
        if kind != NodeStateKind::DeepLand && height > BRANCHING_HEIGHT {
            Self::Branching
        } else {
            // Terrain clearance takes precedence over inherited branching.
            Self::Trunk
        }
    }
}

/// Megatree propagation information for one node.
///
/// The bit at each side indicates that this node propagates its tree state to
/// the neighbor on that side.
#[derive(Clone, Copy)]
pub(super) struct MegatreeNode {
    child_sides: u16,
    /// Outgoing faces carrying branching state; the other child faces carry trunks.
    branching_sides: u16,
    branch_sides: u16,
}

impl MegatreeNode {
    pub(super) fn new(
        graph: &Graph,
        node: NodeId,
        kind: NodeStateKind,
        ground_depth: u32,
        surface: &Plane,
        enviro: EnviroFactors,
    ) -> Option<Self> {
        let estimated_height =
            surface.distance_to(&MPoint::origin()) - enviro.max_elevation / TERRAIN_SMOOTHNESS;
        // A Land seed has no incoming face; align its trunk with the surface normal.
        let ground_side = Side::iter()
            .max_by(|a, b| {
                a.normal()
                    .mip(surface.scaled_normal())
                    .total_cmp(&b.normal().mip(surface.scaled_normal()))
            })
            .unwrap();
        Self::with_height(
            graph,
            node,
            kind,
            ground_depth,
            enviro,
            estimated_height,
            ground_side,
        )
    }

    fn with_height(
        graph: &Graph,
        node: NodeId,
        kind: NodeStateKind,
        ground_depth: u32,
        enviro: EnviroFactors,
        estimated_height: f32,
        ground_side: Side,
    ) -> Option<Self> {
        let spice = graph.hash_of(node) as u64;
        let mut rng = Pcg64Mcg::seed_from_u64(super::hash(spice, SEED));
        let parent_sides: Vec<_> = graph.parents(node).map(|(side, _)| side).collect();

        // `Graph::parents` returns every shallower neighbor and the shared side.
        // Parent states have already been finalized by `ensure_node_state`.
        let incoming_sides: Vec<_> = graph
            .parents(node)
            .filter_map(|(side, parent)| {
                graph
                    .node_state(parent)
                    .megatree
                    .filter(|state| state.propagates_through(side))
                    .map(|state| (side, state.growth_through(side)))
            })
            .collect();

        let is_seed = incoming_sides.is_empty();
        let is_sky = matches!(kind, NodeStateKind::Sky | NodeStateKind::DeepSky);
        // Any underground Sky node can independently seed trees through its
        // eligible exits, whether or not it inherited Megatree state.
        let is_underground_sky =
            estimated_height < -UNDERGROUND_DEPTH_THRESHOLD && is_sky && ground_depth > 0;
        let can_seed_trunk = is_sky && !is_underground_sky && estimated_height < 0.0;
        if is_seed && !is_underground_sky {
            // Sky seeds must start below terrain. Above-ground trunks require
            // incoming tree state, even when they have several graph parents.
            if (kind != NodeStateKind::Land && !can_seed_trunk)
                || rng.random::<f32>() >= tree_generation_probability(enviro.rainfall)
            {
                return None;
            }
        }

        let mut result = Self {
            child_sides: 0,
            branching_sides: 0,
            branch_sides: incoming_sides
                .iter()
                .fold(0, |mask, &(side, _)| mask | (1 << side as usize)),
        };
        if is_underground_sky {
            // Underground Sky nodes independently roll for each exit toward
            // the ground plane, regardless of incoming tree state.
            let probability = tree_generation_probability(enviro.rainfall);
            for side in groundward_sides(graph, node, ground_depth) {
                if rng.random::<f32>() < probability {
                    result.propagate(side, Growth::Trunk);
                }
            }
        } else if is_seed {
            if can_seed_trunk {
                let side = parent_sides[rng.random_range(0..parent_sides.len())];
                result.propagate(side.opposite(), Growth::Trunk);
            } else {
                result.propagate(ground_side, Growth::Trunk);
                result.propagate(ground_side.opposite(), Growth::Trunk);
            }
        }
        if !is_underground_sky {
            let mut trunk_parents = Vec::new();
            for (parent_side, growth) in incoming_sides {
                match growth.at_height(kind, estimated_height) {
                    Growth::Trunk => trunk_parents.push(parent_side),
                    Growth::Branching => {
                        for side in Side::iter() {
                            if side != parent_side
                                && !side.adjacent_to(parent_side)
                                && rng.random::<f32>() < branch_probability(enviro.temperature)
                            {
                                result.propagate(side, Growth::Branching);
                            }
                        }
                    }
                }
            }
            if !trunk_parents.is_empty() {
                let index = if trunk_parents.len() == 1 {
                    0
                } else {
                    rng.random_range(0..trunk_parents.len())
                };
                result.propagate(trunk_parents[index].opposite(), Growth::Trunk);
            }
        }

        // A newly seeded tree exists only if at least one exit was selected.
        if is_seed && result.child_sides == 0 {
            return None;
        }

        Some(result)
    }

    fn propagate(&mut self, side: Side, growth: Growth) {
        let bit = 1 << side as usize;
        self.child_sides |= bit;
        self.branch_sides |= bit;
        // If paths merge onto a face, preserve any established branching state.
        if growth == Growth::Branching {
            self.branching_sides |= bit;
        }
    }

    fn growth_through(self, side: Side) -> Growth {
        if self.branching_sides & (1 << side as usize) != 0 {
            Growth::Branching
        } else {
            Growth::Trunk
        }
    }

    fn propagates_through(self, side: Side) -> bool {
        self.child_sides & (1 << side as usize) != 0
    }
}

/// Descending ground-depth directions known from the already generated graph parents.
fn groundward_sides(
    graph: &Graph,
    node: NodeId,
    ground_depth: u32,
) -> impl Iterator<Item = Side> + '_ {
    graph.parents(node).filter_map(move |(side, parent)| {
        (graph.node_state(parent).ground_depth + 1 == ground_depth).then_some(side)
    })
}

fn tree_generation_probability(precipitation: f32) -> f32 {
    (precipitation * SPAWN_RATE).clamp(0.0, 1.0)
}

fn branch_probability(temperature: f32) -> f32 {
    ((temperature - BRANCH_TEMPERATURE_MIN) / (BRANCH_TEMPERATURE_MAX - BRANCH_TEMPERATURE_MIN))
        .clamp(0.0, 1.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::worldgen::WorldgenConfig;
    use crate::{
        proto::Position,
        traversal::{ensure_nearby, nearby_nodes},
    };
    use std::collections::{HashSet, VecDeque};

    fn climate(rainfall: f32, temperature: f32) -> EnviroFactors {
        EnviroFactors {
            max_elevation: 0.0,
            rainfall,
            temperature,
            blockiness: 0.0,
        }
    }

    fn shortest_ground_distance(graph: &mut Graph, node: NodeId, cfg: &WorldgenConfig) -> u32 {
        let mut pending = VecDeque::from([(node, 0)]);
        let mut visited = HashSet::from([node]);
        while let Some((current, distance)) = pending.pop_front() {
            graph.ensure_node_state(current, cfg);
            if matches!(
                graph.node_state(current).kind,
                NodeStateKind::Sky | NodeStateKind::Land
            ) {
                return distance;
            }
            for side in Side::iter() {
                let neighbor = graph.ensure_neighbor(current, side);
                if visited.insert(neighbor) {
                    pending.push_back((neighbor, distance + 1));
                }
            }
        }
        unreachable!("the ground layer is reachable")
    }

    #[test]
    fn groundward_directions_match_local_shortest_paths_without_expanding_graph() {
        let cfg = WorldgenConfig::default();
        let mut graph = Graph::new(1);
        ensure_nearby(&mut graph, &Position::origin(), 3.0);
        let sky = graph.ensure_neighbor(NodeId::ROOT, Side::A);
        let sky_position = Position {
            node: sky,
            local: crate::math::MIsometry::identity(),
        };
        ensure_nearby(&mut graph, &sky_position, 3.0);
        let nodes = nearby_nodes(&graph, &Position::origin(), 2.0)
            .into_iter()
            .chain(nearby_nodes(&graph, &sky_position, 2.0))
            .map(|(node, _)| node)
            .collect::<HashSet<_>>();
        let mut deep_sky = 0;
        let mut deep_land = 0;
        let mut multiple_groundward = 0;
        let mut multiple_deep_sky = 0;
        for node in nodes {
            graph.ensure_node_state(node, &cfg);
            let state = graph.node_state(node);
            if !matches!(state.kind, NodeStateKind::DeepSky | NodeStateKind::DeepLand) {
                assert_eq!(state.ground_depth, 0);
                continue;
            }
            deep_sky += usize::from(state.kind == NodeStateKind::DeepSky);
            deep_land += usize::from(state.kind == NodeStateKind::DeepLand);
            let depth = state.ground_depth;
            let before = graph.len();
            let actual = groundward_sides(&graph, node, depth).collect::<Vec<_>>();
            assert_eq!(graph.len(), before, "groundward query expanded the graph");
            let mut expected = Vec::new();
            for side in Side::iter() {
                let neighbor = graph.ensure_neighbor(node, side);
                graph.ensure_node_state(neighbor, &cfg);
                let neighbor_depth = graph.node_state(neighbor).ground_depth;
                assert!(
                    depth.abs_diff(neighbor_depth) <= 1,
                    "node={node:?}, side={side:?}"
                );
                if neighbor_depth + 1 == depth {
                    expected.push(side);
                }
            }
            assert_eq!(actual, expected, "node={node:?}, depth={depth}");
            assert!(!actual.is_empty());
            assert_eq!(shortest_ground_distance(&mut graph, node, &cfg), depth);
            if actual.len() > 1 {
                multiple_groundward += 1;
                if graph.node_state(node).kind == NodeStateKind::DeepSky {
                    multiple_deep_sky += 1;
                    let tree = MegatreeNode::with_height(
                        &graph,
                        node,
                        NodeStateKind::DeepSky,
                        depth,
                        climate(100.0, 0.0),
                        -1.0,
                        Side::A,
                    )
                    .unwrap();
                    let expected_mask = actual
                        .iter()
                        .fold(0, |mask, side| mask | 1 << *side as usize);
                    assert_eq!(tree.child_sides, expected_mask);
                }
            }
        }
        assert!(
            deep_sky > 0 && deep_land > 0 && multiple_groundward > 0 && multiple_deep_sky > 0,
            "deep_sky={deep_sky}, deep_land={deep_land}, multiple_groundward={multiple_groundward}, multiple_deep_sky={multiple_deep_sky}"
        );
    }

    fn inherited_tree(kind: NodeStateKind, height: f32, growth: Growth) -> MegatreeNode {
        let mut graph = Graph::new(1);
        graph.ensure_node_state(NodeId::ROOT, &WorldgenConfig::default());
        let mut parent = MegatreeNode {
            child_sides: 0,
            branching_sides: 0,
            branch_sides: 0,
        };
        parent.propagate(Side::B, growth);
        graph[NodeId::ROOT].state.as_mut().unwrap().megatree = Some(parent);
        let child = graph.ensure_neighbor(NodeId::ROOT, Side::B);
        MegatreeNode::with_height(&graph, child, kind, 1, climate(0.0, 10.0), height, Side::A)
            .unwrap()
    }

    #[test]
    fn trunk_continues_opposite_incoming_face_until_clear_of_ground() {
        for kind in [
            NodeStateKind::Land,
            NodeStateKind::Sky,
            NodeStateKind::DeepLand,
            NodeStateKind::DeepSky,
        ] {
            for height in [-0.75, -0.5, -0.3, 0.0, 0.75] {
                for growth in [Growth::Trunk, Growth::Branching] {
                    let tree = inherited_tree(kind, height, growth);
                    assert_eq!(tree.child_sides, 1 << Side::B.opposite() as usize);
                    assert_eq!(tree.branching_sides, 0);
                    assert_ne!(tree.branch_sides & (1 << Side::B as usize), 0);
                }
            }
        }
    }

    #[test]
    fn converging_trunks_choose_one_reproducible_opposite_exit() {
        for path in [&[Side::B, Side::C][..], &[Side::A, Side::B, Side::C][..]] {
            let mut graph = Graph::new(1);
            let node = path.iter().fold(NodeId::ROOT, |node, &side| {
                graph.ensure_neighbor(node, side)
            });
            graph.ensure_node_state(node, &WorldgenConfig::default());
            let parents = graph.parents(node).collect::<Vec<_>>();
            assert_eq!(parents.len(), path.len());
            for &(side, parent) in &parents {
                let mut tree = MegatreeNode {
                    child_sides: 0,
                    branching_sides: 0,
                    branch_sides: 0,
                };
                tree.propagate(side, Growth::Trunk);
                graph[parent].state.as_mut().unwrap().megatree = Some(tree);
            }
            let build = || {
                MegatreeNode::with_height(
                    &graph,
                    node,
                    NodeStateKind::DeepSky,
                    graph.node_state(node).ground_depth,
                    climate(0.0, 10.0),
                    0.0,
                    Side::A,
                )
                .unwrap()
            };
            let tree = build();
            assert_eq!(tree.child_sides.count_ones(), 1);
            assert_eq!(tree.branching_sides, 0);
            assert!(
                parents
                    .iter()
                    .any(|&(side, _)| tree.propagates_through(side.opposite()))
            );
            assert!(
                parents
                    .iter()
                    .all(|&(side, _)| tree.branch_sides & (1 << side as usize) != 0)
            );
            assert_eq!(tree.child_sides, build().child_sides);
        }
    }

    #[test]
    fn cleared_trunk_branches() {
        for growth in [Growth::Trunk, Growth::Branching] {
            let tree = inherited_tree(NodeStateKind::DeepSky, 0.751, growth);
            assert_eq!(tree.child_sides.count_ones(), 6);
            assert_eq!(tree.branching_sides, tree.child_sides);
            assert!(!tree.propagates_through(Side::B));
            assert!(
                Side::iter()
                    .filter(|s| s.adjacent_to(Side::B))
                    .all(|s| !tree.propagates_through(s))
            );
        }
    }

    #[test]
    fn deep_land_always_continues_as_trunk() {
        for height in [-1.0, 0.0, 0.751, 10.0] {
            for growth in [Growth::Trunk, Growth::Branching] {
                let tree = inherited_tree(NodeStateKind::DeepLand, height, growth);
                assert_eq!(tree.child_sides, 1 << Side::B.opposite() as usize);
                assert_eq!(tree.branching_sides, 0);
            }
        }
    }

    #[test]
    fn underground_deep_sky_seeding_requires_three_quarters_unit_of_cover() {
        let mut graph = Graph::new(1);
        graph.ensure_node_state(NodeId::ROOT, &WorldgenConfig::default());
        graph[NodeId::ROOT].state.as_mut().unwrap().megatree = None;
        let child = graph.ensure_neighbor(NodeId::ROOT, Side::B);
        let kind = NodeStateKind::DeepSky;
        for height in [-0.75, -0.5, -0.3, -0.001] {
            let tree = MegatreeNode::with_height(
                &graph,
                child,
                kind,
                1,
                climate(100.0, 10.0),
                height,
                Side::A,
            )
            .unwrap();
            assert_eq!(tree.child_sides.count_ones(), 1);
            assert_eq!(tree.branching_sides, 0);
        }
        assert!(MegatreeNode::with_height(
            &graph, child, kind, 1, climate(100.0, 10.0), 0.751, Side::A,
        ).is_none());
        let tree = MegatreeNode::with_height(
            &graph,
            child,
            kind,
            1,
            climate(100.0, 10.0),
            -0.751,
            Side::A,
        )
        .unwrap();
        assert_eq!(tree.child_sides.count_ones(), 1);
        assert_eq!(tree.branching_sides, 0);
    }

    #[test]
    fn trunk_band_can_seed_without_incoming_trees_from_multiple_graph_parents() {
        for path in [&[Side::B, Side::C][..], &[Side::A, Side::B, Side::C][..]] {
            let mut graph = Graph::new(1);
            let node = path.iter().fold(NodeId::ROOT, |node, &side| {
                graph.ensure_neighbor(node, side)
            });
            graph.ensure_node_state(node, &WorldgenConfig::default());
            let parents = graph.parents(node).collect::<Vec<_>>();
            assert_eq!(parents.len(), path.len());
            for &(_, parent) in &parents {
                graph[parent].state.as_mut().unwrap().megatree = None;
            }
            for kind in [NodeStateKind::Sky, NodeStateKind::DeepSky] {
                for height in [-0.75, -0.5, -0.001] {
                    let build = || {
                        MegatreeNode::with_height(
                            &graph,
                            node,
                            kind,
                            graph.node_state(node).ground_depth,
                            climate(100.0, 10.0),
                            height,
                            Side::A,
                        )
                        .unwrap()
                    };
                    let tree = build();
                    assert_eq!(tree.child_sides.count_ones(), 1);
                    assert_eq!(tree.branching_sides, 0);
                    assert!(
                        parents
                            .iter()
                            .any(|&(side, _)| tree.propagates_through(side.opposite()))
                    );
                    assert_eq!(tree.branch_sides, tree.child_sides);
                    assert_eq!(tree.child_sides, build().child_sides);
                }
                for height in [0.0, 0.001, 0.75, 0.751] {
                    assert!(
                        MegatreeNode::with_height(
                            &graph,
                            node,
                            kind,
                            graph.node_state(node).ground_depth,
                            climate(100.0, 10.0),
                            height,
                            Side::A,
                        )
                        .is_none()
                    );
                }
            }
        }
    }

    #[test]
    fn land_seed_uses_supplied_ground_axis() {
        let graph = Graph::new(1);
        let tree = MegatreeNode::with_height(
            &graph,
            NodeId::ROOT,
            NodeStateKind::Land,
            0,
            climate(100.0, 10.0),
            0.0,
            Side::C,
        )
        .unwrap();
        assert_eq!(
            tree.child_sides,
            (1 << Side::C as usize) | (1 << Side::C.opposite() as usize)
        );
        assert_eq!(tree.branching_sides, 0);
    }
}
