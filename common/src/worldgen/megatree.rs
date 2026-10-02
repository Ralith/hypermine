use crate::{
    dodeca::Side,
    graph::{Graph, NodeId},
};
use rand::{RngExt, SeedableRng};
use rand_pcg::Pcg64Mcg;

use super::NodeStateKind;

/// Branch radius, in absolute hyperbolic distance units.
pub const BRANCH_RADIUS: f32 = 0.2;
/// Radius of the leaves ball at a terminal node.
pub const LEAVES_RADIUS: f32 = 0.5;
/// Node-center depth required to switch propagation into underground mode.
const UNDERGROUND_DEPTH_THRESHOLD: f32 = 1.5 * BRANCH_RADIUS;
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
    fn at_height(self, height: f32) -> Self {
        if height > BRANCHING_HEIGHT {
            Self::Branching
        } else {
            self
        }
    }
}

/// Megatree propagation information for one node.
///
/// The bit at each side indicates that this node propagates its tree state to
/// the neighbor on that side.
#[derive(Clone, Copy)]
pub struct MegatreeNode {
    child_sides: u16,
    /// Outgoing faces carrying branching state; the other child faces carry trunks.
    branching_sides: u16,
    branch_sides: u16,
}

impl MegatreeNode {
    pub fn new(
        graph: &Graph,
        node: NodeId,
        kind: NodeStateKind,
        precipitation: f32,
        temperature: f32,
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
        let is_underground_sky = estimated_height < -UNDERGROUND_DEPTH_THRESHOLD && is_sky;
        if is_seed && !is_underground_sky {
            // Only Land nodes may own a tree. Each node's deterministic roll
            // scales linearly with precipitation.
            if kind != NodeStateKind::Land
                || rng.random::<f32>() >= tree_generation_probability(precipitation)
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
            // Underground nodes may descend only through sides that are neither
            // equal nor adjacent to any graph parent face. Underground Sky
            // nodes make this decision independently of incoming tree state.
            let probability = tree_generation_probability(precipitation);
            for side in Side::iter() {
                if parent_sides
                    .iter()
                    .all(|&parent_side| side != parent_side && !side.adjacent_to(parent_side))
                    && rng.random::<f32>() < probability
                {
                    result.propagate(side, Growth::Trunk);
                }
            }
        } else if is_seed {
            result.propagate(ground_side, Growth::Trunk);
            result.propagate(opposite(ground_side), Growth::Trunk);
        }
        for (parent_side, growth) in incoming_sides {
            match growth.at_height(estimated_height) {
                Growth::Trunk => result.propagate(opposite(parent_side), Growth::Trunk),
                Growth::Branching => {
                    for side in Side::iter() {
                        if side != parent_side
                            && !side.adjacent_to(parent_side)
                            && rng.random::<f32>() < branch_probability(temperature)
                        {
                            result.propagate(side, Growth::Branching);
                        }
                    }
                }
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

    pub fn branch_sides(self) -> impl Iterator<Item = Side> {
        Side::iter().filter(move |&side| self.branch_sides & (1 << side as usize) != 0)
    }

    pub fn is_terminal(self) -> bool {
        self.child_sides == 0
    }
}

fn tree_generation_probability(precipitation: f32) -> f32 {
    (precipitation * SPAWN_RATE).clamp(0.0, 1.0)
}

fn branch_probability(temperature: f32) -> f32 {
    ((temperature - BRANCH_TEMPERATURE_MIN) / (BRANCH_TEMPERATURE_MAX - BRANCH_TEMPERATURE_MIN))
        .clamp(0.0, 1.0)
}

fn opposite(side: Side) -> Side {
    let nonadjacent: Vec<_> = Side::iter()
        .filter(|&candidate| candidate != side && !candidate.adjacent_to(side))
        .collect();
    let mut opposites = nonadjacent.iter().copied().filter(|&candidate| {
        nonadjacent
            .iter()
            .all(|&other| candidate == other || candidate.adjacent_to(other))
    });
    let opposite = opposites.next().expect("side has an opposite side");
    assert!(
        opposites.next().is_none(),
        "side has multiple opposite sides"
    );
    opposite
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::worldgen::WorldgenConfig;

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
        MegatreeNode::new(&graph, child, kind, 0.0, 10.0, height, Side::A).unwrap()
    }

    #[test]
    fn trunk_continues_opposite_incoming_face_until_clear_of_ground() {
        for kind in [
            NodeStateKind::Land,
            NodeStateKind::Sky,
            NodeStateKind::DeepLand,
            NodeStateKind::DeepSky,
        ] {
            for height in [0.0, 0.75] {
                let tree = inherited_tree(kind, height, Growth::Trunk);
                assert_eq!(tree.child_sides, 1 << opposite(Side::B) as usize);
                assert_eq!(tree.branching_sides, 0);
                assert!(tree.branch_sides().any(|side| side == Side::B));
            }
        }
    }

    #[test]
    fn cleared_trunk_branches_and_preserves_branching_state() {
        for (height, growth) in [(0.751, Growth::Trunk), (-1.0, Growth::Branching)] {
            let tree = inherited_tree(NodeStateKind::DeepLand, height, growth);
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
    fn land_seed_uses_supplied_ground_axis() {
        let graph = Graph::new(1);
        let tree = MegatreeNode::new(
            &graph,
            NodeId::ROOT,
            NodeStateKind::Land,
            100.0,
            10.0,
            0.0,
            Side::C,
        )
        .unwrap();
        assert_eq!(
            tree.child_sides,
            (1 << Side::C as usize) | (1 << opposite(Side::C) as usize)
        );
        assert_eq!(tree.branching_sides, 0);
    }
}
