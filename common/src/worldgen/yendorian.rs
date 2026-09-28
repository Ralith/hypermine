use crate::{
    dodeca::Side,
    graph::{Graph, NodeId},
};
use rand::{RngExt, SeedableRng};
use rand_pcg::Pcg64Mcg;

use super::NodeStateKind;

/// Yendorian-tree propagation information for one node.
///
/// The bit at each side indicates that this node propagates its tree state to
/// the neighbor on that side.
#[derive(Clone, Copy)]
pub(super) struct YendorianNode {
    child_sides: u16,
    branch_sides: u16,
}

impl YendorianNode {
    pub(super) fn new(
        graph: &Graph,
        node: NodeId,
        kind: NodeStateKind,
        precipitation: f32,
        temperature: f32,
    ) -> Option<Self> {
        // `Graph::parents` returns every shallower neighbor and the shared side.
        // Parent states have already been finalized by `ensure_node_state`.
        let incoming_sides: Vec<_> = graph
            .parents(node)
            .filter_map(|(side, parent)| {
                graph
                    .node_state(parent)
                    .yendorian
                    .filter(|state| state.propagates_through(side))
                    .map(|_| side)
            })
            .collect();

        if incoming_sides.is_empty() {
            // Only Land nodes may own a tree. Each node's deterministic roll
            // scales linearly with precipitation, reaching certainty at 30.
            if kind != NodeStateKind::Land || !tree_seed_is_selected(graph, node, precipitation) {
                return None;
            }
            return Some(Self {
                child_sides: side_bit(Side::A) | side_bit(Side::J),
                branch_sides: side_bit(Side::A) | side_bit(Side::J),
            });
        }

        let mut child_sides = 0;
        let mut branch_sides = incoming_sides
            .iter()
            .copied()
            .fold(0, |mask, side| mask | side_bit(side));
        match kind {
            NodeStateKind::Sky | NodeStateKind::DeepSky => {
                // Union the six non-adjacent-to-parent exits for every incoming
                // tree path. This preserves all branches when paths converge.
                for parent_side in incoming_sides {
                    for side in Side::iter() {
                        if side != parent_side
                            && !side.adjacent_to(parent_side)
                            && branch_is_selected(graph, node, side, temperature)
                        {
                            child_sides |= side_bit(side);
                        }
                    }
                }
            }
            NodeStateKind::DeepLand => {
                // The below-ground trunk continues straight through the face
                // opposite each incoming path.
                for parent_side in incoming_sides {
                    child_sides |= side_bit(opposite(parent_side));
                }
            }
            // Land nodes other than the root do not continue the trunk.
            NodeStateKind::Land => {}
        }

        branch_sides |= child_sides;
        Some(Self {
            child_sides,
            branch_sides,
        })
    }

    fn propagates_through(self, side: Side) -> bool {
        self.child_sides & side_bit(side) != 0
    }

    pub(super) fn branch_sides(self) -> impl Iterator<Item = Side> {
        Side::iter().filter(move |&side| self.branch_sides & side_bit(side) != 0)
    }

    pub(super) fn is_terminal(self) -> bool {
        self.child_sides == 0
    }
}

/// Makes a stable, independent propagation decision for a node-side pair.
fn tree_seed_is_selected(graph: &Graph, node: NodeId, precipitation: f32) -> bool {
    let probability = (precipitation * super::YENDORIAN_TREE_SPAWN_RATE).clamp(
        super::YENDORIAN_PROBABILITY_MIN,
        super::YENDORIAN_PROBABILITY_MAX,
    );
    let spice = graph.hash_of(node) as u64;
    let mut rng = Pcg64Mcg::seed_from_u64(super::hash(spice, u64::MAX));
    rng.random::<f32>() < probability
}

fn branch_is_selected(graph: &Graph, node: NodeId, side: Side, temperature: f32) -> bool {
    let spice = graph.hash_of(node) as u64;
    let mut rng = Pcg64Mcg::seed_from_u64(super::hash(spice, side as u64));
    rng.random::<f32>() < branch_probability(temperature)
}

fn branch_probability(temperature: f32) -> f32 {
    let temperature_fraction = ((temperature - super::YENDORIAN_BRANCH_TEMPERATURE_MIN)
        / (super::YENDORIAN_BRANCH_TEMPERATURE_MAX - super::YENDORIAN_BRANCH_TEMPERATURE_MIN))
        .clamp(
            super::YENDORIAN_PROBABILITY_MIN,
            super::YENDORIAN_PROBABILITY_MAX,
        );
    super::YENDORIAN_BRANCH_PROBABILITY_MIN
        + temperature_fraction
            * (super::YENDORIAN_BRANCH_PROBABILITY_MAX - super::YENDORIAN_BRANCH_PROBABILITY_MIN)
}

fn side_bit(side: Side) -> u16 {
    1 << side as usize
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
