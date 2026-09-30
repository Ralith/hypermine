use crate::{
    dodeca::Side,
    graph::{Graph, NodeId},
};
use rand::{RngExt, SeedableRng};
use rand_pcg::Pcg64Mcg;

use super::NodeStateKind;

/// Radius of Yendorian branches, in absolute hyperbolic distance units.
pub(super) const YENDORIAN_BRANCH_RADIUS: f32 = 0.2;
/// Radius of the leaves ball at a terminal Yendorian node.
pub(super) const YENDORIAN_LEAVES_RADIUS: f32 = 0.5;
/// Node-center depth required to switch Yendorian propagation into underground mode.
pub(super) const YENDORIAN_UNDERGROUND_DEPTH_THRESHOLD: f32 = 1.5 * YENDORIAN_BRANCH_RADIUS;
/// Yendorian tree spawn probability per precipitation unit at Land nodes.
const YENDORIAN_TREE_SPAWN_RATE: f32 = 0.025;
/// Lower and upper temperature limits for the linear branch-probability ramp.
const YENDORIAN_BRANCH_TEMPERATURE_MIN: f32 = -10.0;
const YENDORIAN_BRANCH_TEMPERATURE_MAX: f32 = 10.0;
/// Value mixed into each node's hash to seed Yendorian generation. Chosen randomly.
const YENDORIAN_SEED: u64 = 13334231312061724180;

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
        is_deep_underground: bool,
    ) -> Option<Self> {
        let spice = graph.hash_of(node) as u64;
        let mut rng = Pcg64Mcg::seed_from_u64(super::hash(spice, YENDORIAN_SEED));
        let parent_sides: Vec<_> = graph.parents(node).map(|(side, _)| side).collect();

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

        let is_seed = incoming_sides.is_empty();
        let is_sky = matches!(kind, NodeStateKind::Sky | NodeStateKind::DeepSky);
        // Any underground Sky node can independently seed trees through its
        // eligible exits, whether or not it inherited Yendorian state.
        let is_underground_sky = is_deep_underground && is_sky;
        if is_seed && !is_underground_sky {
            // Only Land nodes may own a tree. Each node's deterministic roll
            // scales linearly with precipitation.
            if kind != NodeStateKind::Land || !tree_seed_is_selected(&mut rng, precipitation) {
                return None;
            }
        }

        let mut child_sides = 0;
        let mut branch_sides = incoming_sides
            .iter()
            .copied()
            .fold(0, |mask, side| mask | side_bit(side));
        if is_deep_underground && is_sky {
            // Underground nodes may descend only through sides that are neither
            // equal nor adjacent to any graph parent face. Underground Sky
            // nodes make this decision independently of incoming tree state.
            let probability = tree_generation_probability(precipitation);
            for side in Side::iter() {
                if parent_sides
                    .iter()
                    .all(|&parent_side| side != parent_side && !side.adjacent_to(parent_side))
                    && side_is_selected(&mut rng, probability)
                {
                    child_sides |= side_bit(side);
                }
            }
        } else if is_seed {
            child_sides = side_bit(Side::A) | side_bit(Side::J);
        } else {
            match kind {
                NodeStateKind::Sky | NodeStateKind::DeepSky => {
                    // Union the six non-adjacent-to-parent exits for every incoming
                    // tree path. This preserves all branches when paths converge.
                    for parent_side in incoming_sides {
                        for side in Side::iter() {
                            if side != parent_side
                                && !side.adjacent_to(parent_side)
                                && branch_is_selected(&mut rng, temperature)
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
                // Land nodes other than the seed do not continue the trunk.
                NodeStateKind::Land => {}
            }
        }

        // A newly seeded tree exists only if at least one exit was selected.
        if is_seed && child_sides == 0 {
            return None;
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

/// Makes a deterministic tree-seed decision using this node's RNG.
fn tree_seed_is_selected(rng: &mut Pcg64Mcg, precipitation: f32) -> bool {
    rng.random::<f32>() < tree_generation_probability(precipitation)
}

fn tree_generation_probability(precipitation: f32) -> f32 {
    (precipitation * YENDORIAN_TREE_SPAWN_RATE).clamp(0.0, 1.0)
}

fn branch_is_selected(rng: &mut Pcg64Mcg, temperature: f32) -> bool {
    side_is_selected(rng, branch_probability(temperature))
}

fn side_is_selected(rng: &mut Pcg64Mcg, probability: f32) -> bool {
    rng.random::<f32>() < probability
}

fn branch_probability(temperature: f32) -> f32 {
    let temperature_fraction = ((temperature - YENDORIAN_BRANCH_TEMPERATURE_MIN)
        / (YENDORIAN_BRANCH_TEMPERATURE_MAX - YENDORIAN_BRANCH_TEMPERATURE_MIN))
        .clamp(0.0, 1.0);
    temperature_fraction
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
