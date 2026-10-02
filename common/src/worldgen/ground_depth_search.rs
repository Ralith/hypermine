//! Reproducible, opt-in searches beyond the complete graph ball tests.
use super::*;
use rand::SeedableRng;
use rand_pcg::Pcg64Mcg;

#[test]
#[ignore = "compares plane-only groundward directions against the graph oracle to depth 500"]
fn adversarial_groundward_side_search() {
    let cfg = WorldgenConfig::default();
    let checkpoints = [20, 50, 100, 200, 500];
    let mut checked = 0;
    let mut deep_sky = 0;
    let mut deep_land = 0;
    let mut multiple_parents = 0;
    for seed in 0..12u64 {
        for lateral in [false, true] {
            let mut reference = Vec::new();
            for reverse in [false, true] {
                let mut graph = Graph::new(1);
                let mut rng = Pcg64Mcg::seed_from_u64(seed);
                let mut path = Vec::new();
                let mut node = NodeId::ROOT;
                if seed % 2 == 1 {
                    node = graph.ensure_neighbor(node, Side::A);
                    path.push(Side::A);
                }
                let mut signatures = Vec::new();
                for step in 1..=500 {
                    let parent_sides = graph
                        .parents(node)
                        .map(|(side, _)| side)
                        .collect::<Vec<_>>();
                    let choices = Side::iter()
                        .filter(|side| !parent_sides.contains(side))
                        .filter(|side| !lateral || side.adjacent_to(Side::A))
                        .collect::<Vec<_>>();
                    let side = choices[rng.random_range(0..choices.len())];
                    node = graph.ensure_neighbor(node, side);
                    path.push(side);
                    if !checkpoints.contains(&step) {
                        continue;
                    }

                    graph.ensure_node_state(node, &cfg);
                    let state = graph.node_state(node);
                    if !matches!(state.kind, DeepSky | DeepLand) {
                        continue;
                    }
                    let surface = state.surface;
                    let kind = state.kind;
                    let current_distance = surface.distance_to(&MPoint::origin()).abs();
                    let per_side = Side::iter()
                        .map(|side| {
                            let neighbor = side.reflection() * MPoint::origin();
                            (side, surface.distance_to(&neighbor).abs())
                        })
                        .collect::<Vec<_>>();
                    let oracle = graph
                        .ground_parents(node, &cfg)
                        .into_iter()
                        .map(|(side, _)| side)
                        .collect::<Vec<_>>();
                    let geometric = groundward_sides(&surface);
                    let parent_count = graph.parents(node).len();
                    checked += 1;
                    deep_sky += usize::from(kind == DeepSky);
                    deep_land += usize::from(kind == DeepLand);
                    multiple_parents += usize::from(parent_count > 1);
                    assert_eq!(
                        geometric,
                        oracle,
                        "groundward mismatch: seed={seed}, lateral={lateral}, reverse={reverse}, node={node:?}, graph_depth={}, kind={kind:?}, surface={surface:?}, current_distance={current_distance}, per_side={per_side:?}, oracle={oracle:?}, geometric={geometric:?}, root_path={path:?}",
                        graph.depth(node),
                    );
                    signatures.push((node, geometric));
                }
                if reverse {
                    assert_eq!(
                        reference, signatures,
                        "expansion order changed results: seed={seed}, lateral={lateral}"
                    );
                } else {
                    reference = signatures;
                }
            }
        }
    }
    println!(
        "adversarial groundward search: checked={checked}, DeepSky={deep_sky}, DeepLand={deep_land}, multi-parent={multiple_parents}, max_graph_depth=500"
    );
}
