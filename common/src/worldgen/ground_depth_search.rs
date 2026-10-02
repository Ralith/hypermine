//! Reproducible, opt-in searches beyond the complete graph ball tests.
use super::*;
use std::collections::{HashSet, VecDeque};

fn shortest_ground_distance(
    graph: &mut Graph,
    start: NodeId,
    cfg: &WorldgenConfig,
    budget: usize,
) -> Option<u32> {
    let mut seen = HashSet::from([start]);
    let mut queue = VecDeque::from([(start, 0)]);
    while let Some((node, distance)) = queue.pop_front() {
        graph.ensure_node_state(node, cfg);
        if matches!(graph.node_state(node).kind, Sky | Land) {
            return Some(distance);
        }
        for side in Side::iter() {
            let neighbor = graph.ensure_neighbor(node, side);
            if seen.insert(neighbor) {
                if seen.len() > budget {
                    return None;
                }
                queue.push_back((neighbor, distance + 1));
            }
        }
    }
    unreachable!("the graph is connected to the ground layer")
}

#[derive(Default, Debug)]
struct Stats {
    samples: usize,
    unique_nodes: HashSet<NodeId>,
    deep_sky: usize,
    deep_land: usize,
    multiple_parents: usize,
    max_graph_depth: u32,
    max_ground_depth: u32,
    bfs_completed: usize,
    bfs_unique: HashSet<NodeId>,
    max_bfs_distance: u32,
    bfs_budget_exhausted: usize,
}

fn inspect(
    graph: &mut Graph,
    node: NodeId,
    path: &[Side],
    seed: u64,
    reverse: bool,
    stats: &mut Stats,
) -> Vec<u32> {
    let cfg = WorldgenConfig::default();
    graph.ensure_node_state(node, &cfg);
    let state = graph.node_state(node);
    assert!(matches!(state.kind, DeepSky | DeepLand));
    let depth = state.ground_depth;
    stats.samples += 1;
    stats.unique_nodes.insert(node);
    stats.deep_sky += usize::from(state.kind == DeepSky);
    stats.deep_land += usize::from(state.kind == DeepLand);
    stats.max_graph_depth = stats.max_graph_depth.max(graph.depth(node));
    stats.max_ground_depth = stats.max_ground_depth.max(depth);

    let mut sides = Side::VALUES;
    if reverse {
        sides.reverse();
    }
    for side in sides {
        let neighbor = graph.ensure_neighbor(node, side);
        graph.ensure_node_state(neighbor, &cfg);
    }
    let parents = graph.parents(node).map(|(_, n)| n).collect::<HashSet<_>>();
    stats.multiple_parents += usize::from(parents.len() > 1);
    let neighbors = Side::iter()
        .map(|side| {
            let n = graph.neighbor(node, side).unwrap();
            (
                side,
                n,
                graph.node_state(n).ground_depth,
                parents.contains(&n),
            )
        })
        .collect::<Vec<_>>();
    let all_min = neighbors.iter().map(|n| n.2).min().unwrap();
    let parent_min = neighbors.iter().filter(|n| n.3).map(|n| n.2).min().unwrap();
    let suspect = all_min < parent_min;
    let bfs = if suspect || depth <= 3 || seed.is_multiple_of(4) {
        let result = shortest_ground_distance(graph, node, &cfg, 10_000);
        stats.bfs_completed += usize::from(result.is_some());
        if let Some(distance) = result {
            stats.bfs_unique.insert(node);
            stats.max_bfs_distance = stats.max_bfs_distance.max(distance);
        }
        stats.bfs_budget_exhausted += usize::from(result.is_none());
        result
    } else {
        None
    };
    assert!(
        !suspect && bfs.is_none_or(|d| d == depth),
        "counterexample: seed={seed}, reverse={reverse}, node={node:?}, graph_depth={}, stored_ground_depth={depth}, true_ground_distance={bfs:?} (None means BFS budget exhausted), all_min={all_min}, parent_min={parent_min}, neighbors=(side,id,depth,is_graph_parent): {neighbors:?}, root_path={path:?}",
        graph.depth(node)
    );
    let mut signature = vec![depth];
    signature.extend(neighbors.iter().map(|n| n.2));
    signature
}

#[test]
#[ignore = "adversarial search; run explicitly with --ignored --nocapture"]
fn adversarial_ground_depth_search() {
    let cfg = WorldgenConfig::default();
    let mut stats = Stats::default();
    for seed in 0..128u64 {
        for lateral in [true, false] {
            let mut reference = Vec::new();
            for reverse in [false, true] {
                let mut graph = Graph::new(1);
                let mut expansion_sides = Side::VALUES;
                if reverse {
                    expansion_sides.reverse();
                }
                for side in expansion_sides {
                    graph.ensure_neighbor(NodeId::ROOT, side);
                }
                let mut rng = Pcg64Mcg::seed_from_u64(seed);
                let mut path = Vec::new();
                let mut node = NodeId::ROOT;
                if seed % 2 == 1 {
                    node = graph.ensure_neighbor(node, Side::A);
                    path.push(Side::A);
                }
                let mut signatures = Vec::new();
                for step in 1..=500 {
                    // Excluding every graph parent makes these paths geodesic,
                    // hence non-backtracking, even when adjacent faces commute.
                    let parent_sides = graph.parents(node).map(|(s, _)| s).collect::<Vec<_>>();
                    let choices = Side::iter()
                        .filter(|s| !parent_sides.contains(s))
                        .filter(|s| !lateral || s.adjacent_to(Side::A))
                        .collect::<Vec<_>>();
                    let side = choices[rng.random_range(0..choices.len())];
                    node = graph.ensure_neighbor(node, side);
                    path.push(side);
                    graph.ensure_node_state(node, &cfg);
                    if ![20, 50, 100, 200, 500].contains(&step) {
                        continue;
                    }
                    let mut sample = node;
                    let mut sample_path = path.clone();
                    for excursion in 0..=4 {
                        if lateral && excursion == 0 {
                            assert!(matches!(graph.node_state(sample).kind, Sky | Land));
                            continue;
                        }
                        if lateral {
                            let parents = graph.parents(sample).map(|(s, _)| s).collect::<Vec<_>>();
                            let choices = Side::iter()
                                .filter(|s| !parents.contains(s))
                                .filter(|s| {
                                    excursion != 1 || (*s != Side::A && !s.adjacent_to(Side::A))
                                })
                                .collect::<Vec<_>>();
                            let side = choices[rng.random_range(0..choices.len())];
                            sample = graph.ensure_neighbor(sample, side);
                            sample_path.push(side);
                        }
                        signatures.push((
                            sample,
                            inspect(&mut graph, sample, &sample_path, seed, reverse, &mut stats),
                        ));
                        if !lateral {
                            break;
                        }
                    }
                }
                if reverse {
                    assert_eq!(
                        reference, signatures,
                        "expansion order changed depths: seed={seed}, lateral={lateral}"
                    );
                } else {
                    reference = signatures;
                }
            }
        }
        if (seed + 1).is_multiple_of(16) {
            println!(
                "seed={seed}: samples={}, unique={}, max_graph_depth={}, max_ground_depth={}, DeepSky={}, DeepLand={}, multiple_parents={}, BFS_completed={}, BFS_budget_exhausted={}",
                stats.samples,
                stats.unique_nodes.len(),
                stats.max_graph_depth,
                stats.max_ground_depth,
                stats.deep_sky,
                stats.deep_land,
                stats.multiple_parents,
                stats.bfs_completed,
                stats.bfs_budget_exhausted
            );
            println!(
                "BFS unique nodes={}, max completed BFS distance={}",
                stats.bfs_unique.len(),
                stats.max_bfs_distance
            );
        }
    }
}

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
