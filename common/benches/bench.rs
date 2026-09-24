use criterion::{Criterion, criterion_group, criterion_main};

use common::{
    dodeca::{Side, Vertex},
    graph::{Graph, NodeId},
    node::{Chunk, ChunkId},
    proto::Position,
    traversal::{ensure_nearby, nearby_nodes},
    worldgen::{ChunkParams, WorldgenConfig},
};

fn build_graph(c: &mut Criterion) {
    let worldgen_cfg = WorldgenConfig::default();

    c.bench_function("build_graph 1000", |b| {
        b.iter(|| {
            let mut graph = Graph::new(12);
            let mut n = NodeId::ROOT;
            for _ in 0..500 {
                n = graph.ensure_neighbor(n, Side::A);
                n = graph.ensure_neighbor(n, Side::J);
            }
            assert_eq!(graph.len(), 1001);
        })
    });

    c.bench_function("nodegen 1000", |b| {
        b.iter(|| {
            let mut graph = Graph::new(12);
            let mut n = NodeId::ROOT;
            for _ in 0..500 {
                n = graph.ensure_neighbor(n, Side::A);
                graph.ensure_node_state(n, &worldgen_cfg);
                n = graph.ensure_neighbor(n, Side::J);
                graph.ensure_node_state(n, &worldgen_cfg);
            }
            assert_eq!(graph.len(), 1001);
        })
    });

    c.bench_function("worldgen", |b| {
        b.iter(|| {
            let mut graph = Graph::new(12);
            ensure_nearby(&mut graph, &Position::origin(), 1.0);
            let all_nodes = nearby_nodes(&graph, &Position::origin(), 1.0);
            let mut n = 0;
            for (node, _) in all_nodes {
                for vertex in Vertex::iter() {
                    let chunk = ChunkId::new(node, vertex);
                    let params = ChunkParams::new(&mut graph, chunk, &worldgen_cfg);
                    graph[chunk] = Chunk::Populated {
                        voxels: params.generate_voxels(),
                        surface: None,
                        old_surface: None,
                    };
                    n += 1;
                }
            }
            assert_eq!(n, 860);
        })
    });
}

criterion_group!(benches, build_graph);
criterion_main!(benches);
