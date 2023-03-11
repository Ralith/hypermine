use criterion::{criterion_group, criterion_main, Criterion};

use common::{
    dodeca::Side,
    graph::{Graph, NodeId},
};

fn build_graph(c: &mut Criterion) {
    c.bench_function("build_graph 1000", |b| {
        b.iter(|| {
            let mut graph = Graph::<()>::new();
            let mut n = NodeId::ROOT;
            for _ in 0..500 {
                n = graph.ensure_neighbor(n, Side::A);
                n = graph.ensure_neighbor(n, Side::J);
            }
            assert_eq!(graph.len(), 1001);
        })
    });
}

criterion_group!(benches, build_graph);
criterion_main!(benches);
