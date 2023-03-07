use save::Save;

use criterion::{
    black_box, criterion_group, criterion_main, BatchSize, BenchmarkId, Criterion, Throughput,
};
use rand::{rngs::SmallRng, Rng, SeedableRng};

fn save(c: &mut Criterion) {
    let mut write = c.benchmark_group("write");
    let node = save::Node {
        archetypes: vec![save::Archetype {
            entities: vec![1, 2, 3],
            component_types: vec![4, 5, 6],
            component_data: vec![Vec::new(), Vec::new(), Vec::new()],
        }],
        chunks: vec![save::Chunk {
            vertex: 0,
            voxels: vec![0; 12 * 12 * 12 * 2],
        }],
    };
    let mut rng = SmallRng::from_entropy();
    for count in [1, 100, 10000] {
        write.throughput(Throughput::Elements(count));
        write.bench_function(BenchmarkId::from_parameter(count), |b| {
            b.iter_batched(
                || {
                    let file = tempfile::NamedTempFile::new().unwrap();
                    let save = Save::open(file.path(), 12).unwrap();
                    let node_ids = (&mut rng)
                        .sample_iter(rand::distributions::Standard)
                        .take(count as usize)
                        .collect::<Vec<u128>>();
                    (file, save, node_ids)
                },
                |(_file, mut save, node_ids)| {
                    let mut tx = save.write().unwrap();
                    let mut writer = tx.get().unwrap();
                    for i in node_ids {
                        writer.put(i, &node).unwrap();
                    }
                    drop(writer);
                    tx.commit().unwrap();
                },
                BatchSize::SmallInput,
            );
        });
    }
    write.finish();

    let mut read = c.benchmark_group("read");
    for count in [1, 100, 10000] {
        read.throughput(Throughput::Elements(count));
        read.bench_function(BenchmarkId::from_parameter(count), |b| {
            b.iter_batched(
                || {
                    let file = tempfile::NamedTempFile::new().unwrap();
                    let mut save = Save::open(file.path(), 12).unwrap();
                    let node_ids = (&mut rng)
                        .sample_iter(rand::distributions::Standard)
                        .take(count as usize)
                        .collect::<Vec<u128>>();

                    let mut tx = save.write().unwrap();
                    let mut writer = tx.get().unwrap();
                    for &i in &node_ids {
                        writer.put(i, &node).unwrap();
                    }
                    drop(writer);
                    tx.commit().unwrap();

                    (file, save, node_ids)
                },
                |(_file, save, node_ids)| {
                    let read = save.read().unwrap();
                    let mut read = read.get().unwrap();
                    for i in node_ids {
                        black_box(read.get(i).unwrap().unwrap());
                    }
                },
                BatchSize::SmallInput,
            );
        });
    }
}

criterion_group!(benches, save);
criterion_main!(benches);
