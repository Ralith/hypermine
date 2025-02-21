use save::Save;

use criterion::{
    BatchSize, BenchmarkId, Criterion, Throughput, black_box, criterion_group, criterion_main,
};
use rand::{Rng, SeedableRng, rngs::SmallRng};

fn save(c: &mut Criterion) {
    let mut write = c.benchmark_group("write");
    let node = save::VoxelNode {
        chunks: vec![save::Chunk {
            vertex: 0,
            voxels: vec![0; 12 * 12 * 12 * 2],
        }],
    };
    let mut rng = SmallRng::from_os_rng();
    for count in [1, 100, 10000] {
        write.throughput(Throughput::Elements(count));
        write.bench_function(BenchmarkId::from_parameter(count), |b| {
            b.iter_batched(
                || {
                    let file = tempfile::NamedTempFile::new().unwrap();
                    let save = Save::open(file.path(), 12).unwrap();
                    let node_ids = (&mut rng)
                        .sample_iter(rand::distr::StandardUniform)
                        .take(count as usize)
                        .collect::<Vec<u128>>();
                    (file, save, node_ids)
                },
                |(_file, save, node_ids)| {
                    let mut tx = save.write().unwrap();
                    let mut writer = tx.get().unwrap();
                    for i in node_ids {
                        writer.put_voxel_node(i, &node).unwrap();
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
                    let save = Save::open(file.path(), 12).unwrap();
                    let node_ids = (&mut rng)
                        .sample_iter(rand::distr::StandardUniform)
                        .take(count as usize)
                        .collect::<Vec<u128>>();

                    let mut tx = save.write().unwrap();
                    let mut writer = tx.get().unwrap();
                    for &i in &node_ids {
                        writer.put_voxel_node(i, &node).unwrap();
                    }
                    drop(writer);
                    tx.commit().unwrap();

                    (file, save, node_ids)
                },
                |(_file, save, node_ids)| {
                    let mut read = save.read().unwrap();
                    for i in node_ids {
                        black_box(read.get_voxel_node(i).unwrap().unwrap());
                    }
                },
                BatchSize::SmallInput,
            );
        });
    }
}

criterion_group!(benches, save);
criterion_main!(benches);
