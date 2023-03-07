use std::time::Instant;

use save::Save;

use rand::{rngs::SmallRng, Rng, SeedableRng};

#[test]
fn write() {
    let mut rng = SmallRng::from_entropy();
    let file = tempfile::NamedTempFile::new().unwrap();
    let mut save = Save::open(file.path(), 12).unwrap();
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

    let start = Instant::now();
    const PASSES: u32 = 100;
    const NODES: u32 = 1_000;
    for _ in 0..PASSES {
        let mut writer_guard = save.write().unwrap();
        let mut writer = writer_guard.get().unwrap();
        for _ in 0..NODES {
            writer.put(rng.gen(), &node).unwrap();
        }
        drop(writer);
        writer_guard.commit().unwrap();
    }
    let dt = start.elapsed();
    println!(
        "{:?} per pass, {:?} per node",
        dt / PASSES,
        dt / (PASSES * NODES)
    );
}
