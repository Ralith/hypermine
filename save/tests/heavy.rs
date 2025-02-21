use std::time::Instant;

use save::Save;

use rand::{Rng, SeedableRng, rngs::SmallRng};

#[test]
fn write() {
    let mut rng = SmallRng::from_os_rng();
    let file = tempfile::NamedTempFile::new().unwrap();
    let save = Save::open(file.path(), 12).unwrap();
    let node = save::VoxelNode {
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
            writer.put_voxel_node(rng.random(), &node).unwrap();
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
