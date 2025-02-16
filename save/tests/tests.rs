use rand::{rngs::SmallRng, Rng, SeedableRng};

use save::Save;

#[test]
fn persist_meta() {
    let file = tempfile::NamedTempFile::new().unwrap();
    let save = Save::open(file.path(), 12).unwrap();
    assert_eq!(save.meta().chunk_size, 12);
    drop(save);
    let save = Save::open(file.path(), 8).unwrap();
    assert_eq!(save.meta().chunk_size, 12);
}

#[test]
fn persist_node() {
    let file = tempfile::NamedTempFile::new().unwrap();
    let save = Save::open(file.path(), 12).unwrap();
    let node = save::VoxelNode {
        chunks: vec![save::Chunk {
            vertex: 0,
            voxels: vec![0; 12 * 12 * 12 * 2],
        }],
    };
    let mut writer_guard = save.write().unwrap();
    writer_guard
        .get()
        .unwrap()
        .put_voxel_node(0, &node)
        .unwrap();
    writer_guard.commit().unwrap();
    assert_eq!(
        node,
        save.read().unwrap().get_voxel_node(0).unwrap().unwrap()
    );
}

#[test]
fn persist_character() {
    let file = tempfile::NamedTempFile::new().unwrap();
    let save = Save::open(file.path(), 12).unwrap();
    let mut writer_guard = save.write().unwrap();
    let mut writer = writer_guard.get().unwrap();
    let mut rng = SmallRng::from_os_rng();
    let mut path = Vec::with_capacity(17000);
    for _ in 0..17000 {
        path.push(rng.random_range(0..12));
    }
    let ch = save::Character { path };
    writer.put_character("asdf", &ch).unwrap();
    drop(writer);
    writer_guard.commit().unwrap();
    drop(save);

    let save = Save::open(file.path(), 12).unwrap();
    assert_eq!(
        ch,
        save.read().unwrap().get_character("asdf").unwrap().unwrap()
    );
}
