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
    let mut writer_guard = save.write().unwrap();
    writer_guard.get().unwrap().put_node(0, &node).unwrap();
    writer_guard.commit().unwrap();
    assert_eq!(
        node,
        save.read()
            .unwrap()
            .get()
            .unwrap()
            .get_node(0)
            .unwrap()
            .unwrap()
    );
}

#[test]
fn persist_character() {
    let file = tempfile::NamedTempFile::new().unwrap();
    let mut save = Save::open(file.path(), 12).unwrap();
    let mut writer_guard = save.write().unwrap();
    let mut writer = writer_guard.get().unwrap();
    let mut rng = SmallRng::from_entropy();
    let mut path = Vec::with_capacity(17000);
    for _ in 0..17000 {
        path.push(rng.gen_range(0..12));
    }
    let ch = save::Character { path };
    writer.put_character("asdf", &ch).unwrap();
    drop(writer);
    writer_guard.commit().unwrap();
    drop(save);

    let save = Save::open(file.path(), 12).unwrap();
    assert_eq!(
        ch,
        save.read()
            .unwrap()
            .get()
            .unwrap()
            .get_character("asdf")
            .unwrap()
            .unwrap()
    );
}
