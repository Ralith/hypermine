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
    writer_guard.get().unwrap().put(0, &node).unwrap();
    writer_guard.commit().unwrap();
    assert_eq!(
        node,
        save.read().unwrap().get().unwrap().get(0).unwrap().unwrap()
    );
}
