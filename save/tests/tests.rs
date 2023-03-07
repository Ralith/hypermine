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
