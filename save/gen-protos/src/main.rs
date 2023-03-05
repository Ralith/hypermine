use std::{io::Result, path::Path};

fn main() -> Result<()> {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .join("src");

    prost_build::Config::new()
        .out_dir(&dir)
        .compile_protos(&[dir.join("protos.proto")], &[dir])
}
