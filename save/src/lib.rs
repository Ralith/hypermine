mod protos;

use std::path::Path;

use prost::Message;
use redb::{Database, ReadableTable, TableDefinition};
use thiserror::Error;

pub use protos::*;

pub struct Save {
    meta: Meta,
    db: Database,
}

impl Save {
    pub fn open(path: &Path, default_chunk_size: u8) -> Result<Self, OpenError> {
        let db = Database::create(path)?;
        let meta = {
            let tx = db.begin_read()?;
            match tx.open_table(META_TABLE) {
                Ok(meta) => {
                    let Some(value) = meta.get(&[][..])? else { return Err(OpenError::MissingMeta); };
                    let mut dctx = dctx();
                    let mut buffer = Vec::new();
                    decompress(&mut dctx, value.value(), &mut buffer)
                        .map_err(OpenError::DecompressionFailed)?;
                    Meta::decode(&*buffer)?
                }
                Err(redb::Error::TableDoesNotExist(_)) => {
                    // Must be an empty save file. Initialize the meta record and create the other tables.
                    let defaults = Meta {
                        chunk_size: default_chunk_size.into(),
                    };
                    let tx = db.begin_write()?;
                    let mut meta = tx.open_table(META_TABLE)?;
                    let mut cctx = cctx();
                    let mut plain = Vec::new();
                    let mut compressed = Vec::new();
                    prepare(&mut cctx, &mut plain, &mut compressed, &defaults);
                    meta.insert(&[][..], &*compressed)?;
                    drop(meta);

                    tx.open_table(VOXEL_NODE_TABLE)?;
                    tx.open_table(ENTITY_NODE_TABLE)?;
                    tx.open_table(CHARACTERS_BY_NAME_TABLE)?;
                    tx.commit()?;
                    defaults.clone()
                }
                Err(e) => return Err(OpenError::Db(DbError(e))),
            }
        };
        Ok(Self { meta, db })
    }

    #[inline]
    pub fn meta(&self) -> &Meta {
        &self.meta
    }

    pub fn read(&self) -> Result<ReaderGuard<'_>, DbError> {
        let tx = self.db.begin_read()?;
        Ok(ReaderGuard { tx })
    }

    pub fn write(&mut self) -> Result<WriterGuard<'_>, DbError> {
        let tx = self.db.begin_write()?;
        Ok(WriterGuard { tx })
    }
}

pub struct ReaderGuard<'a> {
    tx: redb::ReadTransaction<'a>,
}

impl ReaderGuard<'_> {
    pub fn get(&self) -> Result<Reader<'_>, DbError> {
        Ok(Reader {
            voxel_nodes: self.tx.open_table(VOXEL_NODE_TABLE)?,
            entity_nodes: self.tx.open_table(ENTITY_NODE_TABLE)?,
            characters: self.tx.open_table(CHARACTERS_BY_NAME_TABLE)?,
            dctx: dctx(),
            accum: Vec::new(),
        })
    }
}

fn dctx() -> zstd::DCtx<'static> {
    let mut dctx = zstd::DCtx::create();
    dctx.set_parameter(zstd::DParameter::Format(zstd::FrameFormat::Magicless))
        .unwrap();
    dctx
}

pub struct Reader<'a> {
    voxel_nodes: redb::ReadOnlyTable<'a, u128, &'static [u8]>,
    entity_nodes: redb::ReadOnlyTable<'a, u128, &'static [u8]>,
    characters: redb::ReadOnlyTable<'a, &'static str, &'static [u8]>,
    dctx: zstd::DCtx<'static>,
    accum: Vec<u8>,
}

impl Reader<'_> {
    pub fn get_voxel_node(&mut self, node_id: u128) -> Result<Option<VoxelNode>, GetError> {
        let Some(node) = self.voxel_nodes.get(&node_id)? else { return Ok(None); };
        self.accum.clear();
        decompress(&mut self.dctx, node.value(), &mut self.accum)
            .map_err(GetError::DecompressionFailed)?;
        Ok(Some(VoxelNode::decode(&*self.accum)?))
    }

    pub fn get_entity_node(&mut self, node_id: u128) -> Result<Option<EntityNode>, GetError> {
        let Some(node) = self.entity_nodes.get(&node_id)? else { return Ok(None); };
        self.accum.clear();
        decompress(&mut self.dctx, node.value(), &mut self.accum)
            .map_err(GetError::DecompressionFailed)?;
        Ok(Some(EntityNode::decode(&*self.accum)?))
    }

    pub fn get_character(&mut self, name: &str) -> Result<Option<Character>, GetError> {
        let Some(node) = self.characters.get(name)? else { return Ok(None); };
        self.accum.clear();
        decompress(&mut self.dctx, node.value(), &mut self.accum)
            .map_err(GetError::DecompressionFailed)?;
        Ok(Some(Character::decode(&*self.accum)?))
    }
}

fn decompress(
    dctx: &mut zstd::DCtx<'_>,
    compressed: &[u8],
    out: &mut Vec<u8>,
) -> Result<(), &'static str> {
    dctx.init().map_err(zstd::get_error_name)?;
    let mut input = zstd::InBuffer::around(compressed);
    let mut out = zstd::OutBuffer::around(out);
    let out_size = zstd::DCtx::out_size();
    loop {
        if out.dst.len() + out_size > out.dst.capacity() {
            out.dst.reserve(out_size);
        }
        let n = dctx
            .decompress_stream(&mut out, &mut input)
            .map_err(zstd::get_error_name)?;
        if n == 0 {
            return Ok(());
        }
    }
}

pub struct WriterGuard<'a> {
    tx: redb::WriteTransaction<'a>,
}

impl<'a> WriterGuard<'a> {
    pub fn get(&mut self) -> Result<Writer<'a, '_>, DbError> {
        Ok(Writer {
            voxel_nodes: self.tx.open_table(VOXEL_NODE_TABLE)?,
            entity_nodes: self.tx.open_table(ENTITY_NODE_TABLE)?,
            characters: self.tx.open_table(CHARACTERS_BY_NAME_TABLE)?,
            cctx: cctx(),
            plain: Vec::new(),
            compressed: Vec::new(),
        })
    }

    pub fn commit(self) -> Result<(), DbError> {
        self.tx.commit()?;
        Ok(())
    }
}

fn cctx() -> zstd::CCtx<'static> {
    let mut cctx = zstd::CCtx::create();
    cctx.set_parameter(zstd::CParameter::Format(zstd::FrameFormat::Magicless))
        .unwrap();
    cctx.set_parameter(zstd::CParameter::CompressionLevel(2))
        .unwrap();
    cctx
}

pub struct Writer<'save, 'guard> {
    voxel_nodes: redb::Table<'save, 'guard, u128, &'static [u8]>,
    entity_nodes: redb::Table<'save, 'guard, u128, &'static [u8]>,
    characters: redb::Table<'save, 'guard, &'static str, &'static [u8]>,
    cctx: zstd::CCtx<'static>,
    plain: Vec<u8>,
    compressed: Vec<u8>,
}

impl Writer<'_, '_> {
    pub fn put_voxel_node(&mut self, node_id: u128, state: &VoxelNode) -> Result<(), DbError> {
        prepare(&mut self.cctx, &mut self.plain, &mut self.compressed, state);
        self.voxel_nodes.insert(node_id, &*self.compressed)?;
        Ok(())
    }

    pub fn put_entity_node(&mut self, node_id: u128, state: &EntityNode) -> Result<(), DbError> {
        prepare(&mut self.cctx, &mut self.plain, &mut self.compressed, state);
        self.entity_nodes.insert(node_id, &*self.compressed)?;
        Ok(())
    }

    pub fn put_character(&mut self, name: &str, character: &Character) -> Result<(), DbError> {
        prepare(
            &mut self.cctx,
            &mut self.plain,
            &mut self.compressed,
            character,
        );
        self.characters.insert(name, &*self.compressed)?;
        Ok(())
    }
}

/// Buffer the compressed, encoded form of `msg` in `compressed`
fn prepare<T: prost::Message>(
    cctx: &mut zstd::CCtx<'_>,
    plain: &mut Vec<u8>,
    compressed: &mut Vec<u8>,
    msg: &T,
) {
    plain.clear();
    msg.encode(plain).unwrap();
    compressed.clear();
    compressed.reserve(zstd::compress_bound(plain.len()));
    cctx.compress2(compressed, plain)
        .map_err(zstd::get_error_name)
        .unwrap();
}

const META_TABLE: TableDefinition<&[u8], &[u8]> = TableDefinition::new("meta");
const VOXEL_NODE_TABLE: TableDefinition<u128, &[u8]> = TableDefinition::new("voxel nodes");
const ENTITY_NODE_TABLE: TableDefinition<u128, &[u8]> = TableDefinition::new("entity nodes");
const CHARACTERS_BY_NAME_TABLE: TableDefinition<&str, &[u8]> =
    TableDefinition::new("characters by name");

#[derive(Debug, Error)]
pub enum OpenError {
    #[error(transparent)]
    Db(#[from] DbError),
    #[error("missing metadata")]
    MissingMeta,
    #[error("decompression failed: {0}")]
    DecompressionFailed(&'static str),
    #[error(transparent)]
    Corrupt(#[from] prost::DecodeError),
}

impl From<redb::Error> for OpenError {
    fn from(x: redb::Error) -> Self {
        OpenError::Db(DbError(x))
    }
}

#[derive(Debug, Error)]
pub enum GetError {
    #[error(transparent)]
    Db(#[from] DbError),
    #[error("decompression failed: {0}")]
    DecompressionFailed(&'static str),
    #[error(transparent)]
    Corrupt(#[from] prost::DecodeError),
}

impl From<redb::Error> for GetError {
    fn from(x: redb::Error) -> Self {
        GetError::Db(DbError(x))
    }
}

#[derive(Debug, Error)]
#[error(transparent)]
pub struct DbError(#[from] redb::Error);
