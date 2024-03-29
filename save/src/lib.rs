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
        let db = Database::create(path).map_err(redb::Error::from)?;
        let meta = {
            let tx = db.begin_read().map_err(redb::Error::from)?;
            // Intermediate variable to make borrowck happy
            match tx.open_table(META_TABLE) {
                Ok(meta) => {
                    let Some(value) = meta.get(&[][..]).map_err(redb::Error::from)? else {
                        return Err(OpenError::MissingMeta);
                    };
                    let mut dctx = dctx();
                    let mut buffer = Vec::new();
                    decompress(&mut dctx, value.value(), &mut buffer)
                        .map_err(OpenError::DecompressionFailed)?;
                    Meta::decode(&*buffer)?
                }
                Err(redb::TableError::TableDoesNotExist(_)) => {
                    // Must be an empty save file. Initialize the meta record and create the other tables.
                    let defaults = Meta {
                        chunk_size: default_chunk_size.into(),
                    };
                    init_meta_table(&db, &defaults)?;
                    defaults
                }
                Err(e) => return Err(OpenError::Db(DbError(e.into()))),
            }
        };
        Ok(Self { meta, db })
    }

    #[inline]
    pub fn meta(&self) -> &Meta {
        &self.meta
    }

    pub fn read(&self) -> Result<Reader, DbError> {
        let tx = self.db.begin_read().map_err(redb::Error::from)?;
        Ok(Reader {
            voxel_nodes: tx.open_table(VOXEL_NODE_TABLE)?,
            entity_nodes: tx.open_table(ENTITY_NODE_TABLE)?,
            characters: tx.open_table(CHARACTERS_BY_NAME_TABLE)?,
            dctx: dctx(),
            accum: Vec::new(),
        })
    }

    pub fn write(&self) -> Result<WriterGuard, DbError> {
        let tx = self.db.begin_write().map_err(redb::Error::from)?;
        Ok(WriterGuard { tx })
    }
}

fn init_meta_table(db: &Database, value: &Meta) -> Result<(), redb::Error> {
    let tx = db.begin_write()?;
    let mut meta = tx.open_table(META_TABLE)?;
    let mut cctx = cctx();
    let mut plain = Vec::new();
    let mut compressed = Vec::new();
    prepare(&mut cctx, &mut plain, &mut compressed, value);
    meta.insert(&[][..], &*compressed)?;
    drop(meta);

    tx.open_table(VOXEL_NODE_TABLE)?;
    tx.open_table(ENTITY_NODE_TABLE)?;
    tx.open_table(CHARACTERS_BY_NAME_TABLE)?;
    tx.commit()?;
    Ok(())
}

fn dctx() -> zstd::DCtx<'static> {
    let mut dctx = zstd::DCtx::create();
    dctx.set_parameter(zstd::DParameter::Format(zstd::FrameFormat::Magicless))
        .unwrap();
    dctx
}

pub struct Reader {
    voxel_nodes: redb::ReadOnlyTable<u128, &'static [u8]>,
    entity_nodes: redb::ReadOnlyTable<u128, &'static [u8]>,
    characters: redb::ReadOnlyTable<&'static str, &'static [u8]>,
    dctx: zstd::DCtx<'static>,
    accum: Vec<u8>,
}

impl Reader {
    pub fn get_voxel_node(&mut self, node_id: u128) -> Result<Option<VoxelNode>, GetError> {
        let Some(node) = self.voxel_nodes.get(&node_id)? else {
            return Ok(None);
        };
        self.accum.clear();
        decompress(&mut self.dctx, node.value(), &mut self.accum)
            .map_err(GetError::DecompressionFailed)?;
        Ok(Some(VoxelNode::decode(&*self.accum)?))
    }

    pub fn get_entity_node(&mut self, node_id: u128) -> Result<Option<EntityNode>, GetError> {
        let Some(node) = self.entity_nodes.get(&node_id)? else {
            return Ok(None);
        };
        self.accum.clear();
        decompress(&mut self.dctx, node.value(), &mut self.accum)
            .map_err(GetError::DecompressionFailed)?;
        Ok(Some(EntityNode::decode(&*self.accum)?))
    }

    pub fn get_character(&mut self, name: &str) -> Result<Option<Character>, GetError> {
        let Some(node) = self.characters.get(name)? else {
            return Ok(None);
        };
        self.accum.clear();
        decompress(&mut self.dctx, node.value(), &mut self.accum)
            .map_err(GetError::DecompressionFailed)?;
        Ok(Some(Character::decode(&*self.accum)?))
    }

    /// Temporary function to load all voxel-related save data at once.
    /// TODO: Replace this implementation with a streaming implementation
    /// that does not require loading everything at once
    pub fn get_all_voxel_node_ids(&self) -> Result<Vec<u128>, GetError> {
        self.voxel_nodes
            .iter()?
            .map(|n| Ok(n.map_err(GetError::from)?.0.value()))
            .collect()
    }
}

fn decompress(
    dctx: &mut zstd::DCtx<'_>,
    compressed: &[u8],
    out: &mut Vec<u8>,
) -> Result<(), &'static str> {
    dctx.init().map_err(zstd::get_error_name)?;
    let mut input = zstd::InBuffer::around(compressed);
    let out_size = zstd::DCtx::out_size();
    loop {
        if out.len() + out_size > out.capacity() {
            out.reserve(out_size);
        }
        let mut out = zstd::OutBuffer::around_pos(out, out.len());
        let n = dctx
            .decompress_stream(&mut out, &mut input)
            .map_err(zstd::get_error_name)?;
        if n == 0 {
            return Ok(());
        }
    }
}

pub struct WriterGuard {
    tx: redb::WriteTransaction,
}

impl WriterGuard {
    pub fn get(&mut self) -> Result<Writer<'_>, DbError> {
        Ok(Writer {
            voxel_nodes: self
                .tx
                .open_table(VOXEL_NODE_TABLE)
                .map_err(redb::Error::from)?,
            entity_nodes: self
                .tx
                .open_table(ENTITY_NODE_TABLE)
                .map_err(redb::Error::from)?,
            characters: self
                .tx
                .open_table(CHARACTERS_BY_NAME_TABLE)
                .map_err(redb::Error::from)?,
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

pub struct Writer<'guard> {
    voxel_nodes: redb::Table<'guard, u128, &'static [u8]>,
    entity_nodes: redb::Table<'guard, u128, &'static [u8]>,
    characters: redb::Table<'guard, &'static str, &'static [u8]>,
    cctx: zstd::CCtx<'static>,
    plain: Vec<u8>,
    compressed: Vec<u8>,
}

impl Writer<'_> {
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

impl From<redb::StorageError> for GetError {
    fn from(x: redb::StorageError) -> Self {
        GetError::Db(DbError(x.into()))
    }
}

#[derive(Debug, Error)]
#[error(transparent)]
pub struct DbError(#[from] redb::Error);

impl From<redb::StorageError> for DbError {
    fn from(x: redb::StorageError) -> Self {
        DbError(x.into())
    }
}

impl From<redb::CommitError> for DbError {
    fn from(x: redb::CommitError) -> Self {
        DbError(x.into())
    }
}

impl From<redb::TableError> for DbError {
    fn from(x: redb::TableError) -> Self {
        DbError(x.into())
    }
}
