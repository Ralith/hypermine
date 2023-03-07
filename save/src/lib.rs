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
                    Meta::decode(value.value())?
                }
                Err(redb::Error::TableDoesNotExist(_)) => {
                    let defaults = Meta {
                        chunk_size: default_chunk_size.into(),
                    };
                    let tx = db.begin_write()?;
                    let mut meta = tx.open_table(META_TABLE)?;
                    meta.insert(&[][..], &*defaults.encode_to_vec())?;
                    drop(meta);
                    // Create the node table so `read` doesn't have to handle its possible absence
                    tx.open_table(NODE_TABLE)?;
                    tx.commit()?;
                    defaults.clone()
                }
                Err(e) => return Err(e.into()),
            }
        };
        Ok(Self { meta, db })
    }

    #[inline]
    pub fn meta(&self) -> &Meta {
        &self.meta
    }

    pub fn read(&self) -> Result<ReaderGuard<'_>, redb::Error> {
        let tx = self.db.begin_read()?;
        Ok(ReaderGuard { tx })
    }

    pub fn write(&mut self) -> Result<WriterGuard<'_>, redb::Error> {
        let tx = self.db.begin_write()?;
        Ok(WriterGuard { tx })
    }
}

pub struct ReaderGuard<'a> {
    tx: redb::ReadTransaction<'a>,
}

impl ReaderGuard<'_> {
    pub fn get(&self) -> Result<Reader<'_>, redb::Error> {
        let table = self.tx.open_table(NODE_TABLE)?;
        Ok(Reader { table })
    }
}

pub struct Reader<'a> {
    table: redb::ReadOnlyTable<'a, u128, &'static [u8]>,
}

impl Reader<'_> {
    pub fn get(&mut self, node_id: u128) -> Result<Option<Node>, GetError> {
        let Some(node) = self.table.get(&node_id)? else { return Ok(None); };
        Ok(Some(Node::decode(node.value())?))
    }
}

pub struct WriterGuard<'a> {
    tx: redb::WriteTransaction<'a>,
}

impl<'a> WriterGuard<'a> {
    pub fn get(&mut self) -> Result<Writer<'a, '_>, redb::Error> {
        Ok(Writer {
            table: self.tx.open_table(NODE_TABLE)?,
            plain: Vec::new(),
        })
    }

    pub fn commit(self) -> Result<(), redb::Error> {
        self.tx.commit()
    }
}

pub struct Writer<'save, 'guard> {
    table: redb::Table<'save, 'guard, u128, &'static [u8]>,
    plain: Vec<u8>,
}

impl Writer<'_, '_> {
    pub fn put(&mut self, node_id: u128, state: &Node) -> Result<(), redb::Error> {
        self.plain.clear();
        state.encode(&mut self.plain).unwrap();
        self.table.insert(node_id, &*self.plain)?;
        Ok(())
    }
}

const META_TABLE: TableDefinition<&[u8], &[u8]> = TableDefinition::new("meta");
const NODE_TABLE: TableDefinition<u128, &[u8]> = TableDefinition::new("nodes");

#[derive(Debug, Error)]
pub enum OpenError {
    #[error(transparent)]
    Db(#[from] redb::Error),
    #[error("missing metadata")]
    MissingMeta,
    #[error(transparent)]
    Corrupt(#[from] prost::DecodeError),
}

#[derive(Debug, Error)]
pub enum GetError {
    #[error(transparent)]
    Db(#[from] redb::Error),
    #[error(transparent)]
    Corrupt(#[from] prost::DecodeError),
}
