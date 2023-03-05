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
    pub fn open(path: &Path) -> Result<Self, OpenError> {
        let db = Database::open(path)?;
        let meta = {
            let tx = db.begin_read()?;
            let meta = tx.open_table(META_TABLE)?;
            let Some(value) = meta.get(&[][..])? else { return Err(OpenError::MissingMeta); };
            proto::Meta::decode(value.value())?
        };
        Ok(Self { meta, db })
    }

    #[inline]
    pub fn meta(&self) -> &Meta {
        &self.meta
    }

    pub fn get(&self, node_id: u128) -> Result<Option<Node>, GetError> {
        let tx = self.db.begin_read()?;
        let nodes = tx.open_table(NODE_TABLE)?;
        let Some(node) = nodes.get(&node_id)? else { return Ok(None); };
        Ok(Some(Node::decode(node.value())?))
    }

    pub fn put(&self, node_id: u128, node_state: Node) -> Result<(), redb::Error> {
        let msg = node_state.encode_to_vec();
        let tx = self.db.begin_write()?;
        let mut nodes = tx.open_table(NODE_TABLE)?;
        nodes.insert(node_id, &*msg)?;
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
