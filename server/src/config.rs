use std::{
    fs,
    net::SocketAddr,
    path::{Path, PathBuf},
};

use anyhow::{Context, Result};
use serde::Deserialize;

#[derive(Deserialize, Default)]
pub struct Config {
    pub server_name: Option<String>,
    pub certificate_chain: Option<PathBuf>,
    pub private_key: Option<PathBuf>,
    pub listen: Option<SocketAddr>,
}

impl Config {
    pub fn load(path: &Path) -> Result<Self> {
        Ok(
            toml::from_slice(&fs::read(path).context("reading config file")?)
                .context("parsing config file")?,
        )
    }
}
