use std::{
    fs,
    net::{Ipv6Addr, SocketAddr},
    path::{Path, PathBuf},
};

use anyhow::{Context, Result};
use serde::Deserialize;
use server::SimConfig;

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Config {
    pub server_name: Option<String>,
    pub certificate_chain: Option<PathBuf>,
    pub private_key: Option<PathBuf>,
    pub listen: SocketAddr,
    pub simulation: SimConfig,
}

impl Config {
    pub fn load(path: &Path) -> Result<Self> {
        Ok(
            toml::from_slice(&fs::read(path).context("reading config file")?)
                .context("parsing config file")?,
        )
    }
}

impl Default for Config {
    fn default() -> Self {
        Self {
            server_name: None,
            certificate_chain: None,
            private_key: None,
            listen: SocketAddr::new(Ipv6Addr::UNSPECIFIED.into(), 1234),
            simulation: SimConfig::default(),
        }
    }
}
