use std::{
    env, fs, io,
    net::SocketAddr,
    path::{Path, PathBuf},
    sync::Arc,
};

use serde::Deserialize;
use tracing::{debug, error, info};

use common::{Anonymize, SimConfig, SimConfigRaw};

pub struct Config {
    pub name: Arc<str>,
    pub data_dirs: Vec<PathBuf>,
    pub save: PathBuf,
    pub chunk_load_parallelism: u32,
    pub server: Option<SocketAddr>,
    pub local_simulation: SimConfig,
}

impl Config {
    pub fn load(dirs: &directories::ProjectDirs) -> Self {
        // Future work: search $XDG_CONFIG_DIRS
        let path = dirs.config_dir().join("client.toml");
        // Read and parse config file
        let RawConfig {
            name,
            data_dir,
            save,
            local_simulation,
            chunk_load_parallelism,
            server,
        } = match fs::read(&path) {
            Ok(data) => {
                info!("found config at {}", path.anonymize().display());
                match std::str::from_utf8(&data)
                    .map_err(anyhow::Error::from)
                    .and_then(|s| toml::from_str(s).map_err(anyhow::Error::from))
                {
                    Ok(x) => x,
                    Err(e) => {
                        error!("failed to parse config: {}", e);
                        RawConfig::default()
                    }
                }
            }
            Err(ref e) if e.kind() == io::ErrorKind::NotFound => {
                info!("{} not found, using defaults", path.anonymize().display());
                RawConfig::default()
            }
            Err(e) => {
                error!(
                    "failed to read config: {}: {}",
                    path.anonymize().display(),
                    e
                );
                RawConfig::default()
            }
        };
        let mut data_dirs = Vec::new();
        if let Some(dir) = data_dir {
            data_dirs.push(dir);
        }
        data_dirs.push(dirs.data_dir().into());
        if let Ok(path) = env::current_exe()
            && let Some(dir) = path.parent()
        {
            data_dirs.push(dir.into());
        }
        #[cfg(feature = "use-repo-assets")]
        {
            data_dirs.push(
                Path::new(env!("CARGO_MANIFEST_DIR"))
                    .parent()
                    .unwrap()
                    .join("assets"),
            );
        }
        // Massage into final form
        Config {
            name: name.unwrap_or("player".into()),
            data_dirs,
            save: save.unwrap_or("default.save".into()),
            chunk_load_parallelism: chunk_load_parallelism.unwrap_or(256),
            server,
            local_simulation: SimConfig::from_raw(&local_simulation),
        }
    }

    pub fn find_asset(&self, path: &Path) -> Option<PathBuf> {
        for dir in &self.data_dirs {
            let full_path = dir.join(path);
            if full_path.exists() {
                debug!(path = ?path.anonymize().display(), dir = ?dir.anonymize().display(), "found asset");
                return Some(full_path);
            }
        }
        None
    }
}

/// Data as parsed directly out of the config file
#[derive(Deserialize, Default)]
#[serde(deny_unknown_fields)]
struct RawConfig {
    name: Option<Arc<str>>,
    data_dir: Option<PathBuf>,
    save: Option<PathBuf>,
    chunk_load_parallelism: Option<u32>,
    server: Option<SocketAddr>,
    #[serde(default)]
    local_simulation: SimConfigRaw,
}
