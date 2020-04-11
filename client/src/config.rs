use std::{
    fs, io,
    net::SocketAddr,
    path::{Path, PathBuf},
    sync::Arc,
};

use serde::Deserialize;
use tracing::{debug, error, info};

use common::{SimConfig, SimConfigRaw};

pub struct Config {
    pub name: Arc<str>,
    pub data_dir: PathBuf,
    pub chunks_loaded_per_frame: u32,
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
            local_simulation,
            chunks_loaded_per_frame,
            server,
        } = match fs::read(&path) {
            Ok(data) => match toml::from_slice(&data) {
                Ok(x) => x,
                Err(e) => {
                    error!("failed to parse config: {}", e);
                    RawConfig::default()
                }
            },
            Err(ref e) if e.kind() == io::ErrorKind::NotFound => {
                info!("{} not found, using defaults", path.display());
                RawConfig::default()
            }
            Err(e) => {
                error!("failed to read config: {}: {}", path.display(), e);
                RawConfig::default()
            }
        };
        // Massage into final form
        Config {
            name: name.unwrap_or_else(|| whoami::user().into()),
            data_dir: data_dir.unwrap_or_else(|| dirs.data_dir().into()),
            chunks_loaded_per_frame: chunks_loaded_per_frame.unwrap_or(16),
            server,
            local_simulation: SimConfig::from_raw(&local_simulation),
        }
    }

    pub fn find_asset(&self, path: &Path) -> Option<PathBuf> {
        #[cfg(feature = "use-repo-assets")]
        {
            let repo_path = Path::new(env!("CARGO_MANIFEST_DIR"))
                .parent()
                .unwrap()
                .join("assets")
                .join(path);
            if repo_path.exists() {
                return Some(repo_path);
            } else {
                debug!(path = %repo_path.display(), "asset not in repo");
            }
        }
        let user_path = self.data_dir.join(path);
        if user_path.exists() {
            return Some(user_path);
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
    chunks_loaded_per_frame: Option<u32>,
    server: Option<SocketAddr>,
    #[serde(default)]
    local_simulation: SimConfigRaw,
}
