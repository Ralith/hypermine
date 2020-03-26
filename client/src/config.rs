use std::{
    fs, io,
    net::SocketAddr,
    path::{Path, PathBuf},
    sync::Arc,
};

use serde::Deserialize;
use tracing::{debug, error, info};

pub struct Config {
    pub name: Arc<str>,
    pub data_dir: PathBuf,
    pub view_distance: f64,
    pub chunks_loaded_per_frame: u32,
    pub input_send_rate: u16,
    pub server: Option<SocketAddr>,
}

impl Config {
    pub fn load(dirs: &directories::ProjectDirs) -> Self {
        // Future work: search $XDG_CONFIG_DIRS
        let path = dirs.config_dir().join("client.toml");
        // Read and parse config file
        let RawConfig {
            name,
            data_dir,
            view_distance,
            chunks_loaded_per_frame,
            input_send_rate,
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
            view_distance: view_distance.unwrap_or(6.0),
            chunks_loaded_per_frame: chunks_loaded_per_frame.unwrap_or(16),
            input_send_rate: input_send_rate.unwrap_or(30),
            server,
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
        let user_path = self.data_dir.join("materials");
        if user_path.exists() {
            return Some(user_path);
        }
        None
    }
}

/// Data as parsed directly out of the config file
#[derive(Deserialize, Default)]
struct RawConfig {
    name: Option<Arc<str>>,
    data_dir: Option<PathBuf>,
    view_distance: Option<f64>,
    chunks_loaded_per_frame: Option<u32>,
    input_send_rate: Option<u16>,
    server: Option<SocketAddr>,
}
