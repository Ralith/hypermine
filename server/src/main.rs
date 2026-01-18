#![allow(clippy::needless_borrowed_reference)]

mod config;

use std::{fs, net::UdpSocket, path::Path};

use anyhow::{Context, Result, anyhow};
use quinn::rustls::pki_types::{CertificateDer, PrivateKeyDer, PrivatePkcs8KeyDer};
use tracing::{info, warn};

use common::SimConfig;
use config::Config;
use save::Save;

fn main() {
    // Set up logging
    common::init_tracing();

    if let Err(e) = run() {
        eprintln!("{e:#}");
        std::process::exit(1);
    }
}

#[tokio::main]
pub async fn run() -> Result<()> {
    let cfg = match std::env::args_os().nth(1) {
        Some(path) => Config::load(Path::new(&path))?,
        None => Config::default(),
    };

    let (certificate_chain, private_key) = match (&cfg.certificate_chain, &cfg.private_key) {
        (&Some(ref certificate_chain), &Some(ref private_key)) => (
            rustls_pemfile::certs(
                &mut &*fs::read(certificate_chain).context("reading certificate chain")?,
            )
            .collect::<Result<Vec<_>, _>>()
            .context("parsing certificate chain")?,
            rustls_pemfile::pkcs8_private_keys(
                &mut &*fs::read(private_key).context("reading private key")?,
            )
            .next()
            .ok_or_else(|| anyhow!("no private key found with PKCS #8 format"))?
            .context("parsing private key")?
            .into(),
        ),
        _ => {
            // TODO: Cache on disk
            warn!("generating self-signed certificate");
            let certified_key = rcgen::generate_simple_self_signed(vec![
                cfg.server_name.clone().map(Ok).unwrap_or_else(|| {
                    hostname::get().context("getting hostname").and_then(|x| {
                        x.into_string()
                            .map_err(|_| anyhow!("hostname is not valid UTF-8"))
                    })
                })?,
            ])
            .unwrap();
            let key = certified_key.signing_key.serialize_der();
            let cert = certified_key.cert.der().to_vec();
            (
                vec![CertificateDer::from(cert)],
                PrivateKeyDer::from(PrivatePkcs8KeyDer::from(key)),
            )
        }
    };

    let sim_cfg = SimConfig::from_raw(&cfg.simulation);

    let save = cfg.save.unwrap_or_else(|| "hypermine.save".into());
    info!("using save file {}", save.display());
    let save = Save::open(&save, sim_cfg.chunk_size)?;

    let server = server::Server::new(
        Some(server::NetParams {
            certificate_chain,
            private_key,
            socket: UdpSocket::bind(cfg.listen).context("binding socket")?,
        }),
        sim_cfg,
        save,
    )?;
    server.run().await;
    Ok(())
}
