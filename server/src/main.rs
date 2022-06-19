mod config;

use std::{fs, net::UdpSocket, path::Path};

use anyhow::{anyhow, Context, Result};
use tracing::warn;

use common::SimConfig;
use config::Config;

fn main() {
    // Set up logging
    common::init_tracing();

    if let Err(e) = run() {
        eprintln!("{:#}", e);
        std::process::exit(1);
    }
}

pub fn run() -> Result<()> {
    let cfg = match std::env::args_os().nth(1) {
        Some(path) => Config::load(Path::new(&path))?,
        None => Config::default(),
    };

    let (certificate_chain, private_key) = match (&cfg.certificate_chain, &cfg.private_key) {
        (&Some(ref certificate_chain), &Some(ref private_key)) => (
            rustls_pemfile::certs(
                &mut &*fs::read(certificate_chain).context("reading certificate chain")?,
            )
            .context("parsing certificate chain")?
            .into_iter()
            .map(rustls::Certificate)
            .collect(),
            rustls_pemfile::pkcs8_private_keys(
                &mut &*fs::read(private_key).context("reading private key")?,
            )
            .context("parsing private key")?
            .into_iter()
            .map(rustls::PrivateKey)
            .next()
            .ok_or_else(|| anyhow!("No private keys found"))?,
        ),
        _ => {
            // TODO: Cache on disk
            warn!("generating self-signed certificate");
            let cert = rcgen::generate_simple_self_signed(vec![cfg
                .server_name
                .clone()
                .map(Ok)
                .unwrap_or_else(|| {
                    hostname::get().context("getting hostname").and_then(|x| {
                        x.into_string()
                            .map_err(|_| anyhow!("hostname is not valid UTF-8"))
                    })
                })?])
            .unwrap();
            let key = cert.serialize_private_key_der();
            let cert = cert.serialize_der().unwrap();
            (vec![rustls::Certificate(cert)], rustls::PrivateKey(key))
        }
    };

    server::run(
        server::NetParams {
            certificate_chain,
            private_key,
            socket: UdpSocket::bind(&cfg.listen).context("binding socket")?,
        },
        SimConfig::from_raw(&cfg.simulation),
    )
}
