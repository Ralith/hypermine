mod config;

use std::{fs, net::UdpSocket, path::Path};

use anyhow::{anyhow, Context, Result};
use quinn::{Certificate, CertificateChain, PrivateKey};
use tracing::warn;

use config::Config;

fn main() {
    // Set up logging
    tracing_subscriber::fmt::init();

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
            CertificateChain::from_pem(
                &fs::read(certificate_chain).context("reading certificate chain")?,
            )
            .context("parsing certificate chain")?,
            PrivateKey::from_pem(&fs::read(private_key).context("reading private key")?)
                .context("parsing private key")?,
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
            (
                CertificateChain::from_certs(Certificate::from_der(&cert)),
                PrivateKey::from_der(&key).unwrap(),
            )
        }
    };

    server::run(
        server::NetParams {
            certificate_chain,
            private_key,
            socket: UdpSocket::bind(&cfg.listen).context("binding socket")?,
        },
        server::SimConfig {
            rate: cfg.rate,
            view_distance: cfg.view_distance,
        },
    )
}
