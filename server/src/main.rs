mod config;

use std::{
    fs,
    net::{Ipv6Addr, SocketAddr},
    path::Path,
};

use anyhow::{anyhow, Context, Result};
use futures::StreamExt;
use quinn::{Certificate, CertificateChain, PrivateKey};
use tracing::{error, info, warn};

use config::Config;

fn main() {
    // Set up logging
    tracing_subscriber::fmt::init();

    if let Err(e) = run() {
        eprintln!("{:#}", e);
        std::process::exit(1);
    }
}

#[tokio::main]
async fn run() -> Result<()> {
    let cfg = match std::env::args_os().nth(1) {
        Some(path) => Config::load(Path::new(&path))?,
        None => Config::default(),
    };

    let (certs, pkey) = match (&cfg.certificate_chain, &cfg.private_key) {
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
                .map(|x| Ok(x.into()))
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
    let mut server_config = quinn::ServerConfigBuilder::default();
    server_config
        .certificate(certs, pkey)
        .context("parsing certificate")?;
    let mut endpoint = quinn::Endpoint::builder();
    endpoint.listen(server_config.build());
    let (endpoint, mut incoming) = endpoint.bind(
        &cfg.listen
            .unwrap_or_else(|| SocketAddr::new(Ipv6Addr::UNSPECIFIED.into(), 1234)),
    )?;
    info!(address = %endpoint.local_addr().unwrap(), "listening");

    loop {
        let conn = incoming.next().await.unwrap();
        tokio::spawn(async {
            if let Err(e) = handle_conn(conn).await {
                error!("connection lost: {:#}", e);
            }
        });
    }
}

async fn handle_conn(conn: quinn::Connecting) -> Result<()> {
    conn.await?;
    Ok(())
}
