mod config;
mod sim;

use std::{fs, path::Path, sync::Arc, time::Duration};

use anyhow::{anyhow, Context, Error, Result};
use futures::{select, StreamExt, TryStreamExt};
use hecs::Entity;
use quinn::{Certificate, CertificateChain, PrivateKey};
use slotmap::DenseSlotMap;
use tokio::sync::mpsc;
use tracing::{error, info, trace, warn};

use common::{codec, proto};
use config::Config;
use sim::Sim;

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
    let mut server_config = quinn::ServerConfigBuilder::default();
    server_config
        .certificate(certs, pkey)
        .context("parsing certificate")?;
    let mut endpoint = quinn::Endpoint::builder();
    endpoint.listen(server_config.build());
    let (endpoint, incoming) = endpoint.bind(&cfg.listen)?;
    info!(address = %endpoint.local_addr().unwrap(), "listening");

    let server = Server::new(cfg);
    server.run(incoming).await;
    Ok(())
}

struct Server {
    cfg: Config,
    sim: Sim,
    clients: DenseSlotMap<ClientId, Client>,
}

impl Server {
    fn new(cfg: Config) -> Self {
        Self {
            cfg,
            sim: Sim::new(),
            clients: DenseSlotMap::default(),
        }
    }

    async fn run(mut self, incoming: quinn::Incoming) {
        let mut ticks =
            tokio::time::interval(Duration::from_secs_f64(1.0 / self.cfg.rate as f64)).fuse();
        let mut incoming = incoming
            .inspect(|x| trace!(address = %x.remote_address(), "connection incoming"))
            .buffer_unordered(16);
        let (client_events_send, client_events) = mpsc::channel(128);
        let mut client_events = client_events.fuse();
        loop {
            select! {
                _ = ticks.next() => { self.on_step() }
                conn = incoming.select_next_some() => { self.on_connect(conn, client_events_send.clone()); }
                e = client_events.select_next_some() => { self.on_client_event(e.0, e.1); }
            }
        }
    }

    fn on_step(&mut self) {
        let (spawns, delta) = self.sim.step();
        let spawns = Arc::new(spawns);
        let delta = Arc::new(delta);
        let mut overran = Vec::new();
        for (client_id, client) in &mut self.clients {
            if let Some(ref mut handles) = client.handles {
                let r1 = handles.unordered.try_send(delta.clone());
                let r2 = if !spawns.spawns.is_empty() || !spawns.despawns.is_empty() {
                    handles.ordered.try_send(spawns.clone())
                } else {
                    Ok(())
                };
                use mpsc::error::TrySendError::Full;
                match (r1, r2) {
                    (Err(Full(_)), _) | (_, Err(Full(_))) => {
                        overran.push(client_id);
                    }
                    _ => {}
                }
            }
        }
        for client_id in overran {
            error!("dropping slow client {:?}", client_id.0);
            self.clients[client_id]
                .conn
                .close(1u32.into(), b"client reading too slowly");
            self.cleanup_client(client_id);
        }
    }

    fn on_client_event(&mut self, client_id: ClientId, event: ClientEvent) {
        let client = &mut self.clients[client_id];
        match event {
            ClientEvent::Hello(hello) => {
                assert!(client.handles.is_none());
                let snapshot = Arc::new(self.sim.snapshot());
                let (id, entity) = self.sim.spawn_character(hello);
                let (mut ordered_send, ordered_recv) = mpsc::channel(32);
                ordered_send.try_send(snapshot).unwrap();
                let (unordered_send, unordered_recv) = mpsc::channel(32);
                client.handles = Some(ClientHandles {
                    character: entity,
                    ordered: ordered_send,
                    unordered: unordered_send,
                });
                let connection = client.conn.clone();
                let server_hello = proto::ServerHello { character: id };
                tokio::spawn(async move {
                    // Errors will be handled by recv task
                    let _ =
                        drive_send(connection, server_hello, unordered_recv, ordered_recv).await;
                });
            }
            ClientEvent::Lost(e) => {
                error!("client lost: {:#}", e);
                client.conn.close(0u32.into(), b"");
                self.cleanup_client(client_id);
            }
            ClientEvent::Command(cmd) => {
                if let Some(ref x) = client.handles {
                    self.sim.command(x.character, cmd);
                }
            }
        }
    }

    fn cleanup_client(&mut self, client: ClientId) {
        if let Some(ref x) = self.clients[client].handles {
            self.sim.destroy(x.character);
        }
        self.clients.remove(client);
    }

    fn on_connect(
        &mut self,
        conn: Result<quinn::NewConnection, quinn::ConnectionError>,
        mut send: mpsc::Sender<(ClientId, ClientEvent)>,
    ) {
        let quinn::NewConnection {
            connection,
            uni_streams,
            ..
        } = match conn {
            Ok(x) => x,
            Err(e) => {
                error!("incoming connection failed: {}", e);
                return;
            }
        };
        let id = self.clients.insert(Client {
            conn: connection.clone(),
            handles: None,
        });
        info!(id = ?id.0, address = %connection.remote_address(), "connection established");
        tokio::spawn(async move {
            if let Err(e) = drive_recv(id, uni_streams, &mut send).await {
                let _ = send.send((id, ClientEvent::Lost(e))).await;
            }
        });
    }
}

const MAX_CLIENT_MSG_SIZE: usize = 1 << 16;

async fn drive_recv(
    id: ClientId,
    mut streams: quinn::IncomingUniStreams,
    send: &mut mpsc::Sender<(ClientId, ClientEvent)>,
) -> Result<()> {
    let hello = match streams.next().await {
        None => return Ok(()),
        Some(stream) => {
            codec::recv_whole::<proto::ClientHello>(MAX_CLIENT_MSG_SIZE, stream?).await?
        }
    };
    let _ = send.send((id, ClientEvent::Hello(hello))).await;

    let mut cmds = streams
        .map(|stream| {
            async {
                Ok::<_, Error>(
                    codec::recv_whole::<proto::Command>(MAX_CLIENT_MSG_SIZE, stream?).await?,
                )
            }
        })
        .buffer_unordered(16); // Allow a modest amount of out-of-order completion
    while let Some(msg) = cmds.try_next().await? {
        let _ = send.send((id, ClientEvent::Command(msg))).await;
    }
    Ok(())
}

async fn drive_send(
    conn: quinn::Connection,
    hello: proto::ServerHello,
    unordered: mpsc::Receiver<Unordered>,
    mut ordered: mpsc::Receiver<Ordered>,
) -> Result<()> {
    let mut stream = conn.open_uni().await?;
    codec::send(&mut stream, &hello).await?;

    tokio::spawn(async move {
        // Errors will be handled by recv task
        let _ = drive_send_unordered(conn.clone(), unordered).await;
    });

    while let Some(msg) = ordered.next().await {
        codec::send(&mut stream, &msg).await?;
    }

    Ok(())
}

async fn drive_send_unordered(
    conn: quinn::Connection,
    mut msgs: mpsc::Receiver<Unordered>,
) -> Result<()> {
    while let Some(msg) = msgs.next().await {
        let stream = conn.open_uni().await?;
        codec::send_whole(stream, &msg).await?;
    }
    Ok(())
}

slotmap::new_key_type! {
    struct ClientId;
}

struct Client {
    conn: quinn::Connection,
    /// Filled in after receiving ClientHello
    handles: Option<ClientHandles>,
}

struct ClientHandles {
    character: Entity,
    ordered: mpsc::Sender<Ordered>,
    unordered: mpsc::Sender<Unordered>,
}

enum ClientEvent {
    Hello(proto::ClientHello),
    Command(proto::Command),
    Lost(Error),
}

type Unordered = Arc<proto::StateDelta>;

type Ordered = Arc<proto::Spawns>;
