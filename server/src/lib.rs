mod input_queue;
mod sim;

use std::{
    net::UdpSocket,
    sync::Arc,
    time::{Duration, Instant},
};

use anyhow::{Context, Error, Result};
use futures::{select, StreamExt, TryStreamExt};
use hecs::Entity;
use slotmap::DenseSlotMap;
use tokio::sync::mpsc;
use tracing::{debug, error, error_span, info, trace};

use common::{codec, proto, SimConfig};
use input_queue::InputQueue;
use sim::Sim;

pub struct NetParams {
    pub certificate_chain: quinn::CertificateChain,
    pub private_key: quinn::PrivateKey,
    pub socket: UdpSocket,
}

#[tokio::main]
pub async fn run(net: NetParams, sim: SimConfig) -> Result<()> {
    let mut server_config = quinn::ServerConfigBuilder::default();
    server_config
        .certificate(net.certificate_chain, net.private_key)
        .context("parsing certificate")?;
    let mut endpoint = quinn::Endpoint::builder();
    endpoint.listen(server_config.build());
    let (endpoint, incoming) = endpoint.with_socket(net.socket)?;
    info!(address = %endpoint.local_addr().unwrap(), "listening");

    let server = Server::new(sim);
    server.run(incoming).await;
    Ok(())
}

struct Server {
    cfg: Arc<SimConfig>,
    sim: Sim,
    clients: DenseSlotMap<ClientId, Client>,
}

impl Server {
    fn new(params: SimConfig) -> Self {
        let cfg = Arc::new(params);
        Self {
            sim: Sim::new(cfg.clone()),
            cfg,
            clients: DenseSlotMap::default(),
        }
    }

    async fn run(mut self, incoming: quinn::Incoming) {
        let mut ticks = tokio::time::interval(Duration::from_secs(1) / self.cfg.rate as u32).fuse();
        let mut incoming = incoming
            .inspect(|x| trace!(address = %x.remote_address(), "connection incoming"))
            .buffer_unordered(16);
        let (client_events_send, client_events) = mpsc::channel(128);
        let mut client_events = client_events.fuse();
        loop {
            select! {
                _ = ticks.next() => self.on_step(),
                conn = incoming.select_next_some() => {self.on_connect(conn, client_events_send.clone()); }
                e = client_events.select_next_some() => { self.on_client_event(e.0, e.1); }
            }
        }
    }

    fn on_step(&mut self) {
        let now = Instant::now();
        // Apply queued inputs
        for (id, client) in &mut self.clients {
            if let Some(ref handles) = client.handles {
                if let Some(cmd) = client.inputs.pop(now, self.cfg.input_queue_size) {
                    client.latest_input_processed = cmd.generation;
                    if let Err(e) = self.sim.command(handles.character, cmd) {
                        error!(client = ?id, "couldn't process command: {}", e);
                    }
                }
            }
        }

        // Step the simulation
        let (spawns, delta) = self.sim.step();
        let spawns = Arc::new(spawns);
        let mut overran = Vec::new();
        for (client_id, client) in &mut self.clients {
            if let Some(ref mut handles) = client.handles {
                let mut delta = delta.clone();
                delta.latest_input = client.latest_input_processed;
                let r1 = handles.unordered.try_send(delta);
                let r2 = if !spawns.spawns.is_empty()
                    || !spawns.despawns.is_empty()
                    || !spawns.nodes.is_empty()
                {
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
        let span = error_span!("client", id = ?client_id.0);
        let _guard = span.enter();
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
                let server_hello = proto::ServerHello {
                    character: id,
                    rate: self.cfg.rate,
                    chunk_size: self.cfg.chunk_size,
                    meters_to_absolute: self.cfg.meters_to_absolute,
                    movement_speed: self.cfg.movement_speed,
                    gravity_intensity: self.cfg.gravity_intensity,
                    drag_factor: self.cfg.drag_factor,
                    gravity_method: self.cfg.gravity_method,
                };
                tokio::spawn(async move {
                    // Errors will be handled by recv task
                    let _ =
                        drive_send(connection, server_hello, unordered_recv, ordered_recv).await;
                });
            }
            ClientEvent::Lost(e) => {
                error!("lost: {:#}", e);
                client.conn.close(0u32.into(), b"");
                self.cleanup_client(client_id);
            }
            ClientEvent::Command(cmd) => {
                if cmd.generation.wrapping_sub(client.latest_input_received) < u16::max_value() / 2
                {
                    client.latest_input_received = cmd.generation;
                    client.inputs.push(cmd, Instant::now());
                } else {
                    debug!("dropping obsolete command");
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
        let id = self.clients.insert(Client::new(connection.clone()));
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
        .map(|stream| async {
            Ok::<_, Error>(codec::recv_whole::<proto::Command>(MAX_CLIENT_MSG_SIZE, stream?).await?)
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
    latest_input_received: u16,
    latest_input_processed: u16,
    inputs: InputQueue,
}

impl Client {
    fn new(conn: quinn::Connection) -> Self {
        Self {
            conn,
            handles: None,
            latest_input_received: 0,
            latest_input_processed: 0,
            inputs: InputQueue::new(),
        }
    }
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

type Unordered = proto::StateDelta;

type Ordered = Arc<proto::Spawns>;
