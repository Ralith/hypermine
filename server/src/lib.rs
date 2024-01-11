#![allow(clippy::needless_borrowed_reference)]

extern crate nalgebra as na;
mod input_queue;
mod postcard_helpers;
mod sim;

use std::{net::UdpSocket, sync::Arc, time::Instant};

use anyhow::{Context, Error, Result};
use futures::{select, StreamExt};
use hecs::Entity;
use slotmap::DenseSlotMap;
use tokio::sync::mpsc;
use tokio_stream::wrappers::{IntervalStream, ReceiverStream};
use tracing::{debug, error, error_span, info, trace};

use common::{codec, proto, SimConfig};
use input_queue::InputQueue;
use save::Save;
use sim::Sim;

pub struct NetParams {
    pub certificate_chain: Vec<rustls::Certificate>,
    pub private_key: rustls::PrivateKey,
    pub socket: UdpSocket,
}

#[tokio::main]
pub async fn run(net: NetParams, mut sim: SimConfig, save: Save) -> Result<()> {
    sim.chunk_size = save.meta().chunk_size as u8;
    let server_config =
        quinn::ServerConfig::with_single_cert(net.certificate_chain, net.private_key)
            .context("parsing certificate")?;
    let endpoint = quinn::Endpoint::new(
        quinn::EndpointConfig::default(),
        Some(server_config),
        net.socket,
        quinn::default_runtime().unwrap(),
    )?;
    info!(address = %endpoint.local_addr().unwrap(), "listening");

    let server = Server::new(sim, save);
    server.run(endpoint).await;
    Ok(())
}

struct Server {
    cfg: Arc<SimConfig>,
    sim: Sim,
    clients: DenseSlotMap<ClientId, Client>,
    save: Save,
}

impl Server {
    fn new(params: SimConfig, save: Save) -> Self {
        let cfg = Arc::new(params);
        Self {
            sim: Sim::new(cfg.clone(), &save),
            cfg,
            clients: DenseSlotMap::default(),
            save,
        }
    }

    async fn run(mut self, endpoint: quinn::Endpoint) {
        let mut ticks = IntervalStream::new(tokio::time::interval(self.cfg.step_interval)).fuse();
        let mut incoming = ReceiverStream::new(self.handle_incoming(endpoint)).fuse();
        let (client_events_send, client_events) = mpsc::channel(128);
        let mut client_events = ReceiverStream::new(client_events).fuse();
        loop {
            select! {
                _ = ticks.next() => { self.on_step(); },
                conn = incoming.select_next_some() => { self.on_connect(conn, client_events_send.clone()); }
                e = client_events.select_next_some() => { self.on_client_event(e.0, e.1); }
            }
        }
    }

    fn handle_incoming(&self, endpoint: quinn::Endpoint) -> mpsc::Receiver<quinn::Connection> {
        let (incoming_send, incoming_recv) = mpsc::channel(16);
        tokio::spawn(async move {
            while let Some(conn) = endpoint.accept().await {
                trace!(address = %conn.remote_address(), "connection incoming");
                let incoming_send = incoming_send.clone();
                tokio::spawn(async move {
                    match conn.await {
                        Err(e) => {
                            error!("incoming connection failed: {}", e.to_string());
                        }
                        Ok(connection) => {
                            let _ = incoming_send.send(connection).await;
                        }
                    }
                });
            }
        });
        incoming_recv
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
                    || !spawns.block_updates.is_empty()
                    || !spawns.voxel_data.is_empty()
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

        // Save the world. Could be less frequent if it becomes a bottleneck.
        if let Err(e) = self.sim.save(&mut self.save) {
            error!("couldn't save: {}", e);
        }
    }

    fn on_client_event(&mut self, client_id: ClientId, event: ClientEvent) {
        let span = error_span!("client", id = ?client_id.0);
        let _guard = span.enter();
        let Some(client) = self.clients.get_mut(client_id) else {
            // Skip messages from cleaned-up clients
            return;
        };
        match event {
            ClientEvent::Hello(hello) => {
                assert!(client.handles.is_none());
                let snapshot = Arc::new(self.sim.snapshot());
                let (id, entity) = self.sim.spawn_character(hello);
                let (ordered_send, ordered_recv) = mpsc::channel(32);
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
                    sim_config: (*self.cfg).clone(),
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
        connection: quinn::Connection,
        mut send: mpsc::Sender<(ClientId, ClientEvent)>,
    ) {
        let id = self.clients.insert(Client::new(connection.clone()));
        info!(id = ?id.0, address = %connection.remote_address(), "connection established");
        tokio::spawn(async move {
            if let Err(e) = drive_recv(id, connection, &mut send).await {
                // drive_recv returns an error when any connection-terminating issue occurs, so we
                // send a `Lost` message to ensure the client is cleaned up. Note that this message may
                // be redundant, as dropping a slow client also sends a `Lost` message.
                let _ = send.send((id, ClientEvent::Lost(e))).await;
            } else {
                unreachable!("Graceful disconnects are not implemented.")
            }
        });
    }
}

const MAX_CLIENT_MSG_SIZE: usize = 1 << 16;

async fn drive_recv(
    id: ClientId,
    connection: quinn::Connection,
    send: &mut mpsc::Sender<(ClientId, ClientEvent)>,
) -> Result<()> {
    let stream = connection.accept_uni().await.map_err(Error::msg)?;
    let hello = codec::recv_whole::<proto::ClientHello>(MAX_CLIENT_MSG_SIZE, stream).await?;
    let _ = send.send((id, ClientEvent::Hello(hello))).await;

    loop {
        let stream = connection.accept_uni().await.map_err(Error::msg)?;
        let send = send.clone();

        // We spawn a separate task to allow messages to be processed in a different order from when they were
        // initiated.
        let connection = connection.clone();
        tokio::spawn(async move {
            match codec::recv_whole::<proto::Command>(MAX_CLIENT_MSG_SIZE, stream).await {
                Err(e) => {
                    // This error can occur if the client sends a badly-formatted command. In this case,
                    // we want to drop the client. We close the connection, which will cause `drive_recv` to
                    // return eventually.
                    tracing::error!("Error when parsing unordered stream from client: {e}");
                    connection.close(2u32.into(), b"could not process stream");
                }
                Ok(msg) => {
                    let _ = send.send((id, ClientEvent::Command(msg))).await;
                }
            }
        });
    }
}

async fn drive_send(
    conn: quinn::Connection,
    hello: proto::ServerHello,
    unordered: mpsc::Receiver<Unordered>,
    ordered: mpsc::Receiver<Ordered>,
) -> Result<()> {
    let mut stream = conn.open_uni().await?;
    codec::send(&mut stream, &hello).await?;

    tokio::spawn(async move {
        // Errors will be handled by recv task
        let _ = drive_send_unordered(conn.clone(), unordered).await;
    });

    let mut ordered = ReceiverStream::new(ordered);
    while let Some(msg) = ordered.next().await {
        codec::send(&mut stream, &msg).await?;
    }

    Ok(())
}

async fn drive_send_unordered(
    conn: quinn::Connection,
    msgs: mpsc::Receiver<Unordered>,
) -> Result<()> {
    let mut msgs = ReceiverStream::new(msgs);
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
