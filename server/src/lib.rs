#![allow(clippy::needless_borrowed_reference)]

extern crate nalgebra as na;
mod input_queue;
mod postcard_helpers;
mod sim;

use std::{net::UdpSocket, sync::Arc, time::Instant};

use anyhow::{Context, Error, Result, anyhow};
use hecs::Entity;
use quinn::rustls::pki_types::{CertificateDer, PrivateKeyDer};
use slotmap::DenseSlotMap;
use tokio::sync::mpsc;
use tracing::{debug, error, error_span, info, trace, warn};

use common::{
    SimConfig, codec,
    proto::{self, connection_error_codes},
};
use input_queue::InputQueue;
use save::Save;
use sim::Sim;

pub struct NetParams {
    pub certificate_chain: Vec<CertificateDer<'static>>,
    pub private_key: PrivateKeyDer<'static>,
    pub socket: UdpSocket,
}

pub struct Server {
    cfg: Arc<SimConfig>,
    sim: Sim,
    clients: DenseSlotMap<ClientId, Client>,
    save: Save,
    endpoint: Option<quinn::Endpoint>,

    new_clients_send: mpsc::UnboundedSender<(quinn::Connection, proto::ClientHello)>,
    new_clients_recv: mpsc::UnboundedReceiver<(quinn::Connection, proto::ClientHello)>,
    client_events_send: mpsc::Sender<(ClientId, ClientEvent)>,
    client_events_recv: mpsc::Receiver<(ClientId, ClientEvent)>,
}

impl Server {
    pub fn new(net: Option<NetParams>, mut cfg: SimConfig, save: Save) -> Result<Self> {
        cfg.chunk_size = save.meta().chunk_size as u8;
        let endpoint = net
            .map(|net| {
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
                Ok::<_, Error>(endpoint)
            })
            .transpose()?;

        let (new_clients_send, new_clients_recv) = mpsc::unbounded_channel();
        let (client_events_send, client_events_recv) = mpsc::channel(128);

        let cfg = Arc::new(cfg);
        Ok(Self {
            sim: Sim::new(cfg.clone(), &save),
            cfg,
            clients: DenseSlotMap::default(),
            save,
            endpoint,

            new_clients_send,
            new_clients_recv,
            client_events_send,
            client_events_recv,
        })
    }

    pub fn connect(&mut self, hello: proto::ClientHello, mut backend: HandleBackend) -> Result<()> {
        let snapshot = Arc::new(self.sim.snapshot());
        let (id, entity) = self
            .sim
            .activate_or_spawn_character(&hello)
            .ok_or_else(|| anyhow!("could not spawn {} due to name conflict", hello.name))?;
        let (ordered_send, mut ordered_recv) = mpsc::channel(32);
        ordered_send.try_send(snapshot).unwrap();
        let (unordered_send, mut unordered_recv) = mpsc::channel(32);
        let client_id = self.clients.insert(Client {
            conn: None,
            character: entity,
            ordered: ordered_send,
            unordered: unordered_send,
            latest_input_received: 0,
            latest_input_processed: 0,
            inputs: InputQueue::new(),
        });

        backend
            .incoming
            .send(Message::Hello(proto::ServerHello {
                character: id,
                sim_config: (*self.cfg).clone(),
            }))
            .unwrap();

        // Adapt channels. TODO: Make this unnecessary.
        let client_events_send = self.client_events_send.clone();
        tokio::spawn(async move {
            while let Some(msg) = backend.outgoing.recv().await {
                _ = client_events_send
                    .send((client_id, ClientEvent::Command(msg)))
                    .await;
            }
        });
        tokio::spawn({
            let incoming_send = backend.incoming.clone();
            async move {
                while let Some(msg) = ordered_recv.recv().await {
                    _ = incoming_send.send(Message::Spawns(proto::Spawns::clone(&*msg)));
                }
            }
        });
        tokio::spawn(async move {
            while let Some(msg) = unordered_recv.recv().await {
                _ = backend.incoming.send(Message::StateDelta(msg));
            }
        });

        info!(id = ?client_id.0, "connected locally");
        Ok(())
    }

    pub async fn run(mut self) {
        let mut ticks = tokio::time::interval(self.cfg.step_interval);
        let mut incoming = self.handle_incoming();
        loop {
            tokio::select! {
                _ = ticks.tick() => { self.on_step(); },
                Some(conn) = incoming.recv() => { self.on_connect(conn); }
                Some((id, event)) = self.client_events_recv.recv() => { self.on_client_event(id, event); }
                Some((conn, hello)) = self.new_clients_recv.recv() => { self.on_client(conn, hello); }
            }
        }
    }

    fn handle_incoming(&self) -> mpsc::Receiver<quinn::Connection> {
        let (incoming_send, incoming_recv) = mpsc::channel(16);
        let Some(endpoint) = self.endpoint.clone() else {
            return incoming_recv;
        };
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
            if let Some(cmd) = client.inputs.pop(now, self.cfg.input_queue_size) {
                client.latest_input_processed = cmd.generation;
                if let Err(e) = self.sim.command(client.character, cmd) {
                    error!(client = ?id, "couldn't process command: {}", e);
                }
            }
        }

        // Step the simulation
        let (spawns, delta) = self.sim.step();
        let spawns = spawns.map(Arc::new);
        let mut overran = Vec::new();
        for (client_id, client) in &mut self.clients {
            let mut delta = delta.clone();
            delta.latest_input = client.latest_input_processed;
            let r1 = client.unordered.try_send(delta);
            let r2 = if let Some(spawns) = spawns.as_ref() {
                client.ordered.try_send(Arc::clone(spawns))
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
        for client_id in overran {
            match self.clients[client_id].conn {
                Some(ref conn) => {
                    error!("dropping slow client {:?}", client_id.0);
                    conn.close(
                        connection_error_codes::STREAM_ERROR,
                        b"client reading too slowly",
                    );
                    self.cleanup_client(client_id);
                }
                None => {
                    warn!("slow local client {:?}", client_id.0);
                }
            }
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
            ClientEvent::Lost(e) => {
                error!("lost: {:#}", e);
                self.cleanup_client(client_id);
            }
            ClientEvent::Command(cmd) => {
                if cmd.generation.wrapping_sub(client.latest_input_received) < u16::MAX / 2 {
                    client.latest_input_received = cmd.generation;
                    client.inputs.push(cmd, Instant::now());
                } else {
                    debug!("dropping obsolete command");
                }
            }
        }
    }

    fn cleanup_client(&mut self, client: ClientId) {
        self.sim
            .deactivate_character(self.clients[client].character);
        self.clients.remove(client);
    }

    fn on_connect(&mut self, connection: quinn::Connection) {
        let send = self.new_clients_send.clone();
        tokio::spawn(async move {
            let result: anyhow::Result<()> = async move {
                let stream = connection.accept_uni().await.map_err(Error::msg)?;
                let hello =
                    codec::recv_whole::<proto::ClientHello>(MAX_CLIENT_MSG_SIZE, stream).await?;
                _ = send.send((connection, hello));
                Ok(())
            }
            .await;
            if let Err(e) = result {
                warn!("error reading client hello: {e:#}");
            }
        });
    }

    fn on_client(&mut self, connection: quinn::Connection, hello: proto::ClientHello) {
        let snapshot = Arc::new(self.sim.snapshot());
        let Some((id, entity)) = self.sim.activate_or_spawn_character(&hello) else {
            error!("could not spawn {} due to name conflict", hello.name);
            connection.close(connection_error_codes::NAME_CONFLICT, b"name conflict");
            return;
        };
        let (ordered_send, ordered_recv) = mpsc::channel(32);
        ordered_send.try_send(snapshot).unwrap();
        let (unordered_send, unordered_recv) = mpsc::channel(32);
        let client_id = self.clients.insert(Client {
            conn: Some(connection.clone()),
            character: entity,
            ordered: ordered_send,
            unordered: unordered_send,
            latest_input_received: 0,
            latest_input_processed: 0,
            inputs: InputQueue::new(),
        });
        info!(id = ?client_id.0, address = %connection.remote_address(), "connection established");
        let server_hello = proto::ServerHello {
            character: id,
            sim_config: (*self.cfg).clone(),
        };
        tokio::spawn({
            let connection = connection.clone();
            async move {
                // Errors will be handled by recv task
                let _ = drive_send(connection, server_hello, unordered_recv, ordered_recv).await;
            }
        });
        let mut send = self.client_events_send.clone();
        tokio::spawn(async move {
            if let Err(e) = drive_recv(client_id, connection, &mut send).await {
                // drive_recv returns an error when any connection-terminating issue occurs, so we
                // send a `Lost` message to ensure the client is cleaned up. Note that this message may
                // be redundant, as dropping a slow client also sends a `Lost` message.
                let _ = send.send((client_id, ClientEvent::Lost(e))).await;
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
                    connection.close(
                        connection_error_codes::BAD_CLIENT_COMMAND,
                        b"could not process stream",
                    );
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
    mut ordered: mpsc::Receiver<Ordered>,
) -> Result<()> {
    let mut stream = conn.open_uni().await?;
    codec::send(&mut stream, &hello).await?;

    tokio::spawn(async move {
        // Errors will be handled by recv task
        let _ = drive_send_unordered(conn.clone(), unordered).await;
    });

    while let Some(msg) = ordered.recv().await {
        codec::send(&mut stream, &msg).await?;
    }

    Ok(())
}

async fn drive_send_unordered(
    conn: quinn::Connection,
    mut msgs: mpsc::Receiver<Unordered>,
) -> Result<()> {
    while let Some(msg) = msgs.recv().await {
        let stream = conn.open_uni().await?;
        codec::send_whole(stream, &msg).await?;
    }
    Ok(())
}

slotmap::new_key_type! {
    struct ClientId;
}

struct Client {
    conn: Option<quinn::Connection>,
    character: Entity,
    ordered: mpsc::Sender<Ordered>,
    unordered: mpsc::Sender<Unordered>,
    latest_input_received: u16,
    latest_input_processed: u16,
    inputs: InputQueue,
}

enum ClientEvent {
    Command(proto::Command),
    Lost(Error),
}

type Unordered = proto::StateDelta;

type Ordered = Arc<proto::Spawns>;

/// A client's view of a server
pub struct Handle {
    pub incoming: mpsc::UnboundedReceiver<Message>,
    pub outgoing: mpsc::UnboundedSender<proto::Command>,
}

impl Handle {
    pub fn loopback() -> (Self, HandleBackend) {
        let (incoming_send, incoming_recv) = mpsc::unbounded_channel();
        let (outgoing_send, outgoing_recv) = mpsc::unbounded_channel();
        (
            Self {
                incoming: incoming_recv,
                outgoing: outgoing_send,
            },
            HandleBackend {
                incoming: incoming_send,
                outgoing: outgoing_recv,
            },
        )
    }
}

pub struct HandleBackend {
    incoming: mpsc::UnboundedSender<Message>,
    outgoing: mpsc::UnboundedReceiver<proto::Command>,
}

#[derive(Debug)]
pub enum Message {
    Hello(proto::ServerHello),
    Spawns(proto::Spawns),
    StateDelta(proto::StateDelta),
    ConnectionLost(Error),
}
