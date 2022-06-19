use std::{sync::Arc, thread};

use anyhow::{anyhow, Error, Result};
use futures_util::{StreamExt, TryStreamExt};
use tokio::sync::mpsc;

use common::{codec, proto};

use crate::Config;

pub struct Net {
    pub incoming: mpsc::UnboundedReceiver<Message>,
    pub outgoing: mpsc::UnboundedSender<proto::Command>,
    pub thread: thread::JoinHandle<()>,
}

pub fn spawn(cfg: Arc<Config>) -> Net {
    let (incoming_send, incoming_recv) = mpsc::unbounded_channel();
    let (outgoing_send, outgoing_recv) = mpsc::unbounded_channel();
    let thread = thread::spawn(move || {
        if let Err(e) = run(cfg, incoming_send.clone(), outgoing_recv) {
            let _ = incoming_send.send(Message::ConnectionLost(e));
        }
    });
    Net {
        incoming: incoming_recv,
        outgoing: outgoing_send,
        thread,
    }
}

#[derive(Debug)]
pub enum Message {
    Hello(proto::ServerHello),
    Spawns(proto::Spawns),
    StateDelta(proto::StateDelta),
    ConnectionLost(Error),
}

#[tokio::main(worker_threads = 1)]
async fn run(
    cfg: Arc<Config>,
    incoming: mpsc::UnboundedSender<Message>,
    outgoing: mpsc::UnboundedReceiver<proto::Command>,
) -> Result<()> {
    let mut endpoint = quinn::Endpoint::client("[::]:0".parse().unwrap())?;
    let crypto = rustls::ClientConfig::builder()
        .with_safe_defaults()
        .with_custom_certificate_verifier(Arc::new(AcceptAnyCert))
        .with_no_client_auth();
    let client_cfg = quinn::ClientConfig::new(Arc::new(crypto));
    endpoint.set_default_client_config(client_cfg);

    let result = inner(cfg, incoming, outgoing, endpoint.clone()).await;
    endpoint.wait_idle().await;
    result
}

async fn inner(
    cfg: Arc<Config>,
    incoming: mpsc::UnboundedSender<Message>,
    outgoing: mpsc::UnboundedReceiver<proto::Command>,
    endpoint: quinn::Endpoint,
) -> Result<()> {
    let server = cfg.server.unwrap();
    let quinn::NewConnection {
        connection,
        mut uni_streams,
        ..
    } = endpoint.connect(server, "localhost").unwrap().await?;

    // Open the first stream for our hello message
    let clienthello_stream = connection.open_uni().await?;
    // Start sending commands asynchronously
    tokio::spawn(handle_outgoing(outgoing, connection));
    // Actually send the hello message
    codec::send_whole(
        clienthello_stream,
        &proto::ClientHello {
            name: (*cfg.name).into(),
        },
    )
    .await?;

    let mut ordered = uni_streams.next().await.unwrap()?;
    // Handle unordered messages
    tokio::spawn(handle_unordered(incoming.clone(), uni_streams));

    // Receive the server's hello message
    let hello = codec::recv::<proto::ServerHello>(&mut ordered)
        .await?
        .ok_or_else(|| anyhow!("ordered stream closed unexpectedly"))?;
    // Forward it on
    incoming.send(Message::Hello(hello)).unwrap();

    // Receive ordered messages from the server
    loop {
        let spawns = codec::recv::<proto::Spawns>(&mut ordered)
            .await?
            .ok_or_else(|| anyhow!("ordered stream closed unexpectedly"))?;
        incoming.send(Message::Spawns(spawns)).unwrap();
    }
}

/// Send commands to the server
async fn handle_outgoing(
    mut outgoing: mpsc::UnboundedReceiver<proto::Command>,
    connection: quinn::Connection,
) -> Result<()> {
    while let Some(cmd) = outgoing.recv().await {
        let stream = connection.open_uni().await?;
        // TODO: Don't silently die on parse errors
        codec::send_whole(stream, &cmd).await?;
    }
    Ok(())
}

/// Receive unordered messages from the server
async fn handle_unordered(
    incoming: mpsc::UnboundedSender<Message>,
    uni_streams: quinn::IncomingUniStreams,
) -> Result<()> {
    let mut msgs = uni_streams
        .map(|stream| async {
            let stream = stream?;
            codec::recv_whole::<proto::StateDelta>(2usize.pow(16), stream).await
        })
        .buffer_unordered(128);
    // TODO: Don't silently die on parse errors
    while let Some(msg) = msgs.try_next().await? {
        // Ignore errors so we don't panic if the simulation thread goes away between checking
        // `msgs` and here.
        let _ = incoming.send(Message::StateDelta(msg));
    }
    Ok(())
}

struct AcceptAnyCert;

impl rustls::client::ServerCertVerifier for AcceptAnyCert {
    fn verify_server_cert(
        &self,
        _end_entity: &rustls::Certificate,
        _intermediates: &[rustls::Certificate],
        _server_name: &rustls::ServerName,
        _scts: &mut dyn Iterator<Item = &[u8]>,
        _ocsp_response: &[u8],
        _now: std::time::SystemTime,
    ) -> Result<rustls::client::ServerCertVerified, rustls::Error> {
        Ok(rustls::client::ServerCertVerified::assertion())
    }
}
