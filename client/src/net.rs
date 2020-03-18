use std::{sync::Arc, thread};

use anyhow::{anyhow, Error, Result};
use futures_util::{StreamExt, TryStreamExt};
use tokio::sync::mpsc;

use common::{codec, proto};

pub struct Net {
    pub incoming: mpsc::UnboundedReceiver<Message>,
    pub outgoing: mpsc::UnboundedSender<proto::Command>,
    pub thread: thread::JoinHandle<()>,
}

pub fn spawn() -> Net {
    let (incoming_send, incoming_recv) = mpsc::unbounded_channel();
    let (outgoing_send, outgoing_recv) = mpsc::unbounded_channel();
    let thread = thread::spawn(move || {
        if let Err(e) = run(incoming_send.clone(), outgoing_recv) {
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

#[tokio::main(core_threads = 1)]
async fn run(
    incoming: mpsc::UnboundedSender<Message>,
    outgoing: mpsc::UnboundedReceiver<proto::Command>,
) -> Result<()> {
    let mut endpoint = quinn::Endpoint::builder();
    let mut client_cfg = quinn::ClientConfig::default();
    let tls_cfg = Arc::get_mut(&mut client_cfg.crypto).unwrap();
    tls_cfg
        .dangerous()
        .set_certificate_verifier(Arc::new(AcceptAnyCert));
    endpoint.default_client_config(client_cfg);
    let (endpoint, _) = endpoint.bind(&"[::]:0".parse().unwrap())?;

    let result = inner(incoming, outgoing, endpoint.clone()).await;
    endpoint.wait_idle().await;
    result
}

async fn inner(
    incoming: mpsc::UnboundedSender<Message>,
    outgoing: mpsc::UnboundedReceiver<proto::Command>,
    endpoint: quinn::Endpoint,
) -> Result<()> {
    let quinn::NewConnection {
        connection,
        mut uni_streams,
        ..
    } = endpoint
        .connect(&"[::1]:1234".parse().unwrap(), "localhost")
        .unwrap()
        .await?;

    // Open the first stream for our hello message
    let clienthello_stream = connection.open_uni().await?;
    // Start sending commands asynchronously
    tokio::spawn(handle_outgoing(outgoing, connection));
    // Actually send the hello message
    codec::send_whole(clienthello_stream, &proto::ClientHello {}).await?;

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
            Ok::<_, Error>(codec::recv_whole::<proto::StateDelta>(2usize.pow(16), stream).await?)
        })
        .buffer_unordered(128);
    // TODO: Don't silently die on parse errors
    while let Some(msg) = msgs.try_next().await? {
        incoming.send(Message::StateDelta(msg)).unwrap();
    }
    Ok(())
}

struct AcceptAnyCert;

impl rustls::ServerCertVerifier for AcceptAnyCert {
    fn verify_server_cert(
        &self,
        _roots: &rustls::RootCertStore,
        _presented_certs: &[rustls::Certificate],
        _dns_name: webpki::DNSNameRef,
        _ocsp_response: &[u8],
    ) -> Result<rustls::ServerCertVerified, rustls::TLSError> {
        Ok(rustls::ServerCertVerified::assertion())
    }
}
