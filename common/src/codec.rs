use anyhow::{Result, bail};
use serde::{Serialize, de::DeserializeOwned};

pub async fn send<T: Serialize + ?Sized>(stream: &mut quinn::SendStream, msg: &T) -> Result<()> {
    let buf = postcard::to_stdvec(msg).unwrap();
    let len = buf.len();
    if len >= 1 << 24 {
        bail!("{} byte ordered message exceeds maximum length", len);
    }
    stream.write_all(&(len as u32).to_le_bytes()[0..3]).await?;
    stream.write_all(&buf).await?;
    Ok(())
}

/// Returns `None` on end of stream
pub async fn recv<T: DeserializeOwned>(stream: &mut quinn::RecvStream) -> Result<Option<T>> {
    let mut tag = [0; 4];
    match stream.read_exact(&mut tag[0..3]).await {
        Err(quinn::ReadExactError::FinishedEarly(_)) => return Ok(None),
        Err(quinn::ReadExactError::ReadError(e)) => return Err(e.into()),
        Ok(()) => {}
    }

    let len = u32::from_le_bytes(tag) as usize;
    let mut buf = vec![0; len];
    match stream.read_exact(&mut buf).await {
        Err(quinn::ReadExactError::FinishedEarly(_)) => return Ok(None),
        Err(quinn::ReadExactError::ReadError(e)) => return Err(e.into()),
        Ok(()) => {}
    }
    Ok(Some(postcard::from_bytes(&buf)?))
}

/// Send a message as the entirety of `stream`
pub async fn send_whole<T: Serialize + ?Sized>(
    mut stream: quinn::SendStream,
    msg: &T,
) -> std::result::Result<(), quinn::WriteError> {
    let buf = postcard::to_stdvec(msg).unwrap();
    stream.write_all(&buf).await?;
    Ok(())
}

/// Receive the entirety of `stream` as a `T`
pub async fn recv_whole<T: DeserializeOwned>(
    size_limit: usize,
    mut stream: quinn::RecvStream,
) -> Result<T> {
    let buf = stream.read_to_end(size_limit).await?;
    Ok(postcard::from_bytes(&buf)?)
}
