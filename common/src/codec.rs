use anyhow::{bail, Result};
use serde::{de::DeserializeOwned, Serialize};

pub async fn send<T: Serialize + ?Sized>(stream: &mut quinn::SendStream, msg: &T) -> Result<()> {
    let mut buf = Vec::new();
    let len = bincode::serialized_size(msg).unwrap();
    if len >= 2u64.pow(24) {
        bail!("{} byte ordered message exceeds maximum length", len);
    }
    let tag = (len as u32).to_le_bytes();
    buf.extend_from_slice(&tag[0..3]);
    bincode::serialize_into(&mut buf, msg).unwrap();
    stream.write_all(&buf).await?;
    Ok(())
}

/// Returns `None` on end of stream
pub async fn recv<T: DeserializeOwned>(stream: &mut quinn::RecvStream) -> Result<Option<T>> {
    let mut tag = [0; 4];
    match stream.read_exact(&mut tag[0..3]).await {
        Err(quinn::ReadExactError::FinishedEarly) => return Ok(None),
        Err(quinn::ReadExactError::ReadError(e)) => return Err(e.into()),
        Ok(()) => {}
    }

    let len = u32::from_le_bytes(tag) as usize;
    let mut buf = vec![0; len];
    match stream.read_exact(&mut buf).await {
        Err(quinn::ReadExactError::FinishedEarly) => return Ok(None),
        Err(quinn::ReadExactError::ReadError(e)) => return Err(e.into()),
        Ok(()) => {}
    }
    Ok(Some(bincode::deserialize(&buf)?))
}

/// Send a message as the entirety of `stream`
pub async fn send_whole<T: Serialize + ?Sized>(
    mut stream: quinn::SendStream,
    msg: &T,
) -> std::result::Result<(), quinn::WriteError> {
    let buf = bincode::serialize(msg).unwrap();
    stream.write_all(&buf).await?;
    stream.finish().await?;
    Ok(())
}

/// Receive the entirety of `stream` as a `T`
pub async fn recv_whole<T: DeserializeOwned>(
    size_limit: usize,
    mut stream: quinn::RecvStream,
) -> Result<T> {
    let mut buf = Vec::<u8>::with_capacity(1024);
    let mut cursor = 0;
    loop {
        let slice = unsafe {
            std::slice::from_raw_parts_mut(
                buf.as_mut_ptr().add(cursor),
                size_limit.min(buf.capacity()) - cursor,
            )
        };
        match stream.read(slice).await? {
            Some(n) => {
                cursor += n;
                if cursor == size_limit {
                    bail!("message too large");
                }
                if cursor == buf.capacity() {
                    buf.reserve(size_limit.min(buf.capacity() * 2) - buf.len());
                }
                unsafe {
                    buf.set_len(cursor);
                }
            }
            None => {
                break;
            }
        }
    }
    Ok(bincode::deserialize(&buf)?)
}
