use std::sync::Arc;

use anyhow::Result;
use lahar::{transfer, transfer::TransferHandle, StagingBuffer};
use tokio::sync::mpsc;

use super::Base;

pub trait Loadable: Send + 'static {
    type Output: Send + 'static;
    fn load<'a>(self, handle: &'a Handle) -> LoadFuture<'a, Self::Output>;
}

pub type LoadFuture<'a, T> =
    std::pin::Pin<Box<dyn std::future::Future<Output = Result<T>> + 'a + Send>>;

pub struct Loader<T> {
    runtime: tokio::runtime::Runtime,
    recv: mpsc::UnboundedReceiver<Box<dyn FnOnce(&mut T) + Send + 'static>>,
    shared: Arc<Shared<T>>,
    reactor: transfer::Reactor,
}

impl<T: 'static> Loader<T> {
    pub fn new(gfx: Arc<Base>) -> Self {
        let runtime = tokio::runtime::Runtime::new().unwrap();
        let (send, recv) = mpsc::unbounded_channel();
        let staging =
            StagingBuffer::new(gfx.device.clone(), &gfx.memory_properties, 32 * 1024 * 1024);
        unsafe {
            gfx.set_name(staging.buffer(), cstr!("staging"));
        }
        let (transfer, reactor) = unsafe {
            transfer::Reactor::new(gfx.device.clone(), gfx.queue_family, gfx.queue, None)
        };
        let shared = Arc::new(Shared {
            send,
            handle: Handle {
                gfx,
                staging,
                transfer,
            },
        });
        Self {
            runtime,
            recv,
            shared,
            reactor,
        }
    }

    pub fn spawn<L: Loadable>(
        &mut self,
        x: L,
        finish: impl FnOnce(&mut T, Result<L::Output>) + Send + 'static,
    ) {
        let shared = self.shared.clone();
        self.runtime.spawn(async move {
            let result = shared.handle.load(x).await;
            let _ = shared
                .send
                .clone()
                .send(Box::new(move |ctx| finish(ctx, result)));
        });
    }

    /// Invoke `finish` functions of spawned loading operations
    pub fn drive(&mut self, ctx: &mut T) {
        self.reactor.poll().unwrap();
        while let Ok(callback) = self.recv.try_recv() {
            callback(ctx);
        }
    }
}

struct Shared<T> {
    send: mpsc::UnboundedSender<Box<dyn FnOnce(&mut T) + Send + 'static>>,
    handle: Handle,
}

pub struct Handle {
    pub gfx: Arc<Base>,
    pub staging: StagingBuffer,
    pub transfer: TransferHandle,
}

impl Handle {
    async fn load<T: Loadable>(&self, x: T) -> Result<T::Output> {
        x.load(self).await
    }
}
