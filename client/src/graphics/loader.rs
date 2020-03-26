use std::{
    any::{Any, TypeId},
    convert::TryFrom,
    marker::PhantomData,
    sync::Arc,
};

use anyhow::Result;
use downcast_rs::{impl_downcast, Downcast};
use fxhash::FxHashMap;
use lahar::{transfer, transfer::TransferHandle, DedicatedImage, StagingBuffer};
use tokio::sync::mpsc;
use tracing::error;

use super::Base;
use crate::Config;

pub trait Cleanup {
    unsafe fn cleanup(self, gfx: &Base);
}

impl Cleanup for DedicatedImage {
    unsafe fn cleanup(mut self, gfx: &Base) {
        self.destroy(&gfx.device);
    }
}

pub trait Loadable: Send + 'static {
    type Output: Send + 'static + Cleanup;
    fn load(self, ctx: &LoadCtx) -> LoadFuture<'_, Self::Output>;
}

pub type LoadFuture<'a, T> =
    std::pin::Pin<Box<dyn std::future::Future<Output = Result<T>> + 'a + Send>>;

pub struct Loader {
    runtime: tokio::runtime::Runtime,
    recv: mpsc::UnboundedReceiver<Message>,
    shared: Arc<Shared>,
    reactor: transfer::Reactor,
    tables_index: FxHashMap<TypeId, u32>,
    tables: Vec<Box<dyn AnyTable>>,
}

impl Loader {
    pub fn new(cfg: Arc<Config>, gfx: Arc<Base>) -> Self {
        let runtime = tokio::runtime::Builder::new()
            .threaded_scheduler()
            .build()
            .unwrap();
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
            ctx: LoadCtx {
                cfg,
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
            tables_index: FxHashMap::default(),
            tables: Vec::new(),
        }
    }

    pub fn load<L: Loadable>(&mut self, description: &'static str, x: L) -> Asset<L::Output> {
        let tables = &mut self.tables;
        let table = *self
            .tables_index
            .entry(TypeId::of::<L::Output>())
            .or_insert_with(|| {
                let n = u32::try_from(tables.len()).unwrap();
                tables.push(Box::new(Table::<L::Output>::new()));
                n
            });
        let index = self.tables[table as usize]
            .downcast_mut::<Table<L::Output>>()
            .unwrap()
            .alloc();
        let shared = self.shared.clone();
        self.runtime.spawn(async move {
            match shared.ctx.load(x).await {
                Ok(x) => {
                    let _ = shared.send.send(Message {
                        table,
                        index,
                        result: Box::new(x),
                    });
                }
                Err(e) => {
                    error!("{} load failed: {:#}", description, e);
                }
            }
        });
        Asset {
            table,
            index,
            _marker: PhantomData,
        }
    }

    /// Invoke `finish` functions of spawned loading operations
    pub fn drive(&mut self) {
        self.reactor.poll().unwrap();
        while let Ok(msg) = self.recv.try_recv() {
            self.tables[msg.table as usize].finish(msg.index, msg.result);
        }
    }

    pub fn get<T: 'static + Cleanup>(&self, handle: Asset<T>) -> Option<&T> {
        self.tables[handle.table as usize]
            .downcast_ref::<Table<T>>()
            .unwrap()
            .data[handle.index as usize]
            .as_ref()
    }
}

impl Drop for Loader {
    fn drop(&mut self) {
        for table in self.tables.drain(..) {
            table.cleanup(&self.shared.ctx.gfx);
        }
    }
}

struct Shared {
    send: mpsc::UnboundedSender<Message>,
    ctx: LoadCtx,
}

struct Message {
    table: u32,
    index: u32,
    result: Box<dyn Any + Send>,
}

pub struct LoadCtx {
    pub cfg: Arc<Config>,
    pub gfx: Arc<Base>,
    pub staging: StagingBuffer,
    pub transfer: TransferHandle,
}

impl LoadCtx {
    async fn load<T: Loadable>(&self, x: T) -> Result<T::Output> {
        x.load(self).await
    }
}

trait AnyTable: Downcast {
    fn finish(&mut self, index: u32, value: Box<dyn Any + Send>);
    fn cleanup(self: Box<Self>, gfx: &Base);
}

impl_downcast!(AnyTable);

struct Table<T> {
    data: Vec<Option<T>>,
}

impl<T> Table<T> {
    fn new() -> Self {
        Self { data: Vec::new() }
    }

    fn alloc(&mut self) -> u32 {
        let n = u32::try_from(self.data.len()).unwrap();
        self.data.push(None);
        n
    }
}

impl<T: 'static + Cleanup> AnyTable for Table<T> {
    fn finish(&mut self, index: u32, value: Box<dyn Any + Send>) {
        self.data[index as usize] = Some(*value.downcast().unwrap());
    }

    fn cleanup(self: Box<Self>, gfx: &Base) {
        for x in self.data.into_iter().filter_map(|x| x) {
            unsafe {
                x.cleanup(gfx);
            }
        }
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct Asset<T: 'static> {
    table: u32,
    index: u32,
    _marker: PhantomData<T>,
}
