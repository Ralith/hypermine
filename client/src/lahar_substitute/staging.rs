use std::future::Future;
use std::ops::{Deref, DerefMut};
use std::sync::{Arc, Mutex};
use std::task::Poll;

use ash::{vk, Device};
use futures_util::future;

use super::condition::{self, Condition};
use super::ring_alloc::{self, RingAlloc};
use lahar::DedicatedMapping;

/// A host-visible circular buffer for short-lived allocations
///
/// Best for transient uses like streaming transfers. Retaining an allocation of any size will block
/// future allocations once the buffer wraps back aground.
pub struct StagingBuffer {
    device: Arc<Device>,
    buffer: DedicatedMapping<[u8]>,
    state: Mutex<State>,
}

struct State {
    alloc: RingAlloc,
    free: Condition,
}

impl StagingBuffer {
    pub fn new(
        device: Arc<Device>,
        props: &vk::PhysicalDeviceMemoryProperties,
        capacity: usize,
    ) -> Self {
        let buffer = unsafe {
            DedicatedMapping::zeroed_array(
                &*device,
                props,
                vk::BufferUsageFlags::TRANSFER_SRC,
                capacity,
            )
        };
        Self {
            device,
            buffer,
            state: Mutex::new(State {
                alloc: RingAlloc::new(),
                free: Condition::new(),
            }),
        }
    }

    pub fn buffer(&self) -> vk::Buffer {
        self.buffer.buffer()
    }

    /// Largest possible allocation
    pub fn capacity(&self) -> usize {
        self.buffer.len()
    }

    /// Completes when sufficient space is available
    ///
    /// Yields `None` if `size > self.capacity()`. No fairness guarantees, i.e. small allocations
    /// may starve large ones.
    pub fn alloc(&self, size: usize) -> impl Future<Output = Option<Alloc<'_>>> {
        let mut cond_state = condition::State::default();
        future::poll_fn(move |cx| {
            if size > self.capacity() {
                return Poll::Ready(None);
            }
            let mut state = self.state.lock().unwrap();
            match state.alloc.alloc(self.capacity(), size) {
                None => {
                    state.free.register(cx, &mut cond_state);
                    Poll::Pending
                }
                Some((offset, id)) => Poll::Ready(Some(Alloc {
                    buf: self,
                    bytes: unsafe {
                        std::slice::from_raw_parts_mut(
                            (self.buffer.as_ptr() as *const u8).add(offset) as *mut u8,
                            size,
                        )
                    },
                    id,
                })),
            }
        })
    }

    fn free(&self, id: ring_alloc::Id) {
        let mut state = self.state.lock().unwrap();
        state.alloc.free(id);
        state.free.notify();
    }
}

impl Drop for StagingBuffer {
    fn drop(&mut self) {
        unsafe {
            self.buffer.destroy(&*self.device);
        }
    }
}

/// An allocation from a `StagingBuffer`
pub struct Alloc<'a> {
    buf: &'a StagingBuffer,
    bytes: &'a mut [u8],
    id: ring_alloc::Id,
}

impl Alloc<'_> {
    pub fn offset(&self) -> vk::DeviceSize {
        self.bytes.as_ptr() as vk::DeviceSize
            - self.buf.buffer.as_ptr() as *const u8 as vk::DeviceSize
    }

    pub fn size(&self) -> vk::DeviceSize {
        self.bytes.len() as _
    }
}

impl Deref for Alloc<'_> {
    type Target = [u8];

    fn deref(&self) -> &[u8] {
        &self.bytes
    }
}

impl DerefMut for Alloc<'_> {
    fn deref_mut(&mut self) -> &mut [u8] {
        self.bytes
    }
}

impl Drop for Alloc<'_> {
    fn drop(&mut self) {
        self.buf.free(self.id);
    }
}
