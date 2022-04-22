use std::convert::TryFrom;
use std::fmt;
use std::future::Future;
use std::sync::Arc;
use std::thread;
use std::time::Duration;

use ash::vk;
use tokio::sync::{mpsc::{self, error::TryRecvError}, oneshot};
use futures_util::FutureExt;

#[derive(Clone)]
pub struct TransferHandle {
    send: mpsc::UnboundedSender<Message>,
}

impl TransferHandle {
    pub unsafe fn run(
        &self,
        f: impl FnOnce(&mut TransferContext, vk::CommandBuffer) + Send + 'static,
    ) -> impl Future<Output = Result<(), ShutDown>> {
        let (sender, recv) = oneshot::channel();
        let _ = self.send.send(Message {
            sender,
            op: Box::new(f),
        });
        recv.map(|x| x.map_err(|_| ShutDown))
    }
}

pub struct TransferContext {
    pub device: Arc<ash::Device>,
    pub queue_family: u32,
    /// May be equal to queue_family
    pub dst_queue_family: u32,
    pub stages: vk::PipelineStageFlags,
    pub buffer_barriers: Vec<vk::BufferMemoryBarrier>,
    pub image_barriers: Vec<vk::ImageMemoryBarrier>,
}

pub fn acquire_buffer(
    src_queue_family: u32,
    dst_queue_family: u32,
    buffer: vk::Buffer,
    offset: vk::DeviceSize,
    size: vk::DeviceSize,
) -> vk::BufferMemoryBarrier {
    vk::BufferMemoryBarrier::builder()
        .src_access_mask(vk::AccessFlags::TRANSFER_WRITE)
        .dst_access_mask(vk::AccessFlags::SHADER_READ)
        .src_queue_family_index(src_queue_family)
        .dst_queue_family_index(dst_queue_family)
        .buffer(buffer)
        .offset(offset)
        .size(size)
        .build()
}

pub fn acquire_image(
    src_queue_family: u32,
    dst_queue_family: u32,
    image: vk::Image,
) -> vk::ImageMemoryBarrier {
    vk::ImageMemoryBarrier::builder()
        .src_access_mask(vk::AccessFlags::TRANSFER_WRITE)
        .dst_access_mask(vk::AccessFlags::SHADER_READ)
        .src_queue_family_index(src_queue_family)
        .dst_queue_family_index(dst_queue_family)
        .old_layout(vk::ImageLayout::TRANSFER_DST_OPTIMAL)
        .new_layout(vk::ImageLayout::SHADER_READ_ONLY_OPTIMAL)
        .image(image)
        .build()
}

#[derive(Debug, Copy, Clone)]
pub struct ShutDown;

impl fmt::Display for ShutDown {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.pad("transfer reactor shut down")
    }
}

impl std::error::Error for ShutDown {}

struct Message {
    sender: oneshot::Sender<()>,
    op: Box<dyn FnOnce(&mut TransferContext, vk::CommandBuffer) + Send>,
}

pub struct Reactor {
    queue: vk::Queue,
    spare_fences: Vec<vk::Fence>,
    spare_cmds: Vec<vk::CommandBuffer>,
    in_flight: Vec<Batch>,
    /// Fences for in-flight transfer operations; directly corresponds to in_flight entries
    in_flight_fences: Vec<vk::Fence>,
    cmd_pool: vk::CommandPool,
    pending: Option<Batch>,
    recv: mpsc::UnboundedReceiver<Message>,
    ctx: TransferContext,
}

impl Reactor {
    /// Safety: valid use use of queue_family, queue
    pub unsafe fn new(
        device: Arc<ash::Device>,
        queue_family: u32,
        queue: vk::Queue,
        dst_queue_family: Option<u32>,
    ) -> (TransferHandle, Self) {
        let (send, recv) = mpsc::unbounded_channel();
        let cmd_pool = device
            .create_command_pool(
                &vk::CommandPoolCreateInfo::builder()
                    .queue_family_index(queue_family)
                    .flags(vk::CommandPoolCreateFlags::RESET_COMMAND_BUFFER),
                None,
            )
            .unwrap();
        (
            TransferHandle { send },
            Self {
                queue,
                spare_fences: Vec::new(),
                spare_cmds: Vec::new(),
                in_flight: Vec::new(),
                in_flight_fences: Vec::new(),
                cmd_pool,
                pending: None,
                recv,
                ctx: TransferContext {
                    device,
                    queue_family,
                    dst_queue_family: dst_queue_family.unwrap_or(queue_family),
                    stages: vk::PipelineStageFlags::empty(),
                    buffer_barriers: Vec::new(),
                    image_barriers: Vec::new(),
                },
            },
        )
    }

    pub unsafe fn spawn(
        device: Arc<ash::Device>,
        queue_family: u32,
        queue: vk::Queue,
        dst_queue_family: Option<u32>,
    ) -> (TransferHandle, thread::JoinHandle<()>) {
        let (transfer, mut core) = Self::new(device, queue_family, queue, dst_queue_family);
        let thread = thread::spawn(
            move || {
                while core.run_for(Duration::from_millis(250)).is_ok() {}
            },
        );
        (transfer, thread)
    }

    pub fn poll(&mut self) -> Result<(), Disconnected> {
        self.run_for(Duration::from_secs(0))
    }

    pub fn run_for(&mut self, timeout: Duration) -> Result<(), Disconnected> {
        self.queue()?;
        self.flush();

        if self.in_flight.is_empty() {
            thread::sleep(timeout);
            return Ok(());
        }

        // We could move this to a background thread and continue to submit new work while it's
        // waiting, but we want to batch up operations a bit anyway.
        let result = unsafe {
            self.ctx.device.wait_for_fences(
                &self.in_flight_fences,
                false,
                u64::try_from(timeout.as_nanos()).unwrap_or(u64::max_value()),
            )
        };
        match result {
            Err(vk::Result::TIMEOUT) => return Ok(()),
            Err(e) => panic!("{}", e),
            Ok(()) => {}
        }
        for i in (0..self.in_flight.len()).rev() {
            unsafe {
                if self
                    .ctx
                    .device
                    .get_fence_status(self.in_flight_fences[i])
                    .unwrap()
                {
                    let fence = self.in_flight_fences.swap_remove(i);
                    self.ctx.device.reset_fences(&[fence]).unwrap();
                    self.spare_fences.push(fence);
                    let batch = self.in_flight.swap_remove(i);
                    for sender in batch.senders {
                        let _ = sender.send(());
                    }
                    self.spare_cmds.push(batch.cmd);
                }
            }
        }
        Ok(())
    }

    fn queue(&mut self) -> Result<(), Disconnected> {
        loop {
            match self.recv.try_recv() {
                Ok(Message { sender, op }) => {
                    let cmd = self.prepare(sender);
                    op(&mut self.ctx, cmd);
                }
                Err(TryRecvError::Closed) => return Err(self::Disconnected),
                Err(TryRecvError::Empty) => return Ok(()),
            }
        }
    }

    fn prepare(&mut self, send: oneshot::Sender<()>) -> vk::CommandBuffer {
        if let Some(ref mut pending) = self.pending {
            pending.senders.push(send);
            return pending.cmd;
        }
        let cmd = if let Some(cmd) = self.spare_cmds.pop() {
            cmd
        } else {
            unsafe {
                self.ctx
                    .device
                    .allocate_command_buffers(
                        &vk::CommandBufferAllocateInfo::builder()
                            .command_pool(self.cmd_pool)
                            .command_buffer_count(1),
                    )
                    .unwrap()
                    .into_iter()
                    .next()
                    .unwrap()
            }
        };
        unsafe {
            self.ctx
                .device
                .begin_command_buffer(
                    cmd,
                    &vk::CommandBufferBeginInfo::builder()
                        .flags(vk::CommandBufferUsageFlags::ONE_TIME_SUBMIT),
                )
                .unwrap();
        }
        self.pending = Some(Batch {
            cmd,
            senders: vec![send],
        });
        cmd
    }

    /// Submit queued operations
    fn flush(&mut self) {
        let pending = match self.pending.take() {
            Some(x) => x,
            None => return,
        };
        let device = &self.ctx.device;
        let fence = if let Some(fence) = self.spare_fences.pop() {
            fence
        } else {
            unsafe {
                device
                    .create_fence(&vk::FenceCreateInfo::default(), None)
                    .unwrap()
            }
        };
        unsafe {
            device.cmd_pipeline_barrier(
                pending.cmd,
                vk::PipelineStageFlags::TRANSFER,
                self.ctx.stages,
                vk::DependencyFlags::default(),
                &[],
                &self.ctx.buffer_barriers,
                &self.ctx.image_barriers,
            );
            device.end_command_buffer(pending.cmd).unwrap();
            device
                .queue_submit(
                    self.queue,
                    &[vk::SubmitInfo::builder()
                        .command_buffers(&[pending.cmd])
                        .build()],
                    fence,
                )
                .unwrap();
        }
        self.ctx.stages = vk::PipelineStageFlags::empty();
        self.ctx.buffer_barriers.clear();
        self.ctx.image_barriers.clear();
        self.in_flight.push(pending);
        self.in_flight_fences.push(fence);
    }
}

impl Drop for Reactor {
    fn drop(&mut self) {
        let device = &self.ctx.device;
        unsafe {
            if !self.in_flight.is_empty() {
                device
                    .wait_for_fences(&self.in_flight_fences, true, u64::max_value())
                    .unwrap();
            }
            device.destroy_command_pool(self.cmd_pool, None);
            for fence in self.spare_fences.drain(..) {
                device.destroy_fence(fence, None);
            }
            for fence in self.in_flight_fences.drain(..) {
                device.destroy_fence(fence, None);
            }
        }
    }
}

unsafe impl Send for Reactor {}

struct Batch {
    cmd: vk::CommandBuffer,
    // Future work: efficient broadcast future
    senders: Vec<oneshot::Sender<()>>,
}

#[derive(Debug, Copy, Clone)]
pub struct Disconnected;
