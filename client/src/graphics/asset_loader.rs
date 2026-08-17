use std::{
    path::{Path, PathBuf},
    ptr::NonNull,
    rc::Rc,
    sync::Arc,
    thread::{self, JoinHandle},
};

use ash::vk;
use lahar::{BufferRegionAlloc, GrowableRing, ParallelQueue, parallel_queue};
use skid_steer::Context;
use tokio_util::sync::CancellationToken;

use crate::{
    Config,
    graphics::{Base, meshes, shader_data::ShaderData},
};

/// Contains all the dependencies necessary to load assets.
pub struct AssetLoadContext {
    gfx: Arc<Base>,
    config: Arc<Config>,
    queue_handle: parallel_queue::Handle,
    queue_watch_receiver: tokio::sync::watch::Receiver<u64>,
    queue_unparker: Arc<QueueUnparker>,
    staging: Arc<GrowableRing>,
}

// Rather than exposing its fields, we expose helper functions for the kinds of tasks
// one would need the context for. This helps add some level of separation between how global
// data is organized and what asset loading code sees
impl AssetLoadContext {
    /// # Safety
    /// - [`Work::cmd`] must not be used outside the lifetime of the returned [`Work`]
    /// - Any Vulkan resources this work uses must not be destroyed before the [`Work`]
    ///   is fully executed (or dropped without being sent for submission).
    pub unsafe fn begin_work(&self) -> parallel_queue::Work<'_> {
        unsafe { self.queue_handle.begin(&self.gfx.device) }
    }

    pub fn alloc_staging<T>(
        &self,
        count: usize,
        align: usize,
        free_at: u64,
    ) -> GrowableRingAllocation<T> {
        let (buffer, offset, pointer) =
            self.staging
                .alloc(&self.gfx.device, None, count, align, free_at);
        let size = (count * std::mem::size_of::<T>()) as u64;
        GrowableRingAllocation {
            buffer,
            offset,
            size,
            pointer,
        }
    }

    pub fn alloc_vertices(&self, num_vertices: usize) -> BufferRegionAlloc {
        self.gfx.shader_data.vertex_alloc.lock().unwrap().alloc(
            &self.gfx.device,
            (size_of::<meshes::Vertex>() * num_vertices) as vk::DeviceSize,
            4,
        )
    }

    pub fn alloc_indices(&self, num_indices: usize) -> BufferRegionAlloc {
        self.gfx.shader_data.index_alloc.lock().unwrap().alloc(
            &self.gfx.device,
            (size_of::<u32>() * num_indices) as vk::DeviceSize,
            4,
        )
    }

    pub fn shader_data(&self) -> &ShaderData {
        &self.gfx.shader_data
    }

    pub fn device(&self) -> &ash::Device {
        self.gfx.device.as_ref()
    }

    pub fn memory_properties(&self) -> &vk::PhysicalDeviceMemoryProperties {
        &self.gfx.memory_properties
    }

    pub async fn wait_for_completion(&self, semaphore_value: u64) {
        // To actually get the work to start, we need to unpark the queue. We do it here to avoid getting stuck awaiting something we never kicked off.
        unsafe { self.queue_unparker.unpark_queue(&self.gfx.device) };

        self
            .queue_watch_receiver
            .clone()
            .wait_for(|&value| value >= semaphore_value)
            .await
            .expect("queue_watch_sender should not be dropped until there are no more AssetLoadContexts");
    }

    pub fn find_asset(&self, path: &Path) -> Option<PathBuf> {
        self.config.find_asset(path)
    }
}

/// Convenience wrapper around lahar::GrowableRing's allocation tuple
pub struct GrowableRingAllocation<T> {
    pub buffer: vk::Buffer,
    pub offset: u64,
    pub size: u64,
    pub pointer: NonNull<T>,
}

pub struct AssetLoader {
    gfx: Arc<Base>,
    queue_shutdown_token: CancellationToken,
    loader: skid_steer::Loader,
    staging: Arc<GrowableRing>,
    queue_unparker: Arc<QueueUnparker>,
    task_executor_threads: Vec<JoinHandle<()>>,
    queue_driver_thread: Option<JoinHandle<()>>,
}

impl AssetLoader {
    pub fn new(gfx: Arc<Base>, config: Arc<Config>) -> Self {
        let loader = skid_steer::Loader::new();
        let queue_shutdown_token = CancellationToken::new();
        let queue = unsafe { ParallelQueue::new(&gfx.device, gfx.queue_family, gfx.queue, None) };
        let staging = Arc::new(GrowableRing::new(
            &gfx.device,
            gfx.memory_properties,
            None,
            32 * 1024 * 1024,
        ));
        let queue_unparker = Arc::new(QueueUnparker::new(&gfx.device));
        let (queue_watch_sender, queue_watch_receiver) = tokio::sync::watch::channel(0);

        let mut task_executor_threads = vec![];
        tracing::debug!(
            "Using asset load parallelism {}",
            config.asset_load_parallelism
        );
        for i in 0..(config.asset_load_parallelism) {
            let asset_load_context = AssetLoadContext {
                gfx: Arc::clone(&gfx),
                config: Arc::clone(&config),
                queue_handle: unsafe { queue.handle(&gfx.device) },
                queue_watch_receiver: queue_watch_receiver.clone(),
                queue_unparker: Arc::clone(&queue_unparker),
                staging: Arc::clone(&staging),
            };
            let loader = loader.clone();

            let thread = thread::Builder::new()
                .name(format!("task_executor_{}", i).to_owned())
                .spawn(move || run_task_executor_thread(asset_load_context, loader))
                .unwrap();
            task_executor_threads.push(thread);
        }

        let queue_driver = QueueDriver {
            gfx: Arc::clone(&gfx),
            queue,
            queue_unpark_semaphore: queue_unparker.semaphore(),
            queue_shutdown_token: queue_shutdown_token.clone(),
            queue_watch_sender: queue_watch_sender.clone(),
            staging: Arc::clone(&staging),
        };

        let queue_driver_thread = thread::Builder::new()
            .name("queue_driver".to_owned())
            .spawn(move || {
                queue_driver.run();
            })
            .unwrap();

        AssetLoader {
            gfx,
            queue_shutdown_token,
            loader,
            staging,
            queue_unparker,
            task_executor_threads,
            queue_driver_thread: Some(queue_driver_thread),
        }
    }

    pub fn load<S: skid_steer::Source>(
        &self,
        source: S,
    ) -> skid_steer::Asset<<S as skid_steer::Source>::Output> {
        self.loader.load(source)
    }
}

struct QueueUnparker {
    semaphore_value: std::sync::Mutex<u64>,
    semaphore: vk::Semaphore,
}

impl QueueUnparker {
    fn new(device: &ash::Device) -> Self {
        QueueUnparker {
            semaphore_value: std::sync::Mutex::new(0),
            semaphore: unsafe {
                device
                    .create_semaphore(
                        &vk::SemaphoreCreateInfo::default().push_next(
                            &mut vk::SemaphoreTypeCreateInfo::default()
                                .semaphore_type(vk::SemaphoreType::TIMELINE)
                                .initial_value(0),
                        ),
                        None,
                    )
                    .unwrap()
            },
        }
    }

    /// Safety: The device passed in must match the device passed in to `new`
    unsafe fn unpark_queue(&self, device: &ash::Device) {
        let mut semaphore_value = self.semaphore_value.lock().unwrap();
        *semaphore_value += 1;

        // Safety: The `semaphore_value` lock is held while the semaphore is updated, so that should ensure that it is always
        // signaled with a strictly increasing value
        unsafe {
            device
                .signal_semaphore(
                    &vk::SemaphoreSignalInfo::default()
                        .semaphore(self.semaphore)
                        .value(*semaphore_value),
                )
                .unwrap()
        };
    }

    fn semaphore(&self) -> vk::Semaphore {
        self.semaphore
    }

    /// Safety: The device passed in must match the device passed in to `new`. Also, the semaphore must
    /// no longer be in use. This also means that it is unsound to call `unpark_queue` after calling `destroy`
    unsafe fn destroy(&self, device: &ash::Device) {
        unsafe { device.destroy_semaphore(self.semaphore, None) };
    }
}

/// Runs one thread of the task executor logic. This executor owns an [`AssetLoadContext`] and
/// concurrently runs tasks from the [`skid_steer::Loader`]. Returns the [`parallel_queue::Handle`]
/// from the [`AssetLoadContext`] for later cleanup.
fn run_task_executor_thread(asset_load_context: AssetLoadContext, loader: skid_steer::Loader) {
    let mut asset_load_context = Rc::new(asset_load_context);

    tokio::runtime::LocalRuntime::new()
        .unwrap()
        .block_on(async {
            let mut join_set = tokio::task::JoinSet::new();
            while let Some(task) = loader.next_task().await {
                tracing::trace!(
                    "Found task on {}",
                    thread::current().name().unwrap_or("<unnamed>")
                );
                let asset_load_context = Rc::clone(&asset_load_context);
                join_set.spawn_local(async move {
                    let mut context = Context::new();
                    context.insert::<AssetLoadContext>(&asset_load_context);
                    task.run(&context).await;
                    tracing::trace!(
                        "Task complete on {}",
                        thread::current().name().unwrap_or("<unnamed>")
                    );
                });
                while join_set.try_join_next().is_some() {} // Drain the join set to avoid memory leaks
            }
            tracing::trace!(
                "Ending task executor {}",
                thread::current().name().unwrap_or("<unnamed>")
            );
            join_set.join_all().await; // Since a bug can result in a deadlock here, we log before and after this call.
            tracing::trace!(
                "Task executor ended successfully {}",
                thread::current().name().unwrap_or("<unnamed>")
            );
        });
    let asset_load_context = Rc::get_mut(&mut asset_load_context)
        .expect("runtime using this context should already be dropped");

    // Safety: We make sure not to destroy the handle until we drain it. Since there are no other references to the handle,
    // no work will be in flight when the handle is destroyed.
    unsafe {
        // Fail-safe to ensure that the parallel queue is driven at least once after all work has been sent for submission before
        // we drain the handle
        asset_load_context
            .queue_unparker
            .unpark_queue(asset_load_context.device());

        asset_load_context
            .queue_handle
            .drain(&asset_load_context.gfx.device);
        asset_load_context
            .queue_handle
            .destroy(&asset_load_context.gfx.device);
    };
}

struct QueueDriver {
    gfx: Arc<Base>,
    queue: ParallelQueue,
    queue_unpark_semaphore: vk::Semaphore,
    queue_shutdown_token: CancellationToken,
    queue_watch_sender: tokio::sync::watch::Sender<u64>,
    staging: Arc<GrowableRing>,
}

impl QueueDriver {
    pub fn run(mut self) {
        loop {
            // Systems increment the `queue_unpark_semaphore` value when they want to guarantee
            // that we don't park unless certain things are done. `AssetLoadContext::wait_for_completion`
            // wants to ensure that `ParallelQueue::drive` is called, while the cleanup code
            // wants to ensure `queue_shutdown_token` is checked. Therefore, we put these two
            // operations between reading and waiting on `queue_unpark_semaphore`
            let queue_unpark_semaphore_current_value = unsafe {
                self.gfx
                    .device
                    .get_semaphore_counter_value(self.queue_unpark_semaphore)
                    .unwrap()
            };
            unsafe { self.queue.drive(&self.gfx.device) };
            if self.queue_shutdown_token.is_cancelled() {
                break;
            }
            let semaphore_value = unsafe {
                self.queue.park(
                    &self.gfx.device,
                    self.queue_unpark_semaphore,
                    queue_unpark_semaphore_current_value + 1,
                )
            };
            let _ = self.queue_watch_sender.send(semaphore_value);
            unsafe { self.staging.tick(&self.gfx.device, semaphore_value) };
        }
        unsafe { self.queue.drain(&self.gfx.device) };
        unsafe { self.queue.destroy(&self.gfx.device) };
    }
}

impl Drop for AssetLoader {
    fn drop(&mut self) {
        tracing::trace!("Shutting down AssetLoader");
        pollster::block_on(async {
            self.loader.drain().await;
            self.loader.clear_cache();
            self.loader.drain().await;
        });
        assert!(self.loader.all_assets_freed());
        self.loader.close();
        for thread in self.task_executor_threads.drain(..) {
            thread.join().unwrap();
        }
        self.queue_shutdown_token.cancel();

        unsafe { self.queue_unparker.unpark_queue(&self.gfx.device) };
        self.queue_driver_thread.take().unwrap().join().unwrap();

        unsafe { self.queue_unparker.destroy(&self.gfx.device) };

        unsafe {
            Arc::get_mut(&mut self.staging)
                .expect("All threads using staging should now be joined")
                .destroy(&self.gfx.device);
        }
    }
}

#[cfg(test)]
mod tests {
    use std::{collections::HashSet, sync::Mutex, time::Duration};

    use super::*;

    /// Record-keeping for what has happened so far with the asset loader, useful for assertions
    #[derive(Debug, Clone)]
    struct EventList {
        events: Arc<Mutex<Vec<Event>>>,
        num_checked_events: usize,
        event_received: Arc<std::sync::Condvar>,
    }

    impl EventList {
        fn new() -> Self {
            EventList {
                events: Arc::new(Mutex::new(vec![])),
                num_checked_events: 0,
                event_received: Arc::new(std::sync::Condvar::new()),
            }
        }

        fn push_event(&self, event: Event) {
            self.events.lock().unwrap().push(event);
            self.event_received.notify_all();
        }

        fn get_all_events(&mut self) -> Vec<Event> {
            let events = self.events.lock().unwrap().clone();
            self.num_checked_events = events.len();
            events
        }

        /// Wait until an event has been received since the last call to get_all_events
        fn wait_timeout_while(
            &self,
            timeout: std::time::Duration,
            mut condition: impl FnMut(&[Event]) -> bool,
        ) -> std::sync::WaitTimeoutResult {
            let events = self.events.lock().unwrap();
            self.event_received
                .wait_timeout_while(events, timeout, |events| condition(events))
                .unwrap()
                .1
        }

        /// Drains all events that were returned by the most recent call to get_all_events
        fn drain_queried_events(&mut self) {
            self.events
                .lock()
                .unwrap()
                .drain(0..self.num_checked_events);
            self.num_checked_events = 0;
        }
    }

    #[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
    enum Event {
        Progress {
            asset: String,
            percent_progress: u32,
        },
        Loaded {
            asset: String,
        },
        Freed {
            asset: String,
        },
        LoadCanceled {
            asset: String,
        },
    }

    impl Event {
        fn progress(asset: &str, percent_progress: u32) -> Self {
            Event::Progress {
                asset: asset.to_owned(),
                percent_progress,
            }
        }

        fn loaded(asset: &str) -> Self {
            Event::Loaded {
                asset: asset.to_owned(),
            }
        }

        fn freed(asset: &str) -> Self {
            Event::Freed {
                asset: asset.to_owned(),
            }
        }

        fn load_cancelled(asset: &str) -> Self {
            Event::LoadCanceled {
                asset: asset.to_owned(),
            }
        }
    }

    struct TestAsset<T: Send + 'static> {
        asset: skid_steer::Asset<T>,
        progress_sender: tokio::sync::mpsc::UnboundedSender<u32>,
    }

    impl<T: Send + 'static> TestAsset<T> {
        fn add_percent_progress(&self, percent_progress: u32) {
            self.progress_sender.send(percent_progress).unwrap();
        }

        fn wait_for_completion(&self) {
            // Note: This design pattern is somewhat dangerous because it can panic if called from within another runtime.
            // Since this is only used in unit tests, this should be an acceptable level of risk.
            tokio::runtime::LocalRuntime::new()
                .unwrap()
                .block_on(async {
                    tokio::select! {
                        biased;
                        _ = self.asset.get() => (),
                        _ = tokio::time::sleep(Duration::from_secs(5)) => { panic!("Timed out waiting for asset to load") },
                    };
                });
        }
    }

    struct DummyAsset {
        name: String,
        events: EventList,
    }

    struct DummyAssetSource {
        name: String,
        events: EventList,
        progress_receiver: tokio::sync::mpsc::UnboundedReceiver<u32>,
    }

    impl skid_steer::Source for DummyAssetSource {
        type Output = DummyAsset;

        async fn load(mut self, context: &Context<'_>) -> Option<Self::Output> {
            let mut status = DummyAssetStatus {
                name: self.name.clone(),
                progress: 0,
                can_cancel: true,
                events: self.events.clone(),
            };

            // Use the parallel queue and set up an allocation to exercise this functionality
            let ctx: &AssetLoadContext = context.get().unwrap();
            let work = unsafe { ctx.begin_work() };
            let finish_time = work.time().get();
            let _alloc = ctx.alloc_staging::<u8>(8, 1, finish_time);

            while status.progress < 100 {
                status.progress += self.progress_receiver.recv().await?;
                self.events
                    .push_event(Event::progress(&self.name, status.progress));
            }

            work.end();
            ctx.wait_for_completion(finish_time).await;

            status.can_cancel = false; // Done loading
            self.events.push_event(Event::loaded(&self.name));
            Some(DummyAsset {
                name: self.name,
                events: self.events,
            })
        }

        fn free(output: Self::Output, _context: &Context) {
            output.events.push_event(Event::freed(&output.name));
        }
    }

    struct DummyAssetStatus {
        name: String,
        progress: u32,
        can_cancel: bool,
        events: EventList,
    }

    impl Drop for DummyAssetStatus {
        fn drop(&mut self) {
            if self.can_cancel {
                self.events.push_event(Event::load_cancelled(&self.name));
            }
        }
    }

    fn init_asset_loader(asset_load_parallelism: u32) -> AssetLoader {
        let gfx = Arc::new(Base::headless());
        let config = Arc::new({
            let mut config = Config::create_for_test();
            config.asset_load_parallelism = asset_load_parallelism;
            config
        });
        AssetLoader::new(Arc::clone(&gfx), Arc::clone(&config))
    }

    fn load_dummy_asset(
        asset_loader: &AssetLoader,
        events: &EventList,
        name: &str,
    ) -> TestAsset<DummyAsset> {
        let (progress_sender, progress_receiver) = tokio::sync::mpsc::unbounded_channel();
        TestAsset {
            asset: asset_loader.load(DummyAssetSource {
                name: name.to_owned(),
                events: events.clone(),
                progress_receiver,
            }),
            progress_sender,
        }
    }

    #[test]
    fn test_load_and_free() {
        let mut events = EventList::new();
        let asset_loader = init_asset_loader(2);
        let dummy_asset = load_dummy_asset(&asset_loader, &events, "asset");
        dummy_asset.add_percent_progress(50);
        dummy_asset.add_percent_progress(50);
        dummy_asset.wait_for_completion();
        assert!(dummy_asset.asset.try_get().is_some());
        assert_eq!(
            events.get_all_events(),
            &[
                Event::progress("asset", 50),
                Event::progress("asset", 100),
                Event::loaded("asset")
            ]
        );
        events.drain_queried_events();
        drop(dummy_asset);
        drop(asset_loader);
        assert_eq!(events.get_all_events(), &[Event::freed("asset")]);
    }

    #[test]
    fn test_concurrency_and_cancellation() {
        let mut events = EventList::new();
        let asset_loader = init_asset_loader(2);
        let assets: Vec<_> = (0..4)
            .map(|i| load_dummy_asset(&asset_loader, &events, &format!("asset{i}")))
            .collect();
        for asset in &assets {
            asset.add_percent_progress(50);
        }
        let expected_events = [
            Event::progress("asset0", 50),
            Event::progress("asset1", 50),
            Event::progress("asset2", 50),
            Event::progress("asset3", 50),
        ]
        .into_iter()
        .collect::<HashSet<_>>();
        events.wait_timeout_while(Duration::from_secs(5), |events| {
            events.len() < expected_events.len()
        });
        let actual_events = events.get_all_events().into_iter().collect::<HashSet<_>>();
        assert_eq!(actual_events, expected_events);
    }
}
