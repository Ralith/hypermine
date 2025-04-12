use std::time::Instant;

use common::{
    dodeca::Vertex,
    graph::{Graph, NodeId},
    node::{Chunk, ChunkId, VoxelData},
    proto::{BlockUpdate, Position},
    traversal,
};
use fxhash::FxHashMap;
use metrics::histogram;
use tokio::sync::mpsc;

pub struct WorldgenDriver {
    work_queue: WorkQueue,
    /// Voxel data that have been downloaded from the server for chunks not yet introduced to the graph
    preloaded_block_updates: FxHashMap<ChunkId, Vec<BlockUpdate>>,
    /// Voxel data that has been fetched from the server but not yet introduced to the graph
    preloaded_voxel_data: FxHashMap<ChunkId, VoxelData>,
}

impl WorldgenDriver {
    pub fn new(chunk_load_parallelism: usize) -> Self {
        Self {
            work_queue: WorkQueue::new(chunk_load_parallelism),
            preloaded_block_updates: FxHashMap::default(),
            preloaded_voxel_data: FxHashMap::default(),
        }
    }

    pub fn drive(&mut self, view: Position, chunk_generation_distance: f32, graph: &mut Graph) {
        let drive_worldgen_started = Instant::now();

        // Check for chunks that have finished generating
        while let Some(chunk) = self.work_queue.poll() {
            self.add_chunk_to_graph(graph, ChunkId::new(chunk.node, chunk.chunk), chunk.voxels);
        }

        if !graph.contains(view.node) {
            // Graph is temporarily out of sync with the server; we don't know where we are, so
            // there's no point trying to generate chunks.
            return;
        }

        let nearby_nodes = traversal::nearby_nodes(graph, &view, chunk_generation_distance);

        'nearby_nodes: for &(node, _) in &nearby_nodes {
            for vertex in Vertex::iter() {
                let chunk_id = ChunkId::new(node, vertex);

                if !matches!(graph[chunk_id], Chunk::Fresh) {
                    continue;
                }

                let Some(params) =
                    common::worldgen::ChunkParams::new(graph.layout().dimension(), graph, chunk_id)
                else {
                    continue;
                };

                if let Some(voxel_data) = self.preloaded_voxel_data.remove(&chunk_id) {
                    self.add_chunk_to_graph(graph, chunk_id, voxel_data);
                } else if self.work_queue.load(ChunkDesc { node, params }) {
                    graph[chunk_id] = Chunk::Generating;
                } else {
                    // No capacity is available in the work queue. Stop trying to prepare chunks to generate.
                    break 'nearby_nodes;
                }
            }
        }
        histogram!("frame.cpu.drive_worldgen").record(drive_worldgen_started.elapsed());
    }

    /// Adds established voxel data to the graph. This could come from world generation or sent from the server,
    /// depending on whether the chunk has been modified.
    pub fn add_chunk_to_graph(
        &mut self,
        graph: &mut Graph,
        chunk_id: ChunkId,
        voxel_data: VoxelData,
    ) {
        graph.populate_chunk(chunk_id, voxel_data);

        if let Some(block_updates) = self.preloaded_block_updates.remove(&chunk_id) {
            for block_update in block_updates {
                // The chunk was just populated, so a block update should always succeed.
                assert!(graph.update_block(&block_update));
            }
        }
    }

    pub fn apply_block_update(&mut self, graph: &mut Graph, block_update: BlockUpdate) {
        if graph.update_block(&block_update) {
            return;
        }
        self.preloaded_block_updates
            .entry(block_update.chunk_id)
            .or_default()
            .push(block_update);
    }

    pub fn apply_voxel_data(
        &mut self,
        graph: &mut Graph,
        chunk_id: ChunkId,
        voxel_data: VoxelData,
    ) {
        if graph.contains(chunk_id.node) {
            self.add_chunk_to_graph(graph, chunk_id, voxel_data);
        } else {
            self.preloaded_voxel_data.insert(chunk_id, voxel_data);
        }
    }
}

struct ChunkDesc {
    node: NodeId,
    params: common::worldgen::ChunkParams,
}

struct LoadedChunk {
    node: NodeId,
    chunk: Vertex,
    voxels: VoxelData,
}

struct WorkQueue {
    _runtime: tokio::runtime::Runtime,
    send: tokio::sync::mpsc::Sender<ChunkDesc>,
    recv: tokio::sync::mpsc::Receiver<LoadedChunk>,
    capacity: usize,
    fill: usize,
}

impl WorkQueue {
    pub fn new(chunk_load_parallelism: usize) -> Self {
        let runtime = tokio::runtime::Builder::new_multi_thread().build().unwrap();

        let (input_send, mut input_recv) = mpsc::channel::<ChunkDesc>(chunk_load_parallelism);
        let (output_send, output_recv) = mpsc::channel::<LoadedChunk>(chunk_load_parallelism);
        runtime.spawn(async move {
            while let Some(x) = input_recv.recv().await {
                let out = output_send.clone();
                tokio::spawn(async move {
                    let loaded_chunk = LoadedChunk {
                        node: x.node,
                        chunk: x.params.chunk(),
                        voxels: x.params.generate_voxels(),
                    };
                    let _ = out.send(loaded_chunk).await;
                });
            }
        });

        Self {
            _runtime: runtime,
            send: input_send,
            recv: output_recv,
            capacity: chunk_load_parallelism,
            fill: 0,
        }
    }

    /// Begin loading a single item, if capacity is available
    #[must_use]
    pub fn load(&mut self, x: ChunkDesc) -> bool {
        if self.fill == self.capacity {
            return false;
        }
        if self.send.try_send(x).is_ok() {
            self.fill += 1;
            true
        } else {
            false
        }
    }

    /// Fetch a load result if one is ready, freeing capacity
    pub fn poll(&mut self) -> Option<LoadedChunk> {
        let result = self.recv.try_recv().ok()?;
        self.fill -= 1;
        Some(result)
    }
}
