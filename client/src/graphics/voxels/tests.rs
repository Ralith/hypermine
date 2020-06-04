use std::{mem, sync::Arc};

use ash::{version::DeviceV1_0, vk};
use lahar::DedicatedMapping;
use renderdoc::{RenderDoc, V110};

use super::{surface_extraction, SurfaceExtraction};
use crate::graphics::Base;
use common::world::Material;

struct SurfaceExtractionTest {
    gfx: Arc<Base>,
    extract: SurfaceExtraction,
    scratch: surface_extraction::ScratchBuffer,
    indirect: DedicatedMapping<VkDrawIndirectCommand>,
    vertices: DedicatedMapping<[Vertex]>,
    cmd_pool: vk::CommandPool,
    cmd: vk::CommandBuffer,
    rd: Option<RenderDoc<V110>>,
}

impl SurfaceExtractionTest {
    pub fn new() -> Self {
        let gfx = Arc::new(Base::headless());
        let extract = SurfaceExtraction::new(&gfx);
        let scratch = surface_extraction::ScratchBuffer::new(&gfx, &extract, 1, DIMENSION as u32);

        let device = &*gfx.device;

        unsafe {
            let indirect = DedicatedMapping::<VkDrawIndirectCommand>::zeroed(
                device,
                &gfx.memory_properties,
                vk::BufferUsageFlags::STORAGE_BUFFER,
            );

            let vertices = DedicatedMapping::<[Vertex]>::zeroed_array(
                device,
                &gfx.memory_properties,
                vk::BufferUsageFlags::STORAGE_BUFFER,
                3 * (DIMENSION.pow(3) + DIMENSION.pow(2)),
            );

            let cmd_pool = device
                .create_command_pool(
                    &vk::CommandPoolCreateInfo::builder()
                        .queue_family_index(gfx.queue_family)
                        .flags(vk::CommandPoolCreateFlags::RESET_COMMAND_BUFFER),
                    None,
                )
                .unwrap();

            let cmd = device
                .allocate_command_buffers(
                    &vk::CommandBufferAllocateInfo::builder()
                        .command_pool(cmd_pool)
                        .command_buffer_count(1),
                )
                .unwrap()[0];

            Self {
                gfx,
                extract,
                scratch,
                indirect,
                vertices,
                cmd_pool,
                cmd,
                rd: RenderDoc::new().ok(),
            }
        }
    }

    fn run(&mut self) {
        let device = &*self.gfx.device;

        if let Some(ref mut rd) = self.rd {
            rd.start_frame_capture(std::ptr::null(), std::ptr::null());
        }

        unsafe {
            device
                .begin_command_buffer(
                    self.cmd,
                    &vk::CommandBufferBeginInfo::builder()
                        .flags(vk::CommandBufferUsageFlags::ONE_TIME_SUBMIT),
                )
                .unwrap();

            self.scratch.extract(
                device,
                &self.extract,
                0,
                false,
                DIMENSION as u32,
                self.cmd,
                (self.indirect.buffer(), 0),
                (self.vertices.buffer(), 0),
            );
            device.end_command_buffer(self.cmd).unwrap();

            device
                .queue_submit(
                    self.gfx.queue,
                    &[vk::SubmitInfo::builder()
                        .command_buffers(&[self.cmd])
                        .build()],
                    vk::Fence::null(),
                )
                .unwrap();
            device.device_wait_idle().unwrap();
        }

        if let Some(ref mut rd) = self.rd {
            rd.end_frame_capture(std::ptr::null(), std::ptr::null());
        }
    }
}

impl Drop for SurfaceExtractionTest {
    fn drop(&mut self) {
        let device = &*self.gfx.device;
        unsafe {
            self.extract.destroy(device);
            self.scratch.destroy(device);
            self.indirect.destroy(device);
            self.vertices.destroy(device);
            device.destroy_command_pool(self.cmd_pool, None);
        }
    }
}

const DIMENSION: usize = 2;

#[repr(C)]
#[derive(Debug, Eq, PartialEq)]
struct VkDrawIndirectCommand {
    vertex_count: u32,
    instance_count: u32,
    first_vertex: u32,
    first_instance: u32,
}

#[repr(C)]
#[derive(Debug, Eq, PartialEq)]
struct Vertex {
    x: u8,
    y: u8,
    z: u8,
    axis: u8,
    mat: Material,
    _padding: u8,
    occlusion: u8,
}

#[test]
#[ignore]
fn surface_extraction() {
    assert_eq!(mem::size_of::<Vertex>(), 8);

    let _guard = common::tracing_guard();
    let mut test = SurfaceExtractionTest::new();

    for x in test.scratch.storage(0) {
        *x = Material::Void;
    }

    test.run();

    assert_eq!(
        test.indirect.vertex_count, 0,
        "empty chunks have no surfaces"
    );

    for x in test.scratch.storage(0) {
        *x = Material::Stone;
    }

    test.run();

    assert_eq!(
        test.indirect.vertex_count, 0,
        "solid chunks have no surfaces"
    );

    // TODO: Half-empty
    let storage = test.scratch.storage(0);
    for x in &mut storage[..] {
        *x = Material::Void;
    }
    for z in 0..((DIMENSION + 2) / 2) {
        for y in 0..(DIMENSION + 2) {
            for x in 0..(DIMENSION + 2) {
                storage[x + y * (DIMENSION + 2) + z * (DIMENSION + 2).pow(2)] = Material::Stone;
            }
        }
    }

    test.run();

    assert_eq!(
        test.indirect.vertex_count,
        6 * DIMENSION.pow(2) as u32,
        "half-solid chunks have n^2 surfaces"
    );
    assert_eq!(
        &test.vertices[..DIMENSION.pow(2)],
        &[
            Vertex {
                x: 0,
                y: 0,
                z: 1,
                axis: 5,
                mat: Material::Stone,
                _padding: 0,
                occlusion: 0xFF,
            },
            Vertex {
                x: 1,
                y: 0,
                z: 1,
                axis: 5,
                mat: Material::Stone,
                _padding: 0,
                occlusion: 0xFF,
            },
            Vertex {
                x: 0,
                y: 1,
                z: 1,
                axis: 5,
                mat: Material::Stone,
                _padding: 0,
                occlusion: 0xFF,
            },
            Vertex {
                x: 1,
                y: 1,
                z: 1,
                axis: 5,
                mat: Material::Stone,
                _padding: 0,
                occlusion: 0xFF,
            },
        ]
    )
}
