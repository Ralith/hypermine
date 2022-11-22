use crate::tessellation::ChunkData;

pub trait ChunkRayTracer {
    fn trace_ray_in_chunk(
        &self,
        chunk_data: ChunkData,
        pos: &na::Vector3<f32>,
        dir: &na::Vector3<f32>,
        handle: &mut RayTracingResultHandle,
    );

    fn max_radius(&self) -> f32;
}

pub struct RayTracingResult {
    pub t: f32,
    pub intersection: Option<RayTracingIntersection>,
}

impl RayTracingResult {
    pub fn new(t: f32) -> Self {
        RayTracingResult {
            t,
            intersection: None,
        }
    }
}

pub struct RayTracingIntersection {
    pub voxel_coords: [usize; 2],
    pub normal: na::Vector3<f32>,
}

pub struct RayTracingResultHandle<'a> {
    result: &'a mut RayTracingResult,
    transform: na::Matrix3<f32>,
}

impl<'a> RayTracingResultHandle<'a> {
    pub fn t(&self) -> f32 {
        self.result.t
    }

    pub fn new(result: &'a mut RayTracingResult, transform: na::Matrix3<f32>) -> Self {
        RayTracingResultHandle { result, transform }
    }

    pub fn update(&mut self, t: f32, voxel_coords: [usize; 2], normal: na::Vector3<f32>) {
        self.result.t = t;
        self.result.intersection = Some(RayTracingIntersection {
            voxel_coords,
            normal: self.transform * normal,
        });
    }

    pub fn dependent_handle(&mut self, transform: na::Matrix3<f32>) -> RayTracingResultHandle {
        RayTracingResultHandle {
            result: self.result,
            transform: self.transform * transform,
        }
    }
}
