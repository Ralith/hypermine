use common::{node::VoxelData, world::Material};

#[derive(Copy, Clone)]
pub struct VoxelDataWrapper<'a> {
    voxel_data: &'a VoxelData,
    dimension: usize,
}

impl VoxelDataWrapper<'_> {
    pub fn new(voxel_data: &VoxelData, dimension: usize) -> VoxelDataWrapper {
        VoxelDataWrapper {
            voxel_data,
            dimension,
        }
    }

    pub fn dimension(&self) -> usize {
        self.dimension
    }

    pub fn get(&self, coords: [usize; 3]) -> Material {
        assert!(coords[0] < self.dimension);
        assert!(coords[1] < self.dimension);
        assert!(coords[2] < self.dimension);
        let dimension_with_margin = self.dimension + 2;
        self.voxel_data.get(
            (coords[0] + 1)
                + (coords[1] + 1) * dimension_with_margin
                + (coords[2] + 1) * dimension_with_margin.pow(2),
        )
    }
}

pub trait ChunkRayTracer {
    fn trace_ray_in_chunk(
        &self,
        chunk_data: VoxelDataWrapper,
        pos: &na::Vector4<f64>,
        dir: &na::Vector4<f64>,
        handle: &mut RayTracingResultHandle,
    );

    fn max_radius(&self) -> f64;
}

pub struct RayTracingResult {
    pub t: f64,
    pub intersection: Option<RayTracingIntersection>,
}

impl RayTracingResult {
    pub fn new(t: f64) -> Self {
        RayTracingResult {
            t,
            intersection: None,
        }
    }
}

pub struct RayTracingIntersection {
    pub voxel_coords: [usize; 3],
    pub normal: na::Vector4<f64>,
}

pub struct RayTracingResultHandle<'a> {
    result: &'a mut RayTracingResult,
    transform: na::Matrix4<f64>,
}

impl<'a> RayTracingResultHandle<'a> {
    pub fn t(&self) -> f64 {
        self.result.t
    }

    pub fn new(result: &'a mut RayTracingResult, transform: na::Matrix4<f64>) -> Self {
        RayTracingResultHandle { result, transform }
    }

    pub fn update(&mut self, t: f64, voxel_coords: [usize; 3], normal: na::Vector4<f64>) {
        self.result.t = t;
        self.result.intersection = Some(RayTracingIntersection {
            voxel_coords,
            normal: self.transform * normal,
        });
    }

    pub fn dependent_handle(&mut self, transform: na::Matrix4<f64>) -> RayTracingResultHandle {
        RayTracingResultHandle {
            result: self.result,
            transform: self.transform * transform,
        }
    }
}
