use crate::{
    math,
    penta::Vertex,
    tessellation::{NodeHandle, Tessellation},
};

pub fn is_colliding(tessellation: &Tessellation, node: NodeHandle, v: &na::Vector3<f64>) -> bool {
    for vertex in Vertex::iter() {
        let chunk_data = tessellation.get_chunk_data(node, vertex);
        let float_size = chunk_data.chunk_size() as f64;
        let voxel_coords = (math::euclidean_point(&(tessellation.hyperboloid_to_voxel(vertex) * v))
            * chunk_data.chunk_size() as f64).map(|x| x.floor());

        if voxel_coords.iter().any(|&x| x < 0.0 || x >= float_size) {
            continue;
        }

        let voxel = chunk_data.get(voxel_coords.x as usize, voxel_coords.y as usize);
        return voxel != 0;
    }

    false
}
