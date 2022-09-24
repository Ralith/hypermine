use crate::{
    math::HyperboloidVector,
    penta::Vertex,
    tessellation::{NodeHandle, Tessellation},
};

pub fn is_colliding(tessellation: &Tessellation, node: NodeHandle, pos: &na::Vector3<f64>) -> bool {
    for vertex in Vertex::iter() {
        let chunk_data = tessellation.get_chunk_data(node, vertex);
        let float_size = chunk_data.chunk_size() as f64;
        let voxel_coords = ((vertex.penta_to_voxel() * pos).euclidean_point()
            * chunk_data.chunk_size() as f64)
            .map(|x| x.floor());

        if voxel_coords.iter().any(|&x| x < 0.0 || x >= float_size) {
            continue;
        }

        let voxel = chunk_data.get(voxel_coords.x as usize, voxel_coords.y as usize);
        return voxel != 0;
    }

    false
}

// Ray-tracing version
pub fn collision_point(
    tessellation: &Tessellation,
    node: NodeHandle,
    pos: &na::Vector3<f64>,
    dir: &na::Vector3<f64>,
) -> (f64, Option<na::Vector3<f64>>) {
    let mut t = 1.0;
    let mut normal: Option<na::Vector3<f64>> = None;
    for vertex in Vertex::iter() {
        if vertex != Vertex::AB {
            continue;
        }
        let chunk_data = tessellation.get_chunk_data(node, vertex);
        let float_size = chunk_data.chunk_size() as f64;
        let square_pos = vertex.penta_to_square() * pos;
        let square_dir = vertex.penta_to_square() * dir;
        let x_dir = square_dir
            .mip(&na::Vector3::new(square_pos.z, 0.0, square_pos.x))
            .signum();
        for i in 0..=chunk_data.chunk_size() {
            if i != 9 {
                continue;
            }
            // Factor a in plane equation x/z == a => x == a*z
            let a = i as f64 / float_size * Vertex::voxel_to_square_factor();
            // Solve for t: (square_pos + square_dir*t).x == a * (square_pos + square_dir*t).z
            // square_pos.x - square_pos.z * a == -t * (square_dir.x - square_dir.z * a)
            let t_candidate =
                -(square_pos.x - square_pos.z * a) / (square_dir.x - square_dir.z * a);
            if t_candidate >= 0.0 && t_candidate < t && x_dir > 0.0 {
                t = t_candidate;
                normal = Some((vertex.square_to_penta() * na::Vector3::new(1.0, 0.0, a)).m_normalized_vector());
            }
        }
    }

    (t, normal)
}
