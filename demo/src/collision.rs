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
) -> f64 {
    // TODO: Set up tests to ensure epsilon is large enough. 1e-8 seems to work so far.
    const EPSILON_LARGE: f64 = 2e-8;

    let mut t = 1.0;
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
        let klein_x_with_dir = square_pos.x / square_pos.z * x_dir;
        for i in 0..=chunk_data.chunk_size() {
            if i != 9 {
                continue;
            }
            // Factor a in plane equation x/z == a => x == a*z
            let a = i as f64 / float_size * Vertex::voxel_to_square_factor();
            let a_with_epsilon = a - x_dir * EPSILON_LARGE;
            let t_candidate =
                if klein_x_with_dir < a * x_dir && klein_x_with_dir >= a_with_epsilon * x_dir {
                    // t is within the epsilon buffer, so no movement closer to the wall is allowed.
                    0.0
                } else {
                    // Solve for t: (square_pos + square_dir*t).x == a * (square_pos + square_dir*t).z
                    // square_pos.x - square_pos.z * a == -t * (square_dir.x - square_dir.z * a)
                    -(square_pos.x - square_pos.z * a_with_epsilon)
                        / (square_dir.x - square_dir.z * a_with_epsilon)
                };
            if t_candidate >= 0.0 && t_candidate < t && x_dir > 0.0 {
                t = t_candidate;
            }
        }
    }

    t
}
