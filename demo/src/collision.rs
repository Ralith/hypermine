use crate::{
    math::HyperboloidVector,
    penta::Vertex,
    tessellation::{ChunkData, NodeHandle, Tessellation},
};

pub struct Collision {
    pub t: f64,
    pub normal: Option<na::Vector3<f64>>,
}

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
) -> Collision {
    let mut collision = Collision {
        t: 1.0,
        normal: None,
    };
    for vertex in Vertex::iter() {
        let chunk_data = tessellation.get_chunk_data(node, vertex);
        let square_pos = vertex.penta_to_square() * pos;
        let square_dir = vertex.penta_to_square() * dir;

        for coord_axis in 0..2 {
            handle_basic_collision(
                chunk_data,
                &square_pos,
                &square_dir,
                &mut collision,
                vertex.square_to_penta(),
                coord_axis,
            );
        }
    }

    collision.normal = collision.normal.map(|n| n.m_normalized_vector());
    collision
}

fn handle_basic_collision(
    chunk_data: ChunkData,
    square_pos: &na::Vector3<f64>,
    square_dir: &na::Vector3<f64>,
    collision: &mut Collision,
    collision_transform: &na::Matrix3<f64>,
    coord_axis: usize,
) {
    let float_size = chunk_data.chunk_size() as f64;
    let coord_plane0 = (coord_axis + 1) % 2;
    let mip_dir_axis_sign =
        (square_dir[coord_axis] * square_pos.z - square_dir.z * square_pos[coord_axis]).signum();
    let i_offset = 0.5 - mip_dir_axis_sign * 0.5;

    for i in 0..chunk_data.chunk_size() {
        // Factor a in plane equation x/z == a => x == a*z
        let a = (i as f64 + i_offset) / float_size * Vertex::voxel_to_square_factor();
        // Solve for t: (square_pos + square_dir*t).x == a * (square_pos + square_dir*t).z
        // square_pos.x - square_pos.z * a == -t * (square_dir.x - square_dir.z * a)
        let t_candidate = -(square_pos[coord_axis] - square_pos.z * a)
            / (square_dir[coord_axis] - square_dir.z * a);
        if t_candidate >= 0.0 && t_candidate < collision.t {
            let b = (square_pos[coord_plane0] + square_dir[coord_plane0] * t_candidate)
                / (square_pos.z + square_dir.z * t_candidate);
            let j = (b * Vertex::square_to_voxel_factor() * float_size).floor();
            if j >= 0.0 && j < float_size {
                let j = j as usize;
                if chunk_data.get2(coord_axis, i, coord_plane0, j) != 0 {
                    collision.t = t_candidate;
                    let mut normal = na::Vector3::new(0.0, 0.0, a);
                    normal[coord_axis] = 1.0;
                    collision.normal = Some(collision_transform * normal);
                }
            }
        }
    }
}
