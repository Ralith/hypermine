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
        if vertex != Vertex::AB {
            continue;
        }
        let chunk_data = tessellation.get_chunk_data(node, vertex);
        let square_pos = vertex.penta_to_square() * pos;
        let square_dir = vertex.penta_to_square() * dir;
        let x_dir = if square_dir
            .mip(&na::Vector3::new(square_pos.z, 0.0, square_pos.x))
            .signum()
            > 0.0
        {
            1
        } else {
            0
        };
        let y_dir = if square_dir
            .mip(&na::Vector3::new(0.0, square_pos.z, square_pos.y))
            .signum()
            > 0.0
        {
            1
        } else {
            0
        };
        handle_basic_collision(
            &chunk_data,
            &square_pos,
            &square_dir,
            &mut collision,
            vertex.square_to_penta(),
            0,
            x_dir,
        );
        handle_basic_collision(
            &chunk_data,
            &square_pos,
            &square_dir,
            &mut collision,
            vertex.square_to_penta(),
            1,
            y_dir,
        );
    }

    collision
}

fn handle_basic_collision(
    chunk_data: &ChunkData,
    square_pos: &na::Vector3<f64>,
    square_dir: &na::Vector3<f64>,
    collision: &mut Collision,
    collision_transform: &na::Matrix3<f64>,
    coord: usize,
    coord_dir: usize,
) {
    let float_size = chunk_data.chunk_size() as f64;

    for i in 0..chunk_data.chunk_size() {
        // Factor a in plane equation x/z == a => x == a*z
        let a = (i + (coord_dir + 1) % 2) as f64 / float_size * Vertex::voxel_to_square_factor();
        // Solve for t: (square_pos + square_dir*t).x == a * (square_pos + square_dir*t).z
        // square_pos.x - square_pos.z * a == -t * (square_dir.x - square_dir.z * a)
        let t_candidate =
            -(square_pos[coord] - square_pos.z * a) / (square_dir[coord] - square_dir.z * a);
        if t_candidate >= 0.0 && t_candidate < collision.t {
            let b = (square_pos[(coord + 1) % 2] + square_dir[(coord + 1) % 2] * collision.t)
                / (square_pos.z + square_dir.z * collision.t);
            let j = (b * Vertex::square_to_voxel_factor() * float_size).floor();
            if j >= 0.0 && j < float_size {
                let j = j as usize;
                if coord == 0 && chunk_data.get(i, j) != 0
                    || coord == 1 && chunk_data.get(j, i) != 0
                {
                    collision.t = t_candidate;
                    if coord == 0 {
                        collision.normal = Some(
                            collision_transform
                                * na::Vector3::new(1.0, 0.0, a).m_normalized_vector(),
                        );
                    } else {
                        collision.normal = Some(
                            collision_transform
                                * na::Vector3::new(0.0, 1.0, a).m_normalized_vector(),
                        );
                    }
                }
            }
        }
    }
}
