use std::collections::{HashSet, VecDeque};

use crate::{
    math::{HyperboloidMatrix, HyperboloidVector},
    penta::Vertex,
    tessellation::{ChunkData, NodeHandle, Tessellation},
};

pub struct Collision {
    pub t: f64,
    pub normal: Option<na::Vector3<f64>>,
}

#[derive(Clone, Copy, PartialEq, Eq, Hash)]
struct ChunkHandle {
    node: NodeHandle,
    vertex: Vertex,
}

impl ChunkHandle {
    fn new(node: NodeHandle, vertex: Vertex) -> Self {
        ChunkHandle { node, vertex }
    }
}

// TODO: This function is outdated and is not as complete a check as collision_point.
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
    radius: f64,
    node: NodeHandle,
    pos: &na::Vector3<f64>,
    dir: &na::Vector3<f64>,
    t: f64,
) -> Collision {
    let mut visited_chunks: HashSet<ChunkHandle> = HashSet::new();
    let mut chunk_queue: VecDeque<(ChunkHandle, na::Matrix3<f64>)> = VecDeque::new();

    let start_chunk = ChunkHandle::new(node, Vertex::AB);
    visited_chunks.insert(start_chunk);
    chunk_queue.push_back((start_chunk, na::Matrix3::identity()));

    let mut collision = Collision { t, normal: None };

    let klein_boundary0 = radius.tanh();
    let klein_boundary1 = (Vertex::voxel_to_square_factor().atanh() - radius).tanh();

    while let Some((chunk, transform)) = chunk_queue.pop_front() {
        let chunk_data = tessellation.get_chunk_data(chunk.node, chunk.vertex);
        let square_pos = chunk.vertex.penta_to_square() * transform * pos;
        let square_dir = chunk.vertex.penta_to_square() * transform * dir;

        // TODO: Need to check collision with vertices as well as just edges
        for coord_axis in 0..2 {
            handle_basic_collision(
                chunk_data,
                radius,
                &square_pos,
                &square_dir,
                &mut collision,
                &(transform.iso_inverse() * chunk.vertex.square_to_penta()),
                coord_axis,
            );
        }

        handle_vertex_collision(
            chunk_data,
            radius,
            &square_pos,
            &square_dir,
            &mut collision,
            &(transform.iso_inverse() * chunk.vertex.square_to_penta()),
        );

        // If pos or pos+dir*max_t lies beyond the chunk boundary, with a buffer to account for radius, repeat
        // collision checking with the neighboring chunk unless it has already been visited. We start at vertex
        // AB for simplicity even if that's not where pos is, although this should be optimized later.
        for coord_boundary in 0..2 {
            let klein_pos0_val = square_pos[coord_boundary] / square_pos[2];
            let klein_pos1_val = (square_pos[coord_boundary]
                + square_dir[coord_boundary] * collision.t)
                / (square_pos[2] + square_dir[2] * collision.t);

            // Check for neighboring nodes. TODO: The use of unwrap here will cause a crash if you arrive at an ungenerated chunk.
            if klein_pos0_val <= klein_boundary0 || klein_pos1_val <= klein_boundary0 {
                let side = chunk.vertex.sides()[coord_boundary];
                let next_chunk = ChunkHandle::new(
                    tessellation.get_neighbor(chunk.node, side).unwrap(),
                    chunk.vertex,
                );
                if visited_chunks.insert(next_chunk) {
                    chunk_queue.push_back((next_chunk, side.reflection() * transform));
                }
            }

            // Check for neighboring chunks within the same node
            if klein_pos0_val >= klein_boundary1 || klein_pos1_val >= klein_boundary1 {
                let vertex = chunk.vertex.adjacent_vertices()[coord_boundary];
                let next_chunk = ChunkHandle::new(chunk.node, vertex);
                if visited_chunks.insert(next_chunk) {
                    chunk_queue.push_back((next_chunk, transform));
                }
            }
        }
    }

    collision
}

fn handle_basic_collision(
    chunk_data: ChunkData,
    radius: f64,
    square_pos: &na::Vector3<f64>,
    square_dir: &na::Vector3<f64>,
    collision: &mut Collision,
    collision_transform: &na::Matrix3<f64>,
    coord_axis: usize,
) {
    let float_size = chunk_data.chunk_size() as f64;
    let coord_plane0 = (coord_axis + 1) % 2;

    for i in 0..=chunk_data.chunk_size() {
        // Factor a in plane equation x/z == a => x == a*z
        let a = i as f64 / float_size * Vertex::voxel_to_square_factor();

        // Solve quadratic equation
        let mut normal = na::Vector3::new(0.0, 0.0, a);
        normal[coord_axis] = 1.0;
        let normal = normal.m_normalized_vector();
        let mip_pos_norm = square_pos.mip(&normal);
        let mip_dir_norm = square_dir.mip(&normal);
        let sinh_radius_squared = radius.sinh().powi(2);
        let quadratic_term = mip_dir_norm * mip_dir_norm + sinh_radius_squared;
        let double_linear_term = mip_pos_norm * mip_dir_norm;
        let constant_term = mip_pos_norm * mip_pos_norm - sinh_radius_squared;
        let discriminant = double_linear_term * double_linear_term - quadratic_term * constant_term;

        if discriminant < 0.0 {
            continue;
        }

        let t_candidate = (-double_linear_term - discriminant.sqrt()) / quadratic_term;

        if t_candidate >= 0.0 && t_candidate < collision.t {
            let i_with_offset = if mip_dir_norm < 0.0 { i } else { i + 1 };
            if i_with_offset > 0 && i_with_offset <= chunk_data.chunk_size() {
                let i_with_offset = i_with_offset - 1;
                let translated_square_pos = square_pos + square_dir * t_candidate;
                let b = translated_square_pos[coord_plane0] * (1.0 - a * a)
                    / (translated_square_pos.z - translated_square_pos[coord_axis] * a);
                let j = (b * Vertex::square_to_voxel_factor() * float_size).floor();
                if j >= 0.0 && j < float_size {
                    let j = j as usize;
                    if chunk_data.get2(coord_axis, i_with_offset, coord_plane0, j) != 0 {
                        collision.t = t_candidate;
                        collision.normal = Some(collision_transform * normal * -mip_dir_norm.signum());
                    }
                }
            }
        }
    }
}

fn handle_vertex_collision(
    chunk_data: ChunkData,
    radius: f64,
    square_pos: &na::Vector3<f64>,
    square_dir: &na::Vector3<f64>,
    collision: &mut Collision,
    collision_transform: &na::Matrix3<f64>,
) {
    let size = chunk_data.chunk_size();
    let float_size = size as f64;

    for i in 0..=chunk_data.chunk_size() {
        for j in 0..=chunk_data.chunk_size() {
            if (i == 0 || j == 0 || chunk_data.get(i - 1, j - 1) == 0)
                && (i == size || j == 0 || chunk_data.get(i, j - 1) == 0)
                && (i == 0 || j == size || chunk_data.get(i - 1, j) == 0)
                && (i == size || j == size || chunk_data.get(i, j) == 0)
            {
                continue;
            }

            let vert = na::Vector3::new(
                i as f64 / float_size * Vertex::voxel_to_square_factor(),
                j as f64 / float_size * Vertex::voxel_to_square_factor(),
                1.0,
            )
            .m_normalized_point();

            let mip_pos_vert = square_pos.mip(&vert);
            let mip_dir_vert = square_dir.mip(&vert);
            let cosh_radius_squared = radius.cosh().powi(2);
            let quadratic_term = mip_dir_vert * mip_dir_vert + cosh_radius_squared;
            let double_linear_term = mip_pos_vert * mip_dir_vert;
            let constant_term = mip_pos_vert * mip_pos_vert - cosh_radius_squared;
            let discriminant =
                double_linear_term * double_linear_term - quadratic_term * constant_term;

            if discriminant < 0.0 {
                continue;
            }

            let t_candidate = (-double_linear_term - discriminant.sqrt()) / quadratic_term;

            if t_candidate >= 0.0 && t_candidate < collision.t {
                let translated_square_pos = square_pos + square_dir * t_candidate;
                collision.t = t_candidate;
                collision.normal = Some(collision_transform * (translated_square_pos - vert));
            }
        }
    }
}
