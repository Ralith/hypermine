use ggez::{graphics, mint, Context, GameResult};

use crate::{
    math,
    node_string::{NodeString, Vertex},
    tessellation::Tessellation,
};

pub struct RenderPass<'a> {
    ctx: &'a mut Context,
    draw_params: graphics::DrawParam,
    scale: f32,
    stack: Vec<(na::Matrix3<f64>, u32)>,
    tessellation: &'a Tessellation,
}

impl<'a> RenderPass<'a> {
    pub fn new(ctx: &'a mut Context, tessellation: &'a Tessellation) -> RenderPass<'a> {
        let screen_coordinates = graphics::screen_coordinates(ctx);
        let width = screen_coordinates.w;
        let height = screen_coordinates.h;

        let scale = (width.min(height) - 1.0) / 2.0;

        let draw_params = graphics::DrawParam::default()
            .offset(mint::Point2 {
                x: -width / 2.0 / scale,
                y: height / 2.0 / scale,
            })
            .scale([scale, -scale]);

        RenderPass {
            ctx,
            draw_params,
            scale,
            stack: vec![(na::Matrix3::identity(), 0)],
            tessellation,
        }
    }

    pub fn push_transform(&mut self, transform: &na::Matrix3<f64>, node_parity: u32) {
        self.stack.push((
            self.get_transform() * transform,
            self.get_parity() ^ node_parity,
        ));
    }

    pub fn pop_transform(&mut self) {
        self.stack.pop();
    }

    fn get_transform(&self) -> &na::Matrix3<f64> {
        &self.stack.last().expect("Transformation stack underflow").0
    }

    fn get_parity(&self) -> u32 {
        self.stack.last().expect("Transformation stack underflow").1
    }

    pub fn draw_background(&mut self) -> GameResult {
        graphics::clear(self.ctx, graphics::Color::new(0.2, 0.2, 0.2, 1.0));
        let circle = graphics::Mesh::new_circle(
            self.ctx,
            graphics::DrawMode::fill(),
            mint::Point2 { x: 0.0, y: 0.0 },
            1.0,
            0.1 / self.scale,
            graphics::Color::BLACK,
        )?;
        graphics::draw(self.ctx, &circle, self.draw_params)
    }

    pub fn draw_node_chunks(&mut self) -> GameResult {
        self.draw_chunk(Vertex::AB)?;
        self.draw_chunk(Vertex::BC)?;
        self.draw_chunk(Vertex::CD)?;
        self.draw_chunk(Vertex::DE)?;
        self.draw_chunk(Vertex::EA)?;
        Ok(())
    }

    fn draw_chunk(&mut self, vertex: Vertex) -> GameResult {
        let transform = *self.get_transform();
        let parity = self.get_parity();
        let mesh = self.get_voxel_mesh(&transform, parity, vertex)?;
        graphics::draw(self.ctx, &mesh, self.draw_params)
    }

    pub fn present(&mut self) -> GameResult {
        graphics::present(self.ctx)
    }

    fn get_voxel_mesh(
        &mut self,
        transform: &na::Matrix3<f64>,
        node_parity: u32,
        vertex: Vertex,
    ) -> GameResult<graphics::Mesh> {
        let transform = transform
            * self
                .tessellation
                .voxel_to_hyperboloid(&NodeString::default(), vertex);

        let hue = match vertex {
            Vertex::AB => [1.0, 0.0, 0.0],
            Vertex::BC => [1.0, 0.8, 0.0],
            Vertex::CD => [0.0, 1.0, 0.4],
            Vertex::DE => [0.0, 0.4, 1.0],
            Vertex::EA => [0.8, 0.0, 1.0],
        };

        let on_color = [
            [
                1.0 - (1.0 - hue[0]) * (1.0 - 0.9),
                1.0 - (1.0 - hue[1]) * (1.0 - 0.9),
                1.0 - (1.0 - hue[2]) * (1.0 - 0.9),
                1.0,
            ],
            [
                1.0 - (1.0 - hue[0]) * (1.0 - 0.7),
                1.0 - (1.0 - hue[1]) * (1.0 - 0.7),
                1.0 - (1.0 - hue[2]) * (1.0 - 0.7),
                1.0,
            ],
        ];

        let resolution = 12;
        let vertices: Vec<graphics::Vertex> = (0..resolution)
            .flat_map(|x| {
                (0..resolution).flat_map(move |y| {
                    let x_min = x as f64 / resolution as f64;
                    let x_max = (x + 1) as f64 / resolution as f64;
                    let y_min = y as f64 / resolution as f64;
                    let y_max = (y + 1) as f64 / resolution as f64;
                    let color = on_color[((x + y + node_parity) % 2) as usize];
                    [
                        graphics::Vertex {
                            pos: math::to_point(&(transform * na::Vector3::new(x_min, y_min, 1.0))),
                            uv: [0., 0.],
                            color,
                        },
                        graphics::Vertex {
                            pos: math::to_point(&(transform * na::Vector3::new(x_max, y_min, 1.0))),
                            uv: [1., 0.],
                            color,
                        },
                        graphics::Vertex {
                            pos: math::to_point(&(transform * na::Vector3::new(x_max, y_max, 1.0))),
                            uv: [1., 1.],
                            color,
                        },
                        graphics::Vertex {
                            pos: math::to_point(&(transform * na::Vector3::new(x_min, y_max, 1.0))),
                            uv: [0., 1.],
                            color,
                        },
                    ]
                })
            })
            .collect();

        let indices: Vec<u32> = (0..vertices.len() as u32)
            .flat_map(|i| [i * 4, i * 4 + 1, i * 4 + 2, i * 4, i * 4 + 2, i * 4 + 3])
            .collect();

        graphics::MeshBuilder::new()
            .raw(&vertices, &indices, None)?
            .build(self.ctx)
    }
}
