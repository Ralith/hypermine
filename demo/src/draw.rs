use ggez::{graphics, mint, Context, GameResult};

use crate::{
    color::Color,
    penta::Vertex,
    player::Player,
    tessellation::{NodeHandle, Tessellation},
};

pub struct RenderPass<'a> {
    ctx: &'a mut Context,
    draw_params: graphics::DrawParam,
    scale: f32,
    stack: Vec<na::Matrix3<f64>>,
    tessellation: &'a Tessellation,
}

impl<'a> RenderPass<'a> {
    pub fn new(
        ctx: &'a mut Context,
        tessellation: &'a Tessellation,
        zoom_factor: f64,
    ) -> RenderPass<'a> {
        let screen_coordinates = graphics::screen_coordinates(ctx);
        let width = screen_coordinates.w;
        let height = screen_coordinates.h;

        let scale = (width.min(height) - 1.0) / 2.0 * zoom_factor as f32;

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
            stack: vec![na::Matrix3::identity()],
            tessellation,
        }
    }

    pub fn push_transform(&mut self, transform: &na::Matrix3<f64>) {
        self.stack.push(self.get_transform() * transform);
    }

    pub fn pop_transform(&mut self) {
        self.stack.pop();
    }

    fn get_transform(&self) -> &na::Matrix3<f64> {
        self.stack.last().expect("Transformation stack underflow")
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

    pub fn draw_node_chunks(&mut self, node: NodeHandle) -> GameResult {
        self.draw_chunk(node, Vertex::AB)?;
        self.draw_chunk(node, Vertex::BC)?;
        self.draw_chunk(node, Vertex::CD)?;
        self.draw_chunk(node, Vertex::DE)?;
        self.draw_chunk(node, Vertex::EA)?;
        Ok(())
    }

    pub fn draw_player(&mut self, player: &Player) -> GameResult {
        let circle = graphics::Mesh::new_circle(
            self.ctx,
            graphics::DrawMode::fill(),
            mint::Point2 { x: 0.0, y: 0.0 },
            player.radius().tanh() as f32,
            0.1 / self.scale,
            graphics::Color::from_rgb(128, 128, 128),
        )?;
        graphics::draw(self.ctx, &circle, self.draw_params)
    }

    fn draw_chunk(&mut self, node: NodeHandle, vertex: Vertex) -> GameResult {
        let mesh = self.get_voxel_mesh(node, vertex)?;
        graphics::draw(self.ctx, &mesh, self.draw_params)
    }

    pub fn present(&mut self) -> GameResult {
        graphics::present(self.ctx)
    }

    fn get_voxel_mesh(&mut self, node: NodeHandle, vertex: Vertex) -> GameResult<graphics::Mesh> {
        let transform = self.get_transform() * vertex.voxel_to_penta();

        let parity = self.tessellation.parity(node) as usize;

        let chunk_data = self.tessellation.chunk_data(node, vertex);

        let hue = match vertex {
            Vertex::AB => Color::from_hue(0.0),
            Vertex::BC => Color::from_hue(0.2),
            Vertex::CD => Color::from_hue(0.4),
            Vertex::DE => Color::from_hue(0.6),
            Vertex::EA => Color::from_hue(0.8),
        };

        let inverted_hue = hue.inverted();
        let hue_normalized = hue / hue.luminance();
        let inverted_hue_normalized = inverted_hue / inverted_hue.luminance();
        let on_color = [
            (Color::WHITE - inverted_hue_normalized * 0.1).into(),
            (Color::WHITE - inverted_hue_normalized * 0.22).into(),
        ];
        let off_color = [
            (hue_normalized * 0.0005).into(),
            (hue_normalized * 0.001).into(),
        ];

        let resolution = 12;
        let vertices: Vec<graphics::Vertex> = (0..resolution)
            .flat_map(|x| {
                (0..resolution).flat_map(move |y| {
                    let x_min = x as f64 / resolution as f64;
                    let x_max = (x + 1) as f64 / resolution as f64;
                    let y_min = y as f64 / resolution as f64;
                    let y_max = (y + 1) as f64 / resolution as f64;
                    let voxel_type = chunk_data.get([x, y]);
                    let color =
                        (if voxel_type != 0 { on_color } else { off_color })[(x + y + parity) % 2];
                    [
                        graphics::Vertex {
                            pos: Self::to_point(&(transform * na::Vector3::new(x_min, y_min, 1.0))),
                            uv: [0., 0.],
                            color,
                        },
                        graphics::Vertex {
                            pos: Self::to_point(&(transform * na::Vector3::new(x_max, y_min, 1.0))),
                            uv: [1., 0.],
                            color,
                        },
                        graphics::Vertex {
                            pos: Self::to_point(&(transform * na::Vector3::new(x_max, y_max, 1.0))),
                            uv: [1., 1.],
                            color,
                        },
                        graphics::Vertex {
                            pos: Self::to_point(&(transform * na::Vector3::new(x_min, y_max, 1.0))),
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

    fn to_point(v: &na::Vector3<f64>) -> [f32; 2] {
        [(v[0] / v[2]) as f32, (v[1] / v[2]) as f32]
    }
}
