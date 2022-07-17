use demo::{
    math,
    node_string::{NodeString, Vertex},
    tessellation::Tessellation,
};
use ggez::{conf, event, graphics, mint, Context, ContextBuilder, GameResult};

fn main() {
    let (mut ctx, event_loop) = ContextBuilder::new("demo", "hypermine")
        .window_mode(conf::WindowMode::default().resizable(true))
        .window_setup(conf::WindowSetup::default().title("Hypermine 2D concept demos"))
        .build()
        .unwrap();
    let demo = State::new(&mut ctx);
    event::run(ctx, event_loop, demo);
}

struct State {
    tessellation: Tessellation,
}

impl State {
    pub fn new(_ctx: &mut Context) -> State {
        State {
            tessellation: Tessellation::new(),
        }
    }

    fn get_voxel_mesh(
        &self,
        ctx: &mut Context,
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

        let resolution = 4;
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
            .build(ctx)
    }
}

impl event::EventHandler for State {
    fn update(&mut self, _ctx: &mut Context) -> GameResult {
        Ok(())
    }

    fn draw(&mut self, ctx: &mut Context) -> GameResult {
        let mut pass = RenderPass::new(ctx, self);
        pass.draw_background()?;
        pass.draw_chunk(Vertex::AB)?;
        pass.draw_chunk(Vertex::BC)?;
        pass.draw_chunk(Vertex::CD)?;
        pass.draw_chunk(Vertex::DE)?;
        pass.draw_chunk(Vertex::EA)?;
        pass.present()
    }

    fn resize_event(&mut self, ctx: &mut Context, width: f32, height: f32) {
        graphics::set_screen_coordinates(
            ctx,
            graphics::Rect {
                x: 0.0,
                y: 0.0,
                w: width.max(1.0),
                h: height.max(1.0),
            },
        )
        .unwrap();
    }
}

struct RenderPass<'a> {
    state: &'a State,
    ctx: &'a mut Context,
    draw_params: graphics::DrawParam,
    scale: f32,
}

impl<'a> RenderPass<'a> {
    fn new(ctx: &'a mut Context, state: &'a State) -> RenderPass<'a> {
        let screen_coordinates = graphics::screen_coordinates(ctx);
        let width = screen_coordinates.w;
        let height = screen_coordinates.h;

        let scale = (width.min(height) - 1.0) / 2.0;

        let draw_params = graphics::DrawParam::default()
            .offset(mint::Point2 {
                x: -width / 2.0 / scale,
                y: -height / 2.0 / scale,
            })
            .scale([scale, scale]);

        RenderPass {
            state,
            ctx,
            draw_params,
            scale,
        }
    }

    fn draw_background(&mut self) -> GameResult {
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

    fn draw_chunk(&mut self, vertex: Vertex) -> GameResult {
        let mesh = self
            .state
            .get_voxel_mesh(self.ctx, &na::Matrix3::identity(), 0, vertex)?;
        graphics::draw(self.ctx, &mesh, self.draw_params)
    }

    fn present(&mut self) -> GameResult {
        graphics::present(self.ctx)
    }
}
