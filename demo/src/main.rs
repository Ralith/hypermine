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

    fn get_voxel_mesh(&self, ctx: &mut Context) -> GameResult<graphics::Mesh> {
        let transform = self
            .tessellation
            .voxel_to_hyperboloid(&NodeString::default(), Vertex::AB);

        let resolution = 4;
        let vertices: Vec<graphics::Vertex> = (0..resolution)
            .flat_map(|x| {
                (0..resolution).flat_map(move |y| {
                    let x_min = x as f64 / resolution as f64;
                    let x_max = (x + 1) as f64 / resolution as f64;
                    let y_min = y as f64 / resolution as f64;
                    let y_max = (y + 1) as f64 / resolution as f64;
                    let color = if (x + y) % 2 == 0 {
                        [0.8, 0.8, 0.8, 1.0]
                    } else {
                        [0.9, 0.9, 0.9, 1.0]
                    };
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
    fn update(&mut self, _ctx: &mut Context) -> GameResult<()> {
        Ok(())
    }

    fn draw(&mut self, ctx: &mut Context) -> GameResult<()> {
        graphics::clear(ctx, graphics::Color::new(0.2, 0.2, 0.2, 1.0));
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

        let circle = graphics::Mesh::new_circle(
            ctx,
            graphics::DrawMode::fill(),
            mint::Point2 { x: 0.0, y: 0.0 },
            1.0,
            0.1 / scale,
            graphics::Color::BLACK,
        )?;
        graphics::draw(ctx, &circle, draw_params)?;

        let test = self.get_voxel_mesh(ctx)?;
        graphics::draw(ctx, &test, draw_params)?;

        graphics::present(ctx)
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
