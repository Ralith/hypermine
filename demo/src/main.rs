use demo::{
    math,
    node_string::{NodeString, Side, Vertex},
    tessellation::Tessellation,
};
use ggez::{conf, event, graphics, input, mint, Context, ContextBuilder, GameResult, timer};

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
    pos: na::Matrix3<f64>,
    tessellation: Tessellation,
}

impl State {
    pub fn new(_ctx: &mut Context) -> State {
        State {
            pos: na::Matrix3::identity(),
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
            .build(ctx)
    }
}

impl event::EventHandler for State {
    fn update(&mut self, ctx: &mut Context) -> GameResult {
        const DESIRED_FPS: u32 = 60;

        while timer::check_update_time(ctx, DESIRED_FPS) {
            let seconds = 1. / (DESIRED_FPS as f64);

            let left_pressed = input::keyboard::is_key_pressed(ctx, event::KeyCode::A);
            let right_pressed = input::keyboard::is_key_pressed(ctx, event::KeyCode::D);
            let down_pressed = input::keyboard::is_key_pressed(ctx, event::KeyCode::S);
            let up_pressed = input::keyboard::is_key_pressed(ctx, event::KeyCode::W);
            let mut x_velocity = if left_pressed { -1. } else { 0. } + if right_pressed { 1. } else { 0. };
            let mut y_velocity = if down_pressed { -1. } else { 0. } + if up_pressed { 1. } else { 0. };
            let sqr_velocity_magnitude: f64 = x_velocity * x_velocity + y_velocity * y_velocity;
            if sqr_velocity_magnitude > 1. {
                let factor = 1. / sqr_velocity_magnitude.sqrt();
                x_velocity *= factor;
                y_velocity *= factor;
            }
            self.pos *= math::displacement(&na::Vector3::new(x_velocity * seconds, y_velocity * seconds, 0.));
            math::qr_normalize(&mut self.pos);
        }
        Ok(())
    }

    fn draw(&mut self, ctx: &mut Context) -> GameResult {
        let mut pass = RenderPass::new(ctx, self);
        pass.push_transform(&math::iso_inverse(&self.pos), 0);
        pass.draw_background()?;
        pass.draw_node_chunks()?;
        pass.push_transform(self.tessellation.reflection(Side::A), 1);
        pass.draw_node_chunks()?;
        pass.push_transform(self.tessellation.reflection(Side::B), 1);
        pass.draw_node_chunks()?;
        pass.pop_transform();
        pass.pop_transform();
        pass.push_transform(self.tessellation.reflection(Side::B), 1);
        pass.draw_node_chunks()?;
        pass.pop_transform();
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
    stack: Vec<(na::Matrix3<f64>, u32)>,
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
                y: height / 2.0 / scale,
            })
            .scale([scale, -scale]);

        RenderPass {
            state,
            ctx,
            draw_params,
            scale,
            stack: vec![(na::Matrix3::identity(), 0)],
        }
    }

    fn push_transform(&mut self, transform: &na::Matrix3<f64>, node_parity: u32) {
        self.stack.push((
            self.get_transform() * transform,
            self.get_parity() ^ node_parity,
        ));
    }

    fn pop_transform(&mut self) {
        self.stack.pop();
    }

    fn get_transform(&self) -> &na::Matrix3<f64> {
        &self.stack.last().expect("Transformation stack underflow").0
    }

    fn get_parity(&self) -> u32 {
        self.stack.last().expect("Transformation stack underflow").1
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

    fn draw_node_chunks(&mut self) -> GameResult {
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
        let mesh = self
            .state
            .get_voxel_mesh(self.ctx, &transform, parity, vertex)?;
        graphics::draw(self.ctx, &mesh, self.draw_params)
    }

    fn present(&mut self) -> GameResult {
        graphics::present(self.ctx)
    }
}
