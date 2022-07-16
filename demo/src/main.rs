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

struct State {}

impl State {
    pub fn new(_ctx: &mut Context) -> State {
        State {}
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
