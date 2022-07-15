use ggez::{
    event::{self, EventHandler},
    graphics::{self, Color},
    mint, Context, ContextBuilder, GameResult,
};

mod math;

fn main() {
    let (mut ctx, event_loop) = ContextBuilder::new("demo", "hypermine").build().unwrap();
    let demo = State::new(&mut ctx);
    event::run(ctx, event_loop, demo);
}

struct State {}

impl State {
    pub fn new(_ctx: &mut Context) -> State {
        State {}
    }
}

impl EventHandler for State {
    fn update(&mut self, _ctx: &mut Context) -> GameResult<()> {
        Ok(())
    }

    fn draw(&mut self, ctx: &mut Context) -> GameResult<()> {
        graphics::clear(ctx, Color::WHITE);
        let circle = graphics::Mesh::new_circle(
            ctx,
            graphics::DrawMode::fill(),
            mint::Point2 { x: 200.0, y: 300.0 },
            100.0,
            0.1,
            graphics::Color::BLACK,
        )?;
        graphics::draw(ctx, &circle, graphics::DrawParam::default())?;
        graphics::present(ctx)
    }
}
