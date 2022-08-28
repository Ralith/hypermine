use demo::{
    draw, math::HyperboloidMatrix,
    player::{Player, PlayerInput},
    tessellation::Tessellation,
};
use ggez::{conf, event, graphics, timer, Context, ContextBuilder, GameResult};

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
    player: Player,
    tessellation: Tessellation,
}

impl State {
    pub fn new(_ctx: &mut Context) -> State {
        let tessellation = Tessellation::new(12);

        State {
            player: Player::new(tessellation.root()),
            tessellation,
        }
    }
}

impl event::EventHandler for State {
    fn update(&mut self, ctx: &mut Context) -> GameResult {
        const DESIRED_FPS: u32 = 60;

        while timer::check_update_time(ctx, DESIRED_FPS) {
            let seconds = 1. / (DESIRED_FPS as f64);
            self.player.step(&PlayerInput::new(ctx, &self.tessellation, seconds));
        }
        Ok(())
    }

    fn draw(&mut self, ctx: &mut Context) -> GameResult {
        let nodes =
            self.tessellation
                .ensure_nearby(self.tessellation.root(), na::Matrix3::identity(), 2);

        let mut pass = draw::RenderPass::new(ctx, &self.tessellation);
        pass.push_transform(&self.player.pos().iso_inverse(), 0);
        pass.draw_background()?;

        for handle in nodes {
            pass.push_transform(&handle.transform, 0);
            pass.draw_node_chunks(handle.node)?;
            pass.pop_transform();
        }

        pass.draw_center()?;
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
