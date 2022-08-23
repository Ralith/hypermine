use demo::{draw, math, tessellation::Tessellation};
use ggez::{conf, event, graphics, input, timer, Context, ContextBuilder, GameResult};

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
            tessellation: Tessellation::new(12),
        }
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
            let mut x_velocity =
                if left_pressed { -1. } else { 0. } + if right_pressed { 1. } else { 0. };
            let mut y_velocity =
                if down_pressed { -1. } else { 0. } + if up_pressed { 1. } else { 0. };
            let sqr_velocity_magnitude: f64 = x_velocity * x_velocity + y_velocity * y_velocity;
            if sqr_velocity_magnitude > 1. {
                let factor = 1. / sqr_velocity_magnitude.sqrt();
                x_velocity *= factor;
                y_velocity *= factor;
            }
            self.pos *= math::displacement(&na::Vector3::new(
                x_velocity * seconds,
                y_velocity * seconds,
                0.,
            ));
            math::qr_normalize(&mut self.pos);
        }
        Ok(())
    }

    fn draw(&mut self, ctx: &mut Context) -> GameResult {
        let nodes = self.tessellation.ensure_nearby(self.tessellation.root(), na::Matrix3::identity(), 2);

        let mut pass = draw::RenderPass::new(ctx, &self.tessellation);
        pass.push_transform(&math::iso_inverse(&self.pos), 0);
        pass.draw_background()?;

        for (_node, transform) in nodes {
            pass.push_transform(&transform, 0);
            pass.draw_node_chunks()?;
            pass.pop_transform();
        }

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
