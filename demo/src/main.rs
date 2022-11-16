use demo::{
    draw,
    math::HyperboloidMatrix,
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
    zoom_factor: f64,
}

impl State {
    pub fn new(_ctx: &mut Context) -> State {
        let tessellation = Tessellation::new(12);

        State {
            player: Player::new(tessellation.root()),
            tessellation,
            zoom_factor: 1.0,
        }
    }

    fn get_mouse_position(&self, ctx: &Context) -> na::Vector3<f64> {
        let screen_coordinates = graphics::screen_coordinates(ctx);
        let width = screen_coordinates.w as f64;
        let height = screen_coordinates.h as f64;

        let scale = (width.min(height) - 1.0) / 2.0 * self.zoom_factor;

        let window_mouse_position = ggez::input::mouse::position(ctx);
        na::Vector3::new(
            (window_mouse_position.x as f64 + 0.5 - width / 2.0) / scale,
            -(window_mouse_position.y as f64 + 0.5 - height / 2.0) / scale,
            1.0,
        )
    }
}

impl event::EventHandler for State {
    fn update(&mut self, ctx: &mut Context) -> GameResult {
        const DESIRED_FPS: u32 = 60;

        while timer::check_update_time(ctx, DESIRED_FPS) {
            let seconds = 1. / (DESIRED_FPS as f64);
            self.player
                .step(&PlayerInput::new(ctx, &self.tessellation, seconds));

            if ggez::input::keyboard::is_key_pressed(ctx, ggez::event::KeyCode::R) {
                self.zoom_factor *= seconds.exp();
            }
            if ggez::input::keyboard::is_key_pressed(ctx, ggez::event::KeyCode::F) {
                self.zoom_factor /= seconds.exp();
            }

            if ggez::input::mouse::button_pressed(ctx, ggez::event::MouseButton::Left) {
                let mouse_position = self.get_mouse_position(ctx);
                if let Some((mut chunk_data, [x, y])) = self
                    .tessellation
                    .get_voxel_at_pos(self.player.node(), self.player.pos() * mouse_position)
                {
                    chunk_data.set(x, y, 1);
                }
            } else if ggez::input::mouse::button_pressed(ctx, ggez::event::MouseButton::Right) {
                let mouse_position = self.get_mouse_position(ctx);
                if let Some((mut chunk_data, [x, y])) = self
                    .tessellation
                    .get_voxel_at_pos(self.player.node(), self.player.pos() * mouse_position)
                {
                    chunk_data.set(x, y, 0);
                }
            }
        }

        self.tessellation
            .ensure_nearby(self.player.node(), na::Matrix3::identity(), 2);
        Ok(())
    }

    fn draw(&mut self, ctx: &mut Context) -> GameResult {
        let nodes = self
            .tessellation
            .get_nearby(self.player.node(), na::Matrix3::identity(), 2);

        let mut pass = draw::RenderPass::new(ctx, &self.tessellation, self.zoom_factor);
        pass.push_transform(&self.player.pos().iso_inverse());
        pass.draw_background()?;

        for handle in nodes {
            pass.push_transform(&handle.transform);
            pass.draw_node_chunks(handle.node)?;
            pass.pop_transform();
        }

        pass.draw_player(&self.player)?;
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
