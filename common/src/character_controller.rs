use crate::{
    graph::Graph,
    math,
    proto::{CharacterInput, Position},
    sanitize_motion_input, SimConfig,
};

pub fn run_character_step<T>(
    cfg: &SimConfig,
    graph: &Graph<T>,
    position: &mut Position,
    input: &CharacterInput,
    dt_seconds: f32,
) {
    CharacterControllerPass {
        cfg,
        graph,
        position,
        input,
        dt_seconds,
    }
    .step();
}

struct CharacterControllerPass<'a, T> {
    cfg: &'a SimConfig,
    graph: &'a Graph<T>,
    position: &'a mut Position,
    input: &'a CharacterInput,
    dt_seconds: f32,
}

impl<T> CharacterControllerPass<'_, T> {
    fn step(&mut self) {
        let movement = sanitize_motion_input(self.input.movement);

        self.position.local *= math::translate_along(
            &(movement * self.cfg.movement_speed * self.dt_seconds),
        );

        // Renormalize
        self.position.local = math::renormalize_isometry(&self.position.local);
        let (next_node, transition_xf) = self
            .graph
            .normalize_transform(self.position.node, &self.position.local);
        if next_node != self.position.node {
            self.position.node = next_node;
            self.position.local = transition_xf * self.position.local;
        }
    }
}
