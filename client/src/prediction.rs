use std::collections::VecDeque;

use common::{
    SimConfig, character_controller,
    graph::Graph,
    proto::{CharacterInput, Position},
};

/// Predicts the result of motion inputs in-flight to the server
///
/// When sending input to the server, call `push` to record the input in a local queue of in-flight
/// inputs, and to obtaining a generation tag to send alongside the input. The server echos the
/// highest tag it's received alongside every state update, which we then use in `reconcile` to
/// determine which inputs have been integrated into the server's state and no longer need to be
/// predicted.
pub struct PredictedMotion {
    log: VecDeque<CharacterInput>,
    generation: u16,
    predicted_position: Position,
    predicted_velocity: na::Vector3<f32>,
    predicted_on_ground: bool,
}

impl PredictedMotion {
    pub fn new(initial_position: Position) -> Self {
        Self {
            log: VecDeque::new(),
            generation: 0,
            predicted_position: initial_position,
            predicted_velocity: na::Vector3::zeros(),
            predicted_on_ground: false,
        }
    }

    /// Update for input about to be sent to the server, returning the generation it should be
    /// tagged with
    pub fn push(&mut self, cfg: &SimConfig, graph: &Graph, input: &CharacterInput) -> u16 {
        character_controller::run_character_step(
            cfg,
            graph,
            &mut self.predicted_position,
            &mut self.predicted_velocity,
            &mut self.predicted_on_ground,
            input,
            cfg.step_interval.as_secs_f32(),
        );
        self.log.push_back(input.clone());
        self.generation = self.generation.wrapping_add(1);
        self.generation
    }

    /// Update with the latest state received from the server and the generation it was based on
    pub fn reconcile(
        &mut self,
        cfg: &SimConfig,
        graph: &Graph,
        generation: u16,
        position: Position,
        velocity: na::Vector3<f32>,
        on_ground: bool,
    ) {
        let first_gen = self.generation.wrapping_sub(self.log.len() as u16);
        let obsolete = usize::from(generation.wrapping_sub(first_gen));
        if obsolete > self.log.len() || obsolete == 0 {
            // We've already processed a state incorporating equal or more recent input
            return;
        }
        self.log.drain(..obsolete);
        self.predicted_position = position;
        self.predicted_velocity = velocity;
        self.predicted_on_ground = on_ground;

        for input in self.log.iter() {
            character_controller::run_character_step(
                cfg,
                graph,
                &mut self.predicted_position,
                &mut self.predicted_velocity,
                &mut self.predicted_on_ground,
                input,
                cfg.step_interval.as_secs_f32(),
            );
        }
    }

    /// Latest estimate of the server's state after receiving all `push`ed inputs.
    pub fn predicted_position(&self) -> &Position {
        &self.predicted_position
    }

    pub fn predicted_velocity(&self) -> &na::Vector3<f32> {
        &self.predicted_velocity
    }

    pub fn predicted_on_ground(&self) -> &bool {
        &self.predicted_on_ground
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::{graph::NodeId, math::MIsometry};

    /// An arbitrary position
    fn pos() -> Position {
        Position {
            node: common::graph::NodeId::ROOT,
            local: MIsometry::identity(),
        }
    }

    #[test]
    fn wraparound() {
        let mock_cfg = SimConfig::from_raw(&common::SimConfigRaw::default());
        let mut mock_graph = Graph::new(1);
        mock_graph.ensure_node_state(NodeId::ROOT);
        let mock_character_input = CharacterInput {
            movement: na::Vector3::x(),
            jump: false,
            no_clip: true,
            block_update: None,
        };

        let mut pred = PredictedMotion::new(pos());

        // Helper functions to make test more readable
        let push =
            |pred: &mut PredictedMotion| pred.push(&mock_cfg, &mock_graph, &mock_character_input);
        let reconcile = |pred: &mut PredictedMotion, generation| {
            pred.reconcile(
                &mock_cfg,
                &mock_graph,
                generation,
                pos(),
                na::Vector3::zeros(),
                false,
            )
        };

        pred.generation = u16::MAX - 1;

        assert_eq!(push(&mut pred), u16::MAX);
        assert_eq!(push(&mut pred), 0);
        assert_eq!(pred.log.len(), 2);

        reconcile(&mut pred, u16::MAX - 1);
        assert_eq!(pred.log.len(), 2);
        reconcile(&mut pred, u16::MAX);
        assert_eq!(pred.log.len(), 1);
        reconcile(&mut pred, 0);
        assert_eq!(pred.log.len(), 0);
    }
}
