use std::collections::VecDeque;

use common::{math, proto::Position};

/// Predicts the result of motion inputs in-flight to the server
///
/// When sending input to the server, call `push` to record the input in a local queue of in-flight
/// inputs, and to obtaining a generation tag to send alongside the input. The server echos the
/// highest tag it's received alongside every state update, which we then use in `reconcile` to
/// determine which inputs have been integrated into the server's state and no longer need to be
/// predicted.
pub struct PredictedMotion {
    log: VecDeque<Input>,
    generation: u16,
    predicted: Position,
}

impl PredictedMotion {
    pub fn new(initial: Position) -> Self {
        Self {
            log: VecDeque::new(),
            generation: 0,
            predicted: initial,
        }
    }

    /// Update for input about to be sent to the server, returning the generation it should be
    /// tagged with
    pub fn push(&mut self, velocity: &na::Vector3<f32>) -> u16 {
        let transform = math::translate_along(velocity);
        self.predicted.local *= transform;
        self.log.push_back(Input { transform });
        self.generation = self.generation.wrapping_add(1);
        self.generation
    }

    /// Update with the latest state received from the server and the generation it was based on
    pub fn reconcile(&mut self, generation: u16, position: Position) {
        let first_gen = self.generation.wrapping_sub(self.log.len() as u16);
        let obsolete = usize::from(generation.wrapping_sub(first_gen));
        if obsolete > self.log.len() || obsolete == 0 {
            // We've already processed a state incorporating equal or more recent input
            return;
        }
        self.log.drain(..obsolete);
        self.predicted.node = position.node;
        self.predicted.local = self
            .log
            .iter()
            .fold(position.local, |acc, x| acc * x.transform);
    }

    /// Latest estimate of the server's state after receiving all `push`ed inputs.
    pub fn predicted(&self) -> &Position {
        &self.predicted
    }
}

struct Input {
    transform: na::Matrix4<f32>,
}

#[cfg(test)]
mod tests {
    use super::*;

    /// An arbitrary position
    fn pos() -> Position {
        Position {
            node: common::graph::NodeId::ROOT,
            local: na::one(),
        }
    }

    #[test]
    fn wraparound() {
        let mut pred = PredictedMotion::new(pos());
        pred.generation = u16::max_value() - 1;
        assert_eq!(pred.push(&na::Vector3::x()), u16::max_value());
        assert_eq!(pred.push(&na::Vector3::x()), 0);
        assert_eq!(pred.log.len(), 2);

        pred.reconcile(u16::max_value() - 1, pos());
        assert_eq!(pred.log.len(), 2);
        pred.reconcile(u16::max_value(), pos());
        assert_eq!(pred.log.len(), 1);
        pred.reconcile(0, pos());
        assert_eq!(pred.log.len(), 0);
    }
}
