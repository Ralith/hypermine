/// Predicts the result of motion inputs in-flight to the server
///
/// When sending input to the server, call `push` to record the input in a local queue of in-flight
/// inputs, and to obtaining a generation tag to send alongside the input. The server echos the
/// highest tag it's received alongside every state update, which we then use in `reconcile` to
/// determine which inputs have been integrated into the server's state and no longer need to be
/// predicted.
pub struct PredictedMotion {
    generation: u16,
}

impl PredictedMotion {
    pub fn new() -> Self {
        Self { generation: 0 }
    }

    /// Update for input about to be sent to the server, returning the generation it should be
    /// tagged with
    pub fn push(&mut self) -> u16 {
        self.generation = self.generation.wrapping_add(1);
        self.generation
    }
}
