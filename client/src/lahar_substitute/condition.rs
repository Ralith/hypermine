use std::task::{Context, Waker};

/// Manages tasks waiting on a single condition
pub struct Condition {
    wakers: Vec<Waker>,
    generation: u64,
}

impl Condition {
    pub fn new() -> Self {
        Self {
            wakers: Vec::new(),
            generation: 0,
        }
    }

    /// Ensure the next `wake` call will wake the calling task
    ///
    /// Checks the task-associated generation counter stored in `state`. If it's present and
    /// current, we already have this task's `Waker` and no action is necessary. Otherwise, record a
    /// `Waker` and store the current generation in `state`.
    pub fn register(&mut self, cx: &mut Context, state: &mut State) {
        if state.0 == Some(self.generation) {
            return;
        }
        state.0 = Some(self.generation);
        self.wakers.push(cx.waker().clone());
    }

    /// Wake all known tasks
    pub fn notify(&mut self) {
        self.generation = self.generation.wrapping_add(1);
        for waker in self.wakers.drain(..) {
            waker.wake();
        }
    }
}

/// State maintained by each interested task
///
/// Stores the generation at which the task previously registered a `Waker`, if any.
#[derive(Default)]
pub struct State(Option<u64>);
