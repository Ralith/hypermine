use rand::{
    distributions::{Distribution, Standard},
    Rng,
};

#[macro_use]
mod id;

pub mod codec;
pub mod math;
pub mod proto;
pub mod world;

mkid!(EntityId: u64);

impl std::fmt::Display for EntityId {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "{:016x}", self.0)
    }
}

impl Distribution<EntityId> for Standard {
    fn sample<R: Rng + ?Sized>(&self, rng: &mut R) -> EntityId {
        EntityId(rng.gen())
    }
}

pub type Step = i32;

pub fn defer<F: FnOnce()>(f: F) -> Defer<F> {
    Defer::new(f)
}

pub struct Defer<F: FnOnce()>(Option<F>);

impl<F: FnOnce()> Defer<F> {
    pub fn new(f: F) -> Self {
        Self(Some(f))
    }

    pub fn invoke(self) {}

    pub fn cancel(mut self) {
        self.0 = None;
    }
}

impl<F: FnOnce()> Drop for Defer<F> {
    fn drop(&mut self) {
        if let Some(f) = self.0.take() {
            f()
        }
    }
}
