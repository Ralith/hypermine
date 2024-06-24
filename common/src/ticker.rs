use serde::{Deserialize, Serialize};

#[derive(Debug, Serialize, Deserialize)]
pub struct Blinker {
    pub on: bool,
}

impl Blinker {
    pub fn new() -> Self {
        Self { on: false }
    }
}

impl Default for Blinker {
    fn default() -> Self {
        Blinker::new()
    }
}