use serde::{Deserialize, Serialize};
use crate::Step;

#[derive(Debug, Serialize, Deserialize)]
pub struct TickerEntity {
    pub last_ticked: Step,
    pub ticker: Ticker,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct Ticker {
    on: bool,
}

impl Ticker {
    pub fn new() -> Self {
        Self {
            on: false,
        }
    }
    pub fn tick(&mut self, _step: Step) {
        self.on = !self.on;

        // Just for testing
        if self.on {
            println!("Ticked ON!");
        } else {
            println!("Ticked OFF!");
        }
    }
}