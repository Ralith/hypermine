use serde::{Deserialize, Serialize};
use hecs::Entity;
use crate::{Step, EntityId};

#[derive(Debug, Serialize, Deserialize)]
pub struct TickerEntity {
    pub lastTicked: Step,
    pub tickerID: u16, // This ID determines what *type* of ticker this is (blinker, sapling, etc)
    pub tickerUID: EntityId,
    pub ticker: Ticker,
}

impl TickerEntity {
    pub fn setUID(&mut self, id: EntityId) {
        self.tickerUID = id;
    }
    pub fn setLastTicked(&mut self, step: Step) {
        self.lastTicked = step;
    }
    pub fn setID(&mut self, id: u16) {
        self.tickerID = id;
    }
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