use rand::{
    distributions::{Distribution, Standard},
    Rng,
};

pub struct World {}

impl World {
    pub fn generate(&self, tile: Tile, out: &mut [Material]) {
        assert_eq!(out.len(), SUBDIVISION_FACTOR.pow(3));
        let fill = if tile.z >= 0 {
            Material::Void
        } else {
            Material::Stone
        };
        for x in out {
            *x = fill;
        }
    }
}

impl Default for World {
    fn default() -> Self {
        Self {}
    }
}

pub const SUBDIVISION_FACTOR: usize = 16;

impl Distribution<World> for Standard {
    fn sample<R: Rng + ?Sized>(&self, _rng: &mut R) -> World {
        World {}
    }
}

/// A quaternary tile in H^3
#[derive(Debug, Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct Tile {
    x: i32,
    y: i32,
    z: i32,
}

impl Tile {
    pub fn new(x: i32, y: i32, z: i32) -> Self {
        Self { x, y, z }
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum Material {
    Void = 0,
    Stone = 1,
}

impl Default for Material {
    fn default() -> Self {
        Material::Void
    }
}
