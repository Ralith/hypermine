use rand::{
    distributions::{Distribution, Standard},
    Rng,
};

pub struct World {
    /// Indexed by TileId
    tiles: Vec<Tile>,
}

impl World {
    /// Returns true and writes `out` iff `tile_id` contains non-void voxels
    pub fn generate(&self, tile_id: TileId, out: &mut [Material]) -> bool {
        assert_eq!(out.len(), SUBDIVISION_FACTOR.pow(3));
        if tile_id == TileId::ROOT {
            for x in out {
                *x = Material::Stone;
            }
        }
        false
    }
}

impl Default for World {
    fn default() -> Self {
        World {
            tiles: vec![Tile {}],
        }
    }
}

pub const SUBDIVISION_FACTOR: usize = 16;

impl Distribution<World> for Standard {
    fn sample<R: Rng + ?Sized>(&self, _rng: &mut R) -> World {
        World {
            tiles: vec![Tile {}],
        }
    }
}

#[derive(Debug)]
pub struct Tile {
    // TODO
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct TileId(u32);

impl TileId {
    pub const ROOT: Self = Self(0);
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash)]
#[repr(u16)]
pub enum Material {
    Void = 0,
    Stone = 1,
    Dirt = 2,
    Sand = 3,
}

impl Material {
    pub const COUNT: usize = 4;
}

impl Default for Material {
    fn default() -> Self {
        Material::Void
    }
}
