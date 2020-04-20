#[derive(Debug, Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash)]
#[repr(u16)]
pub enum Material {
    Void = 0,
    Stone = 1,
    Dirt = 2,
    Sand = 3,
    Wood = 4,
    Leaves = 5,
    Water = 6,
    Snow = 7,
    Grass = 8,
    Redsand = 9,
    Redstone = 10,
    Valite = 11,
    Greystone = 12,
    Flowergrass = 13,
    Gravelstone = 14,
    Graveldirt = 15,
    Blackstone = 16,
    Bigflowergrass = 17,
}

impl Material {
    pub const COUNT: usize = 18;
}

impl Default for Material {
    fn default() -> Self {
        Material::Void
    }
}
