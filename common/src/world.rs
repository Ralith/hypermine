pub const SUBDIVISION_FACTOR: usize = 16;

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
