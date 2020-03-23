pub const SUBDIVISION_FACTOR: usize = 12;

#[derive(Debug, Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash)]
#[repr(C)]
pub struct Material(pub u16);

impl Material {

    pub const Void: Material = Material(0);
    pub const Stone: Material = Material(1);
    pub const Dirt: Material = Material(2);
    pub const Sand: Material = Material(3);
    pub const COUNT: usize = 8;
}

impl Default for Material {
    fn default() -> Self {
        Material::Void
    }
}
