use serde::{Deserialize, Serialize};

#[derive(
    Debug, Copy, Clone, Default, Eq, PartialEq, Ord, PartialOrd, Hash, Serialize, Deserialize,
)]
#[repr(u16)]
pub enum Material {
    #[default]
    Void = 0,
    Dirt = 1,
    Sand = 2,
    Silt = 3,
    Clay = 4,
    Mud = 5,
    SandyLoam = 6,
    SiltyLoam = 7,
    ClayLoam = 8,
    RedSand = 9,
    Limestone = 10,
    Shale = 11,
    Dolomite = 12,
    Sandstone = 13,
    RedSandstone = 14,
    Marble = 15,
    Slate = 16,
    Granite = 17,
    Diorite = 18,
    Andesite = 19,
    Gabbro = 20,
    Basalt = 21,
    Olivine = 22,
    Water = 23,
    Lava = 24,
    Wood = 25,
    Leaves = 26,
    WoodPlanks = 27,
    GreyBrick = 28,
    WhiteBrick = 29,
    Ice = 30,
    IceSlush = 31,
    Gravel = 32,
    Snow = 33,
    CoarseGrass = 34,
    TanGrass = 35,
    LushGrass = 36,
    MudGrass = 37,
    Grass = 38,
    CaveGrass = 39,
}

impl Material {
    pub const COUNT: usize = 40;

    pub const VALUES: [Self; Self::COUNT] = [
        Material::Void,
        Material::Dirt,
        Material::Sand,
        Material::Silt,
        Material::Clay,
        Material::Mud,
        Material::SandyLoam,
        Material::SiltyLoam,
        Material::ClayLoam,
        Material::RedSand,
        Material::Limestone,
        Material::Shale,
        Material::Dolomite,
        Material::Sandstone,
        Material::RedSandstone,
        Material::Marble,
        Material::Slate,
        Material::Granite,
        Material::Diorite,
        Material::Andesite,
        Material::Gabbro,
        Material::Basalt,
        Material::Olivine,
        Material::Water,
        Material::Lava,
        Material::Wood,
        Material::Leaves,
        Material::WoodPlanks,
        Material::GreyBrick,
        Material::WhiteBrick,
        Material::Ice,
        Material::IceSlush,
        Material::Gravel,
        Material::Snow,
        Material::CoarseGrass,
        Material::TanGrass,
        Material::LushGrass,
        Material::MudGrass,
        Material::Grass,
        Material::CaveGrass,
    ];
}

impl TryFrom<u16> for Material {
    type Error = ();

    fn try_from(value: u16) -> Result<Self, Self::Error> {
        Material::VALUES.get(value as usize).ok_or(()).copied()
    }
}

#[cfg(test)]
mod tests {
    use super::Material;

    #[test]
    fn u16_to_material_consistency_check() {
        for i in 0..Material::COUNT {
            let index = u16::try_from(i).unwrap();
            let material =
                Material::try_from(index).expect("no missing entries in try_from match statement");
            assert_eq!(index, material as u16);
        }
    }
}
