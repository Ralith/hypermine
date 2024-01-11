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
}

impl TryFrom<u16> for Material {
    type Error = ();

    fn try_from(value: u16) -> Result<Self, Self::Error> {
        Ok(match value {
            0 => Material::Void,
            1 => Material::Dirt,
            2 => Material::Sand,
            3 => Material::Silt,
            4 => Material::Clay,
            5 => Material::Mud,
            6 => Material::SandyLoam,
            7 => Material::SiltyLoam,
            8 => Material::ClayLoam,
            9 => Material::RedSand,
            10 => Material::Limestone,
            11 => Material::Shale,
            12 => Material::Dolomite,
            13 => Material::Sandstone,
            14 => Material::RedSandstone,
            15 => Material::Marble,
            16 => Material::Slate,
            17 => Material::Granite,
            18 => Material::Diorite,
            19 => Material::Andesite,
            20 => Material::Gabbro,
            21 => Material::Basalt,
            22 => Material::Olivine,
            23 => Material::Water,
            24 => Material::Lava,
            25 => Material::Wood,
            26 => Material::Leaves,
            27 => Material::WoodPlanks,
            28 => Material::GreyBrick,
            29 => Material::WhiteBrick,
            30 => Material::Ice,
            31 => Material::IceSlush,
            32 => Material::Gravel,
            33 => Material::Snow,
            34 => Material::CoarseGrass,
            35 => Material::TanGrass,
            36 => Material::LushGrass,
            37 => Material::MudGrass,
            38 => Material::Grass,
            39 => Material::CaveGrass,
            _ => Err(())?,
        })
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
