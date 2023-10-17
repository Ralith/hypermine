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
