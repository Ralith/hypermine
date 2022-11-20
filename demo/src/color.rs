//! A few simple utilities to make it easier to color entities in the demo appropriately

use std::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Neg, Sub, SubAssign};

/// A color represented in linear RGB. A method exist to get its luminance, which weights red,
/// green, and blue appropriately.
#[derive(Clone, Copy)]
pub struct Color {
    red: f64,
    green: f64,
    blue: f64,
}

impl Color {
    const RED_FACTOR: f64 = 0.299;
    const GREEN_FACTOR: f64 = 0.587;
    const BLUE_FACTOR: f64 = 0.114;

    pub const WHITE: Color = Color {
        red: 1.0,
        green: 1.0,
        blue: 1.0,
    };

    pub fn from_hue(hue: f64) -> Color {
        let hue_normalized = (hue - hue.floor()) * 6.0;
        let hue_type = hue_normalized.floor().min(5.0);
        let hue_frac = hue_normalized - hue_type;
        let rgb = match hue_type as u32 {
            0 => (1.0, hue_frac, 0.0),
            1 => (1.0 - hue_frac, 1.0, 0.0),
            2 => (0.0, 1.0, hue_frac),
            3 => (0.0, 1.0 - hue_frac, 1.0),
            4 => (hue_frac, 0.0, 1.0),
            5 => (1.0, 0.0, 1.0 - hue_frac),
            _ => panic!("{} isn't in [0,6)", hue_type),
        };
        Color {
            red: Self::srgb_to_linear(rgb.0),
            green: Self::srgb_to_linear(rgb.1),
            blue: Self::srgb_to_linear(rgb.2),
        }
    }

    pub fn inverted(&self) -> Color {
        Color {
            red: 1.0 - self.red,
            green: 1.0 - self.green,
            blue: 1.0 - self.blue,
        }
    }

    pub fn luminance(&self) -> f64 {
        self.red * Self::RED_FACTOR
            + self.green * Self::GREEN_FACTOR
            + self.blue * Self::BLUE_FACTOR
    }

    fn srgb_to_linear(srgb: f64) -> f64 {
        if srgb <= 0.04045 {
            srgb / 12.92
        } else {
            ((srgb + 0.055) / 1.055).powf(2.4)
        }
    }

    fn linear_to_srgb(linear: f64) -> f64 {
        if linear <= 0.0031308 {
            linear * 12.92
        } else {
            linear.powf(1.0 / 2.4) * 1.055 - 0.055
        }
    }
}

impl Add for Color {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Color {
            red: self.red + rhs.red,
            green: self.green + rhs.green,
            blue: self.blue + rhs.blue,
        }
    }
}

impl AddAssign for Color {
    fn add_assign(&mut self, rhs: Self) {
        self.red += rhs.red;
        self.green += rhs.green;
        self.blue += rhs.blue;
    }
}

impl Mul<f64> for Color {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self::Output {
        Color {
            red: self.red * rhs,
            green: self.green * rhs,
            blue: self.blue * rhs,
        }
    }
}

impl MulAssign<f64> for Color {
    fn mul_assign(&mut self, rhs: f64) {
        self.red *= rhs;
        self.green *= rhs;
        self.blue *= rhs;
    }
}

impl Div<f64> for Color {
    type Output = Self;

    fn div(self, rhs: f64) -> Self::Output {
        Color {
            red: self.red / rhs,
            green: self.green / rhs,
            blue: self.blue / rhs,
        }
    }
}

impl DivAssign<f64> for Color {
    fn div_assign(&mut self, rhs: f64) {
        self.red /= rhs;
        self.green /= rhs;
        self.blue /= rhs;
    }
}

impl Sub for Color {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Color {
            red: self.red - rhs.red,
            green: self.green - rhs.green,
            blue: self.blue - rhs.blue,
        }
    }
}

impl SubAssign for Color {
    fn sub_assign(&mut self, rhs: Self) {
        self.red -= rhs.red;
        self.green -= rhs.green;
        self.blue -= rhs.blue;
    }
}

impl Neg for Color {
    type Output = Self;

    fn neg(self) -> Self::Output {
        Color {
            red: -self.red,
            green: -self.green,
            blue: -self.blue,
        }
    }
}

impl From<Color> for [f32; 4] {
    fn from(color: Color) -> Self {
        [
            Color::linear_to_srgb(color.red).clamp(0.0, 1.0) as f32,
            Color::linear_to_srgb(color.green).clamp(0.0, 1.0) as f32,
            Color::linear_to_srgb(color.blue).clamp(0.0, 1.0) as f32,
            1.0,
        ]
    }
}
