//! A few simple utilities to make it easier to color entities in the demo appropriately

use std::ops::{Add, AddAssign, Mul, MulAssign, Neg, Sub, SubAssign};

/// A color represented in RGB, where the hue/saturation of red, green, and blue are as usual,
/// but the brightness is normalized so that the ideal grayscale value is just R + G + B.
/// This means that to be a displayable color, red has to be between 0 and 0.299, green has to
/// be between 0 and 0.587, and blue has to be between 0 and 0.114. The color space is also linear.
/// TODO: Edit documentation, as red, green, and blue are now defined as normal (but still linear)
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
            red: rgb.0,
            green: rgb.1,
            blue: rgb.2,
        }
    }

    pub fn inverted(&self) -> Color {
        Color {
            red: 1.0 - self.red,
            green: 1.0 - self.green,
            blue: 1.0 - self.blue,
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
