//! From "Visualizing Hyperbolic Space: Unusual Uses of 4x4 Matrices." Phillips, Gunn.
//!
//! Arguments are 4D vectors that pass through the positive sheet of the 3-hyperboloid and whose
//! length is not meaningful.

use na::RealField;

pub fn reflect<N: RealField>(p: &na::Vector4<N>) -> na::Matrix4<N> {
    na::Matrix4::<N>::identity()
        - (*p * p.transpose() * i31::<N>()) * na::convert::<_, N>(2.0) / mip(&p, &p)
}

pub fn translate<N: RealField>(a: &na::Vector4<N>, b: &na::Vector4<N>) -> na::Matrix4<N> {
    let midpoint = a * (mip(b, b) * mip(a, b)).sqrt() + b * (mip(a, a) * mip(a, b)).sqrt();
    reflect(&midpoint) * reflect(a)
}

pub fn distance<N: RealField>(a: &na::Vector4<N>, b: &na::Vector4<N>) -> N {
    na::convert::<_, N>(2.0) * (mip(a, b).powi(2) / (mip(a, a) * mip(b, b))).sqrt().acosh()
}

/// Minkowski inner product, aka <a, b>_h
fn mip<N: RealField>(a: &na::Vector4<N>, b: &na::Vector4<N>) -> N {
    a.x * b.x + a.y * b.y + a.z * b.z - a.w * b.w
}

#[rustfmt::skip]
fn i31<N: RealField>() -> na::Matrix4<N> {
    na::convert::<_, na::Matrix4<N>>(na::Matrix4::<f64>::new(
        1.0, 0.0, 0.0,  0.0,
        0.0, 1.0, 0.0,  0.0,
        0.0, 0.0, 1.0,  0.0,
        0.0, 0.0, 0.0, -1.0
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::*;

    #[test]
    #[rustfmt::skip]
    fn reflect_example() {
        assert_abs_diff_eq!(
            reflect(&na::Vector4::new(0.5, 0.0, 0.0, 1.0)),
            na::Matrix4::new(
                1.666, 0.0, 0.0, -1.333,
                0.0  , 1.0, 0.0,  0.0,
                0.0  , 0.0, 1.0,  0.0,
                1.333, 0.0, 0.0, -1.666
            ),
            epsilon = 1e-3
        );
    }

    #[test]
    #[rustfmt::skip]
    fn translate_example() {
        assert_abs_diff_eq!(
            translate(&na::Vector4::new(-0.5, -0.5, 0.0, 1.0), &na::Vector4::new(0.3, -0.7, 0.0, 1.0)),
            na::Matrix4::new(
                 1.676, 0.814, 0.0,  1.572,
                -1.369, 0.636, 0.0, -1.130,
                 0.0,   0.0,   1.0,  0.0,
                 1.919, 0.257, 0.0,  2.179,
            ),
            epsilon = 1e-3
        );
    }

    #[test]
    fn translate_identity() {
        let a = na::Vector4::new(-0.5, -0.5, 0.0, 1.0);
        let b = na::Vector4::new(0.3, -0.7, 0.0, 1.0);
        let o = na::Vector4::new(0.0, 0.0, 0.0, 1.0);
        assert_abs_diff_eq!(
            translate(&a, &b),
            translate(&o, &a) * translate(&o, &(translate(&a, &o) * b)) * translate(&a, &o),
            epsilon = 1e-5
        );
    }

    #[test]
    fn distance_example() {
        let a = na::Vector4::new(0.2, 0.0, 0.0, 1.0);
        let b = na::Vector4::new(-0.5, -0.5, 0.0, 1.0);
        assert_abs_diff_eq!(distance(&a, &b), 2.074, epsilon = 1e-3);
    }
}
