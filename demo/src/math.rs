use std::ops::{DivAssign, SubAssign};

use na::{ComplexField, U3};

pub fn translation(v: &na::Vector3<f64>) -> na::Matrix3<f64> {
    let f = 1.0 / v.z;
    na::Matrix3::new(
        v.x * v.x * f + 1.0,
        v.x * v.y * f,
        v.x,
        v.x * v.y * f,
        v.y * v.y * f + 1.0,
        v.y,
        v.x,
        v.y,
        v.z,
    )
}

pub fn rotation(theta: f64) -> na::Matrix3<f64> {
    na::Matrix3::new_rotation(theta)
}

pub fn iso_inverse(m: &na::Matrix3<f64>) -> na::Matrix3<f64> {
    na::Matrix3::new(
        m.m11, m.m21, -m.m31, m.m12, m.m22, -m.m32, -m.m13, -m.m23, m.m33,
    )
}

pub fn normal(v0: &na::Vector3<f64>, v1: &na::Vector3<f64>) -> na::Vector3<f64> {
    na::Vector3::new(
        v0.y * v1.z - v0.z * v1.y,
        v0.z * v1.x - v0.x * v1.z,
        -(v0.x * v1.y - v0.y * v1.x),
    )
}

// TODO: Can nalgebra take advantage of deref coersion so that mip can accept a slice or a regular vector?
pub fn mip<S>(v0: na::Vector<f64, U3, S>, v1: na::Vector<f64, U3, S>) -> f64
where
    S: na::Storage<f64, U3>,
{
    v0[0] * v1[0] + v0[1] * v1[1] - v0[2] * v1[2]
}

pub fn sqr<S>(v: na::Vector<f64, U3, S>) -> f64
where
    S: na::Storage<f64, U3>,
{
    v[0] * v[0] + v[1] * v[1] - v[2] * v[2]
}

pub fn displacement(v: &na::Vector3<f64>) -> na::Matrix3<f64> {
    let norm = v.norm();
    let scale_factor = norm.sinhc();
    translation(&na::Vector3::new(
        v.x * scale_factor,
        v.y * scale_factor,
        norm.cosh(),
    ))
}

pub fn qr_normalize(m: &mut na::Matrix3<f64>) {
    div_assign(sqr(m.column(0)).sqrt(), m.column_mut(0));
    sub_assign(m.column(0) * mip(m.column(0), m.column(1)), m.column_mut(1));
    sub_assign(m.column(0) * mip(m.column(0), m.column(2)), m.column_mut(2));

    div_assign(sqr(m.column(1)).sqrt(), m.column_mut(1));
    sub_assign(m.column(1) * mip(m.column(1), m.column(2)), m.column_mut(2));

    div_assign((-sqr(m.column(2))).sqrt(), m.column_mut(2));
}

fn div_assign(divisor: f64, mut slice: na::VectorSliceMut3<f64>) {
    slice.div_assign(divisor);
}

fn sub_assign(subtrahend: na::Vector3<f64>, mut slice: na::VectorSliceMut3<f64>) {
    slice.sub_assign(subtrahend);
}
