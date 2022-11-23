use na::{ComplexField, RowVector3, U3};
use std::ops::{DivAssign, Index, SubAssign};

#[allow(non_camel_case_types)]
pub type f32or64 = f64;
type GeneralVector3<S> = na::Vector<f32or64, U3, S>;

pub trait HyperboloidVector:
    Index<usize, Output = f32or64> + std::ops::Mul<f32or64, Output = na::Vector3<f32or64>>
{
    fn to_point(&self) -> [f32or64; 2];
    fn translation(&self) -> na::Matrix3<f32or64>;
    fn reflection(&self) -> na::Matrix3<f32or64>;
    fn normal(&self, v1: &GeneralVector3<impl na::Storage<f32or64, U3>>) -> na::Vector3<f32or64>;
    fn mip(&self, v1: &GeneralVector3<impl na::Storage<f32or64, U3>>) -> f32or64;
    fn mop_sqr(&self, v1: &GeneralVector3<impl na::Storage<f32or64, U3>>) -> f32or64;
    fn sqr(&self) -> f32or64;
    fn m_normalized_point(&self) -> na::Vector3<f32or64>;
    fn m_normalized_vector(&self) -> na::Vector3<f32or64>;
    fn displacement(&self) -> na::Matrix3<f32or64>;
    fn displacement_vec(&self) -> na::Vector3<f32or64>;
    fn euclidean_point(&self) -> na::Vector2<f32or64>;
    fn project_xy(&self) -> na::Vector3<f32or64>;
    fn project_ortho(
        &self,
        v: &GeneralVector3<impl na::Storage<f32or64, U3>>,
    ) -> na::Vector3<f32or64>;
}

pub trait HyperboloidMatrix {
    fn iso_inverse(&self) -> na::Matrix3<f32or64>;
    fn qr_normalize(&mut self);
}

impl<S: na::Storage<f32or64, U3>> HyperboloidVector for na::Vector<f32or64, U3, S> {
    fn to_point(&self) -> [f32or64; 2] {
        [
            (self[0] / self[2]) as f32or64,
            (self[1] / self[2]) as f32or64,
        ]
    }

    fn translation(&self) -> na::Matrix3<f32or64> {
        let f = 1.0 / (self[2] + 1.0);
        na::Matrix3::new(
            self[0] * self[0] * f + 1.0,
            self[0] * self[1] * f,
            self[0],
            self[0] * self[1] * f,
            self[1] * self[1] * f + 1.0,
            self[1],
            self[0],
            self[1],
            self[2],
        )
    }

    fn reflection(&self) -> na::Matrix3<f32or64> {
        na::Matrix3::identity() - (self * RowVector3::new(self[0], self[1], -self[2])) * 2.0
    }

    fn normal(&self, v1: &GeneralVector3<impl na::Storage<f32or64, U3>>) -> na::Vector3<f32or64> {
        na::Vector3::new(
            self[1] * v1[2] - self[2] * v1[1],
            self[2] * v1[0] - self[0] * v1[2],
            -(self[0] * v1[1] - self[1] * v1[0]),
        )
    }

    fn mip(&self, v1: &GeneralVector3<impl na::Storage<f32or64, U3>>) -> f32or64 {
        self[0] * v1[0] + self[1] * v1[1] - self[2] * v1[2]
    }

    fn mop_sqr(&self, v1: &GeneralVector3<impl na::Storage<f32or64, U3>>) -> f32or64 {
        self.normal(v1).sqr() // TODO: Be more direct
    }

    fn sqr(&self) -> f32or64 {
        self[0] * self[0] + self[1] * self[1] - self[2] * self[2]
    }

    fn m_normalized_point(&self) -> na::Vector3<f32or64> {
        self / (-self.sqr()).sqrt()
    }

    fn m_normalized_vector(&self) -> na::Vector3<f32or64> {
        self / self.sqr().sqrt()
    }

    fn displacement(&self) -> na::Matrix3<f32or64> {
        self.displacement_vec().translation()
    }

    fn displacement_vec(&self) -> na::Vector3<f32or64> {
        let norm = self.norm();
        let scale_factor = norm.sinhc();
        na::Vector3::new(self[0] * scale_factor, self[1] * scale_factor, norm.cosh())
    }

    fn euclidean_point(&self) -> na::Vector2<f32or64> {
        na::Vector2::new(self[0] / self[2], self[1] / self[2])
    }

    fn project_xy(&self) -> na::Vector3<f32or64> {
        na::Vector3::new(self[0], self[1], 0.0)
    }

    fn project_ortho(
        &self,
        v: &GeneralVector3<impl na::Storage<f32or64, U3>>,
    ) -> na::Vector3<f32or64> {
        self - v * v.mip(self)
    }
}

impl HyperboloidMatrix for na::Matrix3<f32or64> {
    fn iso_inverse(&self) -> na::Matrix3<f32or64> {
        na::Matrix3::new(
            self[(0, 0)],
            self[(1, 0)],
            -self[(2, 0)],
            self[(0, 1)],
            self[(1, 1)],
            -self[(2, 1)],
            -self[(0, 2)],
            -self[(1, 2)],
            self[(2, 2)],
        )
    }

    fn qr_normalize(&mut self) {
        fn div_assign(divisor: f32or64, mut slice: na::VectorSliceMut3<f32or64>) {
            // Reorder arguments to allow for nested expressions
            slice.div_assign(divisor);
        }

        fn sub_assign(subtrahend: na::Vector3<f32or64>, mut slice: na::VectorSliceMut3<f32or64>) {
            // Reorder arguments to allow for nested expressions
            slice.sub_assign(subtrahend);
        }

        div_assign(self.column(0).sqr().sqrt(), self.column_mut(0));
        sub_assign(
            self.column(0) * self.column(0).mip(&self.column(1)),
            self.column_mut(1),
        );
        sub_assign(
            self.column(0) * self.column(0).mip(&self.column(2)),
            self.column_mut(2),
        );

        div_assign(self.column(1).sqr().sqrt(), self.column_mut(1));
        sub_assign(
            self.column(1) * self.column(1).mip(&self.column(2)),
            self.column_mut(2),
        );

        div_assign((-self.column(2).sqr()).sqrt(), self.column_mut(2));
    }
}
