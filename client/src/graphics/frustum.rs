use common::math::{MDirection, MPoint};

#[derive(Debug, Copy, Clone)]
pub struct Frustum {
    pub left: f32,
    pub right: f32,
    pub down: f32,
    pub up: f32,
}

impl Frustum {
    /// Construct a symmetric frustum from a vertical FoV and an aspect ratio (width / height)
    pub fn from_vfov(vfov: f32, aspect_ratio: f32) -> Self {
        let hfov = (aspect_ratio * vfov.tan()).atan();
        Self {
            left: -hfov,
            right: hfov,
            down: -vfov,
            up: vfov,
        }
    }

    #[rustfmt::skip]
    /// Compute right-handed y-up inverse Z perspective projection matrix with far plane at 1.0
    ///
    /// This projection is applied to Beltrami-Klein vertices, which fall within a ball of radius 1
    /// around the viewpoint, so a far plane of 1.0 gives us ideal distribution of depth precision.
    pub fn projection(&self, znear: f32) -> na::Projective3<f32> {
        // Based on http://dev.theomader.com/depth-precision/ (broken link) + OpenVR docs
        // Additional context at https://developer.nvidia.com/content/depth-precision-visualized
        let zfar = 1.0;
        let left = self.left.tan();
        let right = self.right.tan();
        let down = self.down.tan();
        let up = self.up.tan();
        let idx = 1.0 / (right - left);
        let idy = 1.0 / (down - up);
        let sx = right + left;
        let sy = down + up;
        // For an infinite far plane instead, za = 0 and zb = znear
        let za = -znear / (znear - zfar);
        let zb = -(znear * zfar) / (znear - zfar);
        na::Projective3::from_matrix_unchecked(
            na::Matrix4::new(
                2.0 * idx,       0.0,  sx * idx,       0.0,
                      0.0, 2.0 * idy,  sy * idy,       0.0,
                      0.0,       0.0,        za,        zb,
                      0.0,       0.0,      -1.0,       0.0))
    }

    pub fn planes(&self) -> FrustumPlanes {
        FrustumPlanes {
            left: MDirection::from(
                na::UnitQuaternion::from_axis_angle(&na::Vector3::y_axis(), self.left)
                    * -na::Vector3::x_axis(),
            ),
            right: MDirection::from(
                na::UnitQuaternion::from_axis_angle(&na::Vector3::y_axis(), self.right)
                    * na::Vector3::x_axis(),
            ),
            down: MDirection::from(
                na::UnitQuaternion::from_axis_angle(&na::Vector3::x_axis(), self.down)
                    * na::Vector3::y_axis(),
            ),
            up: MDirection::from(
                na::UnitQuaternion::from_axis_angle(&na::Vector3::x_axis(), self.up)
                    * -na::Vector3::y_axis(),
            ),
        }
    }
}

#[derive(Debug, Copy, Clone)]
pub struct FrustumPlanes {
    left: MDirection<f32>,
    right: MDirection<f32>,
    down: MDirection<f32>,
    up: MDirection<f32>,
}

impl FrustumPlanes {
    pub fn contain(&self, point: &MPoint<f32>, radius: f32) -> bool {
        for &plane in &[&self.left, &self.right, &self.down, &self.up] {
            if plane.mip(point).asinh() < -radius {
                return false;
            }
        }
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use common::math::MIsometry;
    use std::f32;

    #[test]
    fn planes_sanity() {
        // 90 degree square
        let planes = Frustum::from_vfov(f32::consts::FRAC_PI_4, 1.0).planes();
        assert!(planes.contain(&MPoint::origin(), 0.1));
        assert!(planes.contain(
            &(MIsometry::translation_along(&-na::Vector3::z()) * MPoint::origin()),
            0.0
        ));
        assert!(!planes.contain(
            &(MIsometry::translation_along(&na::Vector3::z()) * MPoint::origin()),
            0.0
        ));
        assert!(!planes.contain(
            &(MIsometry::translation_along(&na::Vector3::x()) * MPoint::origin()),
            0.0
        ));
        assert!(!planes.contain(
            &(MIsometry::translation_along(&na::Vector3::y()) * MPoint::origin()),
            0.0
        ));
        assert!(!planes.contain(
            &(MIsometry::translation_along(&-na::Vector3::x()) * MPoint::origin()),
            0.0
        ));
        assert!(!planes.contain(
            &(MIsometry::translation_along(&-na::Vector3::y()) * MPoint::origin()),
            0.0
        ));
    }
}
