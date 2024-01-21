use common::{math, proto::Position};

pub struct LocalCharacterController {
    /// The last extrapolated inter-frame view position, used for rendering and gravity-specific
    /// orientation computations
    position: Position,

    /// The up vector relative to position, ignoring orientation
    up: na::UnitVector3<f32>,

    /// The quaternion adjustment to the character position to represent its actual apparent orientation
    orientation: na::UnitQuaternion<f32>,
}

impl LocalCharacterController {
    pub fn new() -> Self {
        LocalCharacterController {
            position: Position::origin(),
            orientation: na::UnitQuaternion::identity(),
            up: na::Vector::z_axis(),
        }
    }

    pub fn position(&self) -> Position {
        self.position
    }

    pub fn orientation(&self) -> na::UnitQuaternion<f32> {
        self.orientation
    }

    /// Updates the LocalCharacter based on outside information. Note that the `up` parameter is relative
    /// only to `position`, not the character's orientation.
    pub fn update_position(
        &mut self,
        position: Position,
        up: na::UnitVector3<f32>,
        preserve_up_alignment: bool,
    ) {
        if preserve_up_alignment {
            // Rotate the character orientation to stay consistent with changes in gravity
            self.orientation = math::rotation_between_axis(&self.up, &up, 1e-5)
                .unwrap_or(na::UnitQuaternion::identity())
                * self.orientation;
        }

        self.position = position;
        self.up = up;
    }

    /// Rotates the camera's view by locally adding pitch and yaw.
    pub fn look_free(&mut self, delta_yaw: f32, delta_pitch: f32, delta_roll: f32) {
        self.orientation *= na::UnitQuaternion::from_axis_angle(&na::Vector3::y_axis(), delta_yaw)
            * na::UnitQuaternion::from_axis_angle(&na::Vector3::x_axis(), delta_pitch)
            * na::UnitQuaternion::from_axis_angle(&na::Vector3::z_axis(), delta_roll);
    }

    /// Rotates the camera's view with standard first-person walking simulator mouse controls. This function
    /// is designed to be flexible enough to work with any starting orientation, but it works best when the
    /// camera is level, not rolled to the left or right.
    pub fn look_level(&mut self, delta_yaw: f32, delta_pitch: f32) {
        // Get orientation-relative up
        let up = self.orientation.inverse() * self.up;

        // Handle yaw. This is as simple as rotating the view about the up vector
        self.orientation *= na::UnitQuaternion::from_axis_angle(&up, delta_yaw);

        // Handling pitch is more compicated because the view angle needs to be capped. The rotation axis
        // is the camera's local x-axis (left-right axis). If the camera is level, this axis is perpendicular
        // to the up vector.

        // We need to know the current pitch to properly cap pitch changes, and this is only well-defined
        // if the pitch axis is not too similar to the up vector, so we skip applying pitch changes if this
        // isn't the case.
        if up.x.abs() < 0.9 {
            // Compute the current pitch by ignoring the x-component of the up vector and assuming the camera
            // is level.
            let current_pitch = -up.z.atan2(up.y);
            let mut target_pitch = current_pitch + delta_pitch;
            if delta_pitch > 0.0 {
                target_pitch = target_pitch
                    .min(std::f32::consts::FRAC_PI_2) // Cap the view angle at looking straight up
                    .max(current_pitch); // But if already upside-down, don't make any corrections.
            } else {
                target_pitch = target_pitch
                    .max(-std::f32::consts::FRAC_PI_2) // Cap the view angle at looking straight down
                    .min(current_pitch); // But if already upside-down, don't make any corrections.
            }

            self.orientation *= na::UnitQuaternion::from_axis_angle(
                &na::Vector3::x_axis(),
                target_pitch - current_pitch,
            );
        }
    }

    /// Instantly updates the current orientation quaternion to make the camera level. This function
    /// is designed to be numerically stable for any camera orientation.
    pub fn align_to_gravity(&mut self) {
        // Get orientation-relative up
        let up = self.orientation.inverse() * self.up;

        if up.z.abs() < 0.9 {
            // If facing not too vertically, roll the camera to make it level.
            let delta_roll = -up.x.atan2(up.y);
            self.orientation *=
                na::UnitQuaternion::from_axis_angle(&na::Vector3::z_axis(), delta_roll);
        } else if up.y > 0.0 {
            // Otherwise, if not upside-down, yaw the camera to make it level.
            let delta_yaw = (up.x / up.z).atan();
            self.orientation *=
                na::UnitQuaternion::from_axis_angle(&na::Vector3::y_axis(), delta_yaw);
        } else {
            // Otherwise, rotate the camera to look straight up or down.
            self.orientation *=
                na::UnitQuaternion::rotation_between(&(na::Vector3::z() * up.z.signum()), &up)
                    .unwrap();
        }
    }

    /// Returns an orientation quaternion that is as faithful as possible to the current orientation quaternion
    /// while being restricted to ensuring the view is level and does not look up or down. This function's main
    /// purpose is to figure out what direction the character should go when a movement key is pressed.
    pub fn horizontal_orientation(&mut self) -> na::UnitQuaternion<f32> {
        // Get orientation-relative up
        let up = self.orientation.inverse() * self.up;

        let forward = if up.x.abs() < 0.9 {
            // Rotate the local forward vector about the locally horizontal axis until it is horizontal
            na::Vector3::new(0.0, -up.z, up.y)
        } else {
            // Project the local forward vector to the level plane
            na::Vector3::z() - up.into_inner() * up.z
        };

        self.orientation * na::UnitQuaternion::face_towards(&forward, &up)
    }

    pub fn renormalize_orientation(&mut self) {
        self.orientation.renormalize_fast();
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_abs_diff_eq;

    use super::*;

    fn assert_aligned_to_gravity(subject: &LocalCharacterController) {
        let up = subject.orientation.inverse() * subject.up;

        // Make sure up vector doesn't point downwards, as that would mean the character is upside-down
        assert!(up.y >= -1e-5);

        // Make sure the up vector has no sideways component, as that would mean the character view is tilted
        assert_abs_diff_eq!(up.x, 0.0, epsilon = 1.0e-5);
    }

    fn assert_yaw_and_pitch_correct(
        base_orientation: na::UnitQuaternion<f32>,
        yaw: f32,
        pitch: f32,
        actual_orientation: na::UnitQuaternion<f32>,
    ) {
        let expected_orientation = base_orientation
            * na::UnitQuaternion::from_axis_angle(&na::Vector3::y_axis(), yaw)
            * na::UnitQuaternion::from_axis_angle(&na::Vector3::x_axis(), pitch);
        assert_abs_diff_eq!(expected_orientation, actual_orientation, epsilon = 1.0e-5);
    }

    #[test]
    fn look_level_and_horizontal_orientation_examples() {
        let mut subject = LocalCharacterController::new();

        // Pick an arbitrary orientation
        let base_orientation = na::UnitQuaternion::new(na::Vector3::new(1.3, -2.1, 0.5));
        subject.orientation = base_orientation;

        // Choose the up vector that makes the current orientation a horizontal orientation
        subject.up = subject.orientation * na::Vector3::y_axis();

        let mut yaw = 0.0;
        let mut pitch = 0.0;

        // Sanity check that the setup makes sense
        assert_aligned_to_gravity(&subject);
        assert_yaw_and_pitch_correct(base_orientation, yaw, pitch, subject.orientation);
        assert_yaw_and_pitch_correct(base_orientation, yaw, 0.0, subject.horizontal_orientation());

        // Standard look_level expression
        subject.look_level(0.5, -0.4);
        yaw += 0.5;
        pitch -= 0.4;
        assert_aligned_to_gravity(&subject);
        assert_yaw_and_pitch_correct(base_orientation, yaw, pitch, subject.orientation);
        assert_yaw_and_pitch_correct(base_orientation, yaw, 0.0, subject.horizontal_orientation());

        // Look up past the cap
        subject.look_level(-0.2, 3.0);
        yaw -= 0.2;
        pitch = std::f32::consts::FRAC_PI_2;
        assert_aligned_to_gravity(&subject);
        assert_yaw_and_pitch_correct(base_orientation, yaw, pitch, subject.orientation);
        assert_yaw_and_pitch_correct(base_orientation, yaw, 0.0, subject.horizontal_orientation());

        // Look down past the cap
        subject.look_level(6.2, -7.2);
        yaw += 6.2;
        pitch = -std::f32::consts::FRAC_PI_2;
        assert_aligned_to_gravity(&subject);
        assert_yaw_and_pitch_correct(base_orientation, yaw, pitch, subject.orientation);
        assert_yaw_and_pitch_correct(base_orientation, yaw, 0.0, subject.horizontal_orientation());

        // Go back to a less unusual orientation
        subject.look_level(-1.2, 0.8);
        yaw -= 1.2;
        pitch += 0.8;
        assert_aligned_to_gravity(&subject);
        assert_yaw_and_pitch_correct(base_orientation, yaw, pitch, subject.orientation);
        assert_yaw_and_pitch_correct(base_orientation, yaw, 0.0, subject.horizontal_orientation());
    }

    #[test]
    fn align_to_gravity_examples() {
        // Pick an arbitrary orientation
        let base_orientation = na::UnitQuaternion::new(na::Vector3::new(1.3, -2.1, 0.5));

        // Choose the up vector that makes the current orientation close to horizontal orientation
        let mut subject = LocalCharacterController::new();
        subject.orientation = base_orientation;
        subject.up =
            subject.orientation * na::UnitVector3::new_normalize(na::Vector3::new(0.1, 1.0, 0.2));
        let look_direction = subject.orientation * na::Vector3::z_axis();

        subject.align_to_gravity();

        assert_aligned_to_gravity(&subject);
        // The look_direction shouldn't change
        assert_abs_diff_eq!(
            look_direction,
            subject.orientation * na::Vector3::z_axis(),
            epsilon = 1e-5
        );

        // Choose the up vector that makes the current orientation close to horizontal orientation but upside-down
        let mut subject = LocalCharacterController::new();
        subject.orientation = base_orientation;
        subject.up =
            subject.orientation * na::UnitVector3::new_normalize(na::Vector3::new(0.1, -1.0, 0.2));
        let look_direction = subject.orientation * na::Vector3::z_axis();

        subject.align_to_gravity();

        assert_aligned_to_gravity(&subject);
        // The look_direction still shouldn't change
        assert_abs_diff_eq!(
            look_direction,
            subject.orientation * na::Vector3::z_axis(),
            epsilon = 1e-5
        );

        // Make the character face close to straight up
        let mut subject = LocalCharacterController::new();
        subject.orientation = base_orientation;
        subject.up = subject.orientation
            * na::UnitVector3::new_normalize(na::Vector3::new(-0.03, 0.05, 1.0));
        subject.align_to_gravity();
        assert_aligned_to_gravity(&subject);

        // Make the character face close to straight down and be slightly upside-down
        let mut subject = LocalCharacterController::new();
        subject.orientation = base_orientation;
        subject.up = subject.orientation
            * na::UnitVector3::new_normalize(na::Vector3::new(-0.03, -0.05, -1.0));
        subject.align_to_gravity();
        assert_aligned_to_gravity(&subject);
    }

    #[test]
    fn update_position_example() {
        // Pick an arbitrary orientation
        let base_orientation = na::UnitQuaternion::new(na::Vector3::new(1.3, -2.1, 0.5));

        let mut subject = LocalCharacterController::new();
        subject.orientation = base_orientation;
        subject.up =
            subject.orientation * na::UnitVector3::new_normalize(na::Vector3::new(0.0, 1.0, 0.2));

        // Sanity check setup (character should already be aligned to gravity)
        assert_aligned_to_gravity(&subject);
        let old_up_vector_y_component = (subject.orientation.inverse() * subject.up).y;

        subject.update_position(
            Position::origin(),
            na::UnitVector3::new_normalize(na::Vector3::new(0.1, 0.2, 0.5)),
            true,
        );
        assert_aligned_to_gravity(&subject);
        let new_up_vector_y_component = (subject.orientation.inverse() * subject.up).y;

        // We don't want the camera pitch to drift as the up vector changes
        assert_abs_diff_eq!(
            old_up_vector_y_component,
            new_up_vector_y_component,
            epsilon = 1e-5
        );
    }
}
