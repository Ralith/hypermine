//! This module is used to transform vectors to ensure that they fit constraints discovered during collision checking.

use rand_distr::num_traits::Zero;
use tracing::warn;

use crate::math;

/// Encapsulates all the information needed to constrain a vector (displacement) based on a set of `VectorBound`s and apply those
/// same constraints to a secondary vector (velocity).
#[derive(Clone)]
pub struct BoundedVectors {
    displacement: na::Vector3<f32>,
    velocity: Option<na::Vector3<f32>>,
    bounds: Vec<VectorBound>,
    temp_bounds: Vec<VectorBound>,
    error_margin: f32,
}

impl BoundedVectors {
    /// Initializes a `BoundedVectors` with an empty list of bounds. The `displacement` is the vector
    /// we will apply the bounds to. The size of this vector also determins the error margin
    /// to prevent floating point approximation limits from causing phantom collisions. Note that this
    /// error margin is not needed if the resulting vector is zero, since no phantom collision can occur
    /// if the character is stopped. The `velocity` is a vector that should have similar bounds applied to
    /// it as `displacement`, but it is not used to compute which bounds to apply.
    pub fn new(displacement: na::Vector3<f32>, velocity: Option<na::Vector3<f32>>) -> Self {
        let error_margin = displacement.magnitude() * 1e-4;

        BoundedVectors {
            displacement,
            velocity,
            bounds: vec![],
            temp_bounds: vec![],
            error_margin,
        }
    }

    pub fn displacement(&self) -> &na::Vector3<f32> {
        &self.displacement
    }

    /// Scales the displacement vector without invalidating any of the `VectorBound`s
    pub fn scale_displacement(&mut self, scale_factor: f32) {
        self.displacement *= scale_factor;
        self.error_margin *= scale_factor;
    }

    pub fn velocity(&self) -> Option<&na::Vector3<f32>> {
        self.velocity.as_ref()
    }

    /// Returns the internal list of `VectorBound`s contained in the `BoundedVectors` struct.
    pub fn bounds(&self) -> &[VectorBound] {
        &self.bounds
    }

    /// Constrains `vector` with `new_bound` while keeping the existing constraints satisfied. All projection
    /// transformations applied to `vector` are also applied to `tagalong` to allow two vectors to be transformed consistently
    /// with each other.
    pub fn add_bound(&mut self, new_bound: VectorBound) {
        self.apply_bound(&new_bound);
        self.bounds.push(new_bound);
    }

    /// Temporarily constrains `vector` with `new_bound` while keeping the existing constraints satisfied. All projection
    /// transformations applied to `vector` are also applied to `tagalong` to allow two vectors to be transformed consistently
    /// with each other. Use `clear_temporary_bounds` to get rid of any existing temporary bounds
    pub fn add_temp_bound(&mut self, new_bound: VectorBound) {
        self.apply_bound(&new_bound);
        self.temp_bounds.push(new_bound);
    }

    /// Removes all temporary bounds
    pub fn clear_temp_bounds(&mut self) {
        self.temp_bounds.clear();
    }

    /// Helper function to apply a new bound without adding it to any lists.
    fn apply_bound(&mut self, new_bound: &VectorBound) {
        // There likely isn't a perfect way to get a vector properly constrained with a list of bounds. The main
        // difficulty is finding which set of linearly independent bounds need to be applied so that all bounds are
        // satisfied. Since bounds are one-sided and not guaranteed to be linearly independent from each other, this
        // requires some ad-hoc choices. The algorithm we choose here is to (1) assume that `new_bound` is one of these
        // linearly independent bounds, (2) if necessary, pair it up with each existing bound to find the first such
        // bound that allows all bounds to be satisfied, and (3) zero out the vector if no such pairing works, as we
        // assume that we need to apply three linearly independent bounds.

        // Combine existing bounds with temporary bounds into an iterator
        let bounds_iter = self.bounds.iter().chain(self.temp_bounds.iter());

        // Apply new_bound if necessary.
        if !new_bound.check_vector(&self.displacement, self.error_margin) {
            new_bound.constrain_vector(&mut self.displacement, self.error_margin);
            if let Some(ref mut velocity) = self.velocity {
                // Note: The velocity vector does not need an error margin.
                new_bound.constrain_vector(velocity, 0.0);
            }
        }

        // Check if all constraints are satisfied
        if (bounds_iter.clone()).all(|b| b.check_vector(&self.displacement, self.error_margin)) {
            return;
        }

        // If not all constraints are satisfied, find the first constraint that if applied will satisfy
        // the remaining constriants
        for bound in
            (bounds_iter.clone()).filter(|b| !b.check_vector(&self.displacement, self.error_margin))
        {
            let Some(ortho_bound) = bound.get_self_constrained_with_bound(new_bound) else {
                warn!("Unsatisfied existing bound is parallel to new bound. Is the character squeezed between two walls?");
                continue;
            };

            let mut candidate = self.displacement;
            ortho_bound.constrain_vector(&mut candidate, self.error_margin);

            if (bounds_iter.clone()).all(|b| b.check_vector(&candidate, self.error_margin)) {
                self.displacement = candidate;
                if let Some(ref mut velocity) = self.velocity {
                    ortho_bound.constrain_vector(velocity, 0.0);
                }
                return;
            }
        }

        // If no choice satisfies all constraints, keep all bounds and set the vector to 0
        self.displacement.set_zero();
        if let Some(ref mut velocity) = self.velocity {
            velocity.set_zero();
        }
    }
}

/// Represents a single constraint for a vector. `VectorBound`s alone conceptually contain
/// enough information to apply to a vector, but practically, one other piece of information
/// is needed: `error_margin`, which exists in `BoundedVectors`.
#[derive(Clone)]
pub struct VectorBound {
    normal: na::UnitVector3<f32>,
    projection_direction: na::UnitVector3<f32>,
    front_facing: bool, // Only used for `check_vector` function
}

impl VectorBound {
    /// Creates a `VectorBound` that pushes vectors away from the plane given
    /// by the normal in `projection_direction`. After applying such a bound to
    /// a vector, its dot product with `normal` should be close to zero but positive
    /// even considering floating point error.
    ///
    /// The `VectorBound` will only push vectors that do not currently fulfill the bounds.
    /// If `front_facing` is true, the bound wants the vector to be "in front" of the plane,
    /// in the direction given by `normal`. Otherwise, the bound wants the vector to be "behind"
    /// the plane. Error margins are set so that two planes, one front_facing and one not, with the
    /// same `normal` and `projection_direction`, can both act on a vector without interfering.
    pub fn new(
        normal: na::UnitVector3<f32>,
        projection_direction: na::UnitVector3<f32>,
        front_facing: bool,
    ) -> Self {
        VectorBound {
            normal,
            projection_direction,
            front_facing,
        }
    }

    /// Updates `subject` with a projection transformation based on the constraint given by `self`.
    /// This function does not check whether such a constraint is needed.
    fn constrain_vector(&self, subject: &mut na::Vector3<f32>, error_margin: f32) {
        math::project_to_plane(
            subject,
            &self.normal,
            &self.projection_direction,
            error_margin,
        );
    }

    /// Checks whether `subject` satisfies the constraint given by `self`. Note that `check_vector` will
    /// return `true` after a vector is constrained by `constrain_vector` with the same error margin, even
    /// if it's perturbed slightly. However, that property only holds if the error margin is not too small.
    fn check_vector(&self, subject: &na::Vector3<f32>, error_margin: f32) -> bool {
        if subject.is_zero() {
            return true;
        }

        // An additional margin of error is needed when the bound is checked to ensure that an
        // applied bound always passes the check. Ostensibly, for an applied bound, the dot
        // product is equal to the error margin.
        if self.front_facing {
            // Using 0.5 here should ensure that the check will pass after the bound is applied, and it will fail if the
            // dot product is too close to zero to guarantee that it won't be treated as negative during collision checking
            subject.dot(&self.normal) >= error_margin * 0.5
        } else {
            // Using 1.5 here keeps the additional margin of error equivalent in magnitude to the front-facing case
            subject.dot(&self.normal) <= error_margin * 1.5
        }
    }

    /// Returns a `VectorBound` that is an altered version of `self` so that it no longer interferes
    /// with `bound`. This is achieved by altering the projection direction by a factor of
    /// `bound`'s projection direction to be orthogonal to `bound`'s normal. If this is not
    /// possible, returns `None`.
    fn get_self_constrained_with_bound(&self, bound: &VectorBound) -> Option<VectorBound> {
        let mut ortho_bound_projection_direction = self.projection_direction.into_inner();
        math::project_to_plane(
            &mut ortho_bound_projection_direction,
            &bound.normal,
            &bound.projection_direction,
            0.0,
        );

        na::UnitVector3::try_new(ortho_bound_projection_direction, 1e-5).map(|d| VectorBound {
            normal: self.normal,
            projection_direction: d,
            front_facing: self.front_facing,
        })
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_abs_diff_eq;

    use super::*;

    #[test]
    fn vector_bound_group_example() {
        let mut bounded_vector = BoundedVectors::new(na::Vector3::new(-4.0, -3.0, 1.0), None);

        // Add a bunch of bounds that are achievable with nonzero vectors
        bounded_vector.add_bound(VectorBound::new(
            unit_vector(1.0, 3.0, 4.0),
            unit_vector(1.0, 2.0, 2.0),
            true,
        ));

        assert_ne!(bounded_vector.displacement, na::Vector3::zero());
        assert_bounds_achieved(&bounded_vector);

        bounded_vector.add_bound(VectorBound::new(
            unit_vector(2.0, -3.0, -4.0),
            unit_vector(1.0, -2.0, -1.0),
            true,
        ));

        assert_ne!(bounded_vector.displacement, na::Vector3::zero());
        assert_bounds_achieved(&bounded_vector);

        bounded_vector.add_bound(VectorBound::new(
            unit_vector(2.0, -3.0, -5.0),
            unit_vector(1.0, -2.0, -2.0),
            true,
        ));

        assert_ne!(bounded_vector.displacement, na::Vector3::zero());
        assert_bounds_achieved(&bounded_vector);

        // Finally, add a bound that overconstrains the system
        bounded_vector.add_bound(VectorBound::new(
            unit_vector(-3.0, 3.0, -2.0),
            unit_vector(-3.0, 3.0, -2.0),
            true,
        ));

        // Using assert_eq instead of assert_ne here
        assert_eq!(bounded_vector.displacement, na::Vector3::zero());
        // Special logic allows bounds checking to work with the zero vector
        assert_bounds_achieved(&bounded_vector);
    }

    #[test]
    fn constrain_vector_example() {
        let normal = unit_vector(1.0, 3.0, 4.0);
        let projection_direction = unit_vector(1.0, 2.0, 2.0);
        let error_margin = 1e-4;
        let bound = VectorBound::new(normal, projection_direction, true);

        let initial_vector = na::Vector3::new(-4.0, -3.0, 1.0);

        assert!(!bound.check_vector(&initial_vector, error_margin));

        let mut constrined_vector = initial_vector;
        bound.constrain_vector(&mut constrined_vector, error_margin);

        assert!(bound.check_vector(&constrined_vector, error_margin));
        assert_collinear(
            constrined_vector - initial_vector,
            projection_direction.into_inner(),
            1e-5,
        );
    }

    #[test]
    fn get_self_constrained_with_bound_example() {
        // For simplicity, we test with an error margin of 0.
        let normal0 = unit_vector(1.0, 3.0, 4.0);
        let projection_direction0 = unit_vector(1.0, 2.0, 2.0);

        let normal1 = unit_vector(1.0, -4.0, 3.0);
        let projection_direction1 = unit_vector(1.0, -2.0, 1.0);

        let bound0 = VectorBound::new(normal0, projection_direction0, true);
        let bound1 = VectorBound::new(normal1, projection_direction1, true);

        let initial_vector = na::Vector3::new(2.0, -1.0, -3.0);
        let mut constrained_vector = initial_vector;
        bound0.constrain_vector(&mut constrained_vector, 0.0);

        let ortho_bound1 = bound1.get_self_constrained_with_bound(&bound0).unwrap();
        ortho_bound1.constrain_vector(&mut constrained_vector, 0.0);

        // Check that the constrained vector is on the intersection between the two bound planes
        assert_abs_diff_eq!(constrained_vector.dot(&normal0), 0.0, epsilon = 1e-5);
        assert_abs_diff_eq!(constrained_vector.dot(&normal1), 0.0, epsilon = 1e-5);

        // Check that the delta of the constrained vector is a linear combination of the projection directions.
        // To do this, we check whether the vector is orthogonal to the normal of the plane produced by the two
        // projection directions.
        assert_abs_diff_eq!(
            (constrained_vector - initial_vector)
                .dot(&projection_direction0.cross(&projection_direction1)),
            0.0,
            epsilon = 1e-5
        );
    }

    fn assert_bounds_achieved(bounds: &BoundedVectors) {
        for bound in bounds.bounds() {
            assert!(bound.check_vector(&bounds.displacement, bounds.error_margin));
        }
    }

    fn assert_collinear(v0: na::Vector3<f32>, v1: na::Vector3<f32>, epsilon: f32) {
        assert_abs_diff_eq!(
            v0.normalize(),
            v1.normalize() * (v0.dot(&v1)).signum(),
            epsilon = epsilon
        );
    }

    /// Unit vector
    fn unit_vector(x: f32, y: f32, z: f32) -> na::UnitVector3<f32> {
        na::UnitVector3::new_normalize(na::Vector3::new(x, y, z))
    }
}
