use crate::dynamics::{RigidBodyHandle, RigidBodySet};
use crate::math::{Real, SPATIAL_DIM};

use super::AvbdBodySet;

#[cfg(feature = "serde-serialize")]
use serde::{Deserialize, Serialize};

/// Cached dual variables and stiffness for a single AVBD constraint.
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct AvbdConstraintState {
    /// The dual variable \(\lambda\) associated with the constraint.
    pub lambda: Real,
    /// The adaptive stiffness parameter used by the augmented Lagrangian update.
    pub stiffness: Real,
}

impl AvbdConstraintState {
    /// Creates a new constraint state initialized with the provided values.
    pub fn new(lambda: Real, stiffness: Real) -> Self {
        Self { lambda, stiffness }
    }

    /// Resets the state to the supplied values.
    pub fn reset(&mut self, lambda: Real, stiffness: Real) {
        self.lambda = lambda;
        self.stiffness = stiffness;
    }
}

/// Number of generalized coordinates handled by the AVBD solver backend.
pub const AVBD_DOF: usize = SPATIAL_DIM;

/// Core interface that any AVBD constraint implementation must satisfy.
pub trait AvbdConstraint {
    /// Returns the handles of all bodies affected by this constraint.
    fn bodies(&self) -> &[RigidBodyHandle];

    /// Returns the mutable state (dual variables, stiffness) for the constraint.
    fn state_mut(&mut self) -> &mut AvbdConstraintState;

    /// Returns the immutable state for the constraint.
    fn state(&self) -> &AvbdConstraintState;

    /// Evaluates the constraint function \(C(x)\) for the current body configuration.
    fn evaluate(&self, bodies: &RigidBodySet, workspace: &AvbdBodySet) -> Real;

    /// Computes the gradient of the constraint with respect to the specified body.
    fn gradient(
        &self,
        bodies: &RigidBodySet,
        workspace: &AvbdBodySet,
        body: RigidBodyHandle,
        out: &mut [Real; AVBD_DOF],
    );

    /// Computes (or approximates) the Hessian contribution for the specified body.
    fn hessian(
        &self,
        bodies: &RigidBodySet,
        workspace: &AvbdBodySet,
        body: RigidBodyHandle,
        out: &mut [[Real; AVBD_DOF]; AVBD_DOF],
    );

    /// Projects the dual variable into the feasible set for this constraint.
    ///
    /// Defaults to the identity (no projection). Contact constraints overload this
    /// to enforce the non-negativity of the normal impulse.
    fn project_lambda(&self, lambda: Real) -> Real {
        lambda
    }

    /// Returns the relative normal velocity associated with this constraint, if applicable.
    fn relative_normal_velocity(
        &self,
        _bodies: &RigidBodySet,
        _workspace: &AvbdBodySet,
    ) -> Option<Real> {
        None
    }
}
