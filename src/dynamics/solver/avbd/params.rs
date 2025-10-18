use crate::math::Real;

#[cfg(feature = "serde-serialize")]
use serde::{Deserialize, Serialize};

/// Configuration parameters specific to the AVBD solver backend.
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct AvbdSolverParams {
    /// Maximum number of outer AVBD iterations per solver step.
    pub iterations: usize,
    /// Coefficient used to warm-start the dual variable between frames.
    pub alpha: Real,
    /// Multiplicative growth applied to the constraint stiffness after each iteration.
    pub beta: Real,
    /// Decay applied to the stored stiffness and dual variable when warm-starting a new frame.
    pub gamma: Real,
    /// Minimum stiffness used when re-initializing constraints.
    pub stiffness_min: Real,
    /// Maximum stiffness permitted for a constraint during the augmented updates.
    pub stiffness_max: Real,
    /// Small diagonal term added to each per-body system to keep Hessians positive definite.
    pub regularization: Real,
}

impl Default for AvbdSolverParams {
    fn default() -> Self {
        Self {
            iterations: 4,
            alpha: 0.95,
            beta: 10.0,
            gamma: 0.99,
            stiffness_min: 1.0,
            stiffness_max: 1.0e6,
            regularization: 1.0e-6,
        }
    }
}
