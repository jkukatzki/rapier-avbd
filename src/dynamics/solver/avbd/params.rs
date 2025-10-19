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
    /// Enables applying the cached dual variables as a positional warm start each step.
    pub warmstart: bool,
    /// Allows the solver to parallelize independent constraint buckets when the `parallel` feature is enabled.
    pub allow_parallelism: bool,
    /// Minimum stiffness used when re-initializing constraints.
    pub stiffness_min: Real,
    /// Maximum stiffness permitted for a constraint during the augmented updates.
    pub stiffness_max: Real,
}

impl Default for AvbdSolverParams {
    fn default() -> Self {
        Self {
            iterations: 4,
            alpha: 0.95,
            beta: 10.0,
            gamma: 0.99,
            warmstart: true,
            allow_parallelism: false,
            stiffness_min: 1.0,
            stiffness_max: 1.0e6,
        }
    }
}
