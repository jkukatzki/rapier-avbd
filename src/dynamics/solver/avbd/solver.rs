use std::sync::Once;

use crate::dynamics::RigidBodySet;
use crate::math::Real;

use super::{AvbdConstraint, AvbdSolverParams};

/// Placeholder AVBD solver entry point.
///
/// The full Gauss–Seidel style implementation described in the AVBD paper will
/// reside here. For now this type stores the solver parameters and exposes an
/// interface compatible with the rest of the Rapier pipeline so subsequent
/// patches can focus on the numerical algorithm itself.
pub struct AvbdSolver {
    params: AvbdSolverParams,
}

impl AvbdSolver {
    /// Creates a new solver with the provided parameters.
    pub fn new(params: AvbdSolverParams) -> Self {
        Self { params }
    }

    /// Returns a shared reference to the current solver parameters.
    pub fn params(&self) -> &AvbdSolverParams {
        &self.params
    }

    /// Returns a mutable reference to the current solver parameters.
    pub fn params_mut(&mut self) -> &mut AvbdSolverParams {
        &mut self.params
    }

    /// Executes an AVBD solve pass for the supplied rigid bodies and constraints.
    ///
    /// The heavy lifting will be added in subsequent patches. At the moment this
    /// simply logs a warning to signal that the backend is under development.
    #[allow(unused_variables)]
    pub fn solve<C>(&mut self, bodies: &mut RigidBodySet, constraints: &mut [C], dt: Real)
    where
        C: AvbdConstraint,
    {
        static WARN_ONCE: Once = Once::new();
        WARN_ONCE.call_once(|| {
            log::warn!(
                "The `solver_avbd` feature is enabled, but the AVBD solver implementation is not yet complete."
            );
        });
    }
}
