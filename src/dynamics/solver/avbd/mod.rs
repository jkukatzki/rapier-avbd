//! Building blocks for the Augmented Vertex Block Descent (AVBD) solver backend.
//!
//! The actual solver implementation will live in this module and its submodules.
//! For now we only provide the data structures and traits required to plug an
//! AVBD solver into Rapier's integration pipeline. The mathematical details
//! follow the formulation described in the SIGGRAPH 2025 AVBD paper as outlined
//! in the accompanying design digests.

mod constraint;
mod contact;
mod params;
mod solver;
mod workspace;

pub use constraint::{AVBD_DOF, AvbdConstraint, AvbdConstraintState};
pub use contact::{AvbdAnyConstraint, AvbdContactConstraint};
pub use params::AvbdSolverParams;
pub use solver::AvbdSolver;
#[allow(unused_imports)]
pub use workspace::{AvbdBodySet, AvbdBodyState};

#[cfg(all(test, feature = "solver_avbd"))]
mod tests;
