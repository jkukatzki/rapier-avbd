//! Building blocks for the Augmented Vertex Block Descent (AVBD) solver backend.
//!
//! The actual solver implementation will live in this module and its submodules.
//! For now we only provide the data structures and traits required to plug an
//! AVBD solver into Rapier's integration pipeline. The mathematical details
//! follow the formulation described in the SIGGRAPH 2025 AVBD paper as outlined
//! in the accompanying design digests.

mod constraint;
mod params;
mod solver;

pub use constraint::{AVBD_DOF, AvbdConstraint, AvbdConstraintState};
pub use params::AvbdSolverParams;
pub use solver::{AvbdSolveReport, AvbdSolver};
