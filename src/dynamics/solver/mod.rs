// #[cfg(not(feature = "parallel"))]
pub(crate) use self::island_solver::IslandSolver;
// #[cfg(feature = "parallel")]
// pub(crate) use self::parallel_island_solver::{ParallelIslandSolver, ThreadContext};
// #[cfg(feature = "parallel")]
// pub(self) use self::parallel_solver_constraints::ParallelSolverConstraints;
// #[cfg(feature = "parallel")]
// pub(self) use self::parallel_velocity_solver::ParallelVelocitySolver;
#[cfg(not(feature = "solver_avbd"))]
use self::velocity_solver::VelocitySolver;

use contact_constraint::*;
pub use joint_constraint::*;
use solver_body::SolverVel;

#[cfg_attr(
    feature = "solver_avbd",
    allow(dead_code, unused_imports, unused_variables)
)]
mod categorization;
#[cfg_attr(
    feature = "solver_avbd",
    allow(dead_code, unused_imports, unused_variables)
)]
mod contact_constraint;
#[cfg_attr(
    feature = "solver_avbd",
    allow(dead_code, unused_imports, unused_variables)
)]
mod interaction_groups;
// #[cfg(not(feature = "parallel"))]
#[cfg(feature = "solver_avbd")]
pub mod avbd;
mod island_solver;
#[cfg_attr(
    feature = "solver_avbd",
    allow(dead_code, unused_imports, unused_variables)
)]
mod joint_constraint;
// #[cfg(feature = "parallel")]
// mod parallel_island_solver;
// #[cfg(feature = "parallel")]
// mod parallel_solver_constraints;
// #[cfg(feature = "parallel")]
// mod parallel_velocity_solver;
#[cfg_attr(
    feature = "solver_avbd",
    allow(dead_code, unused_imports, unused_variables)
)]
mod solver_body;
// #[cfg(not(feature = "parallel"))]
// #[cfg(not(feature = "parallel"))]
#[cfg_attr(
    feature = "solver_avbd",
    allow(dead_code, unused_imports, unused_variables)
)]
mod velocity_solver;

mod motor_parameters;
pub use motor_parameters::MotorParameters;

#[cfg(feature = "solver_avbd")]
pub use self::avbd::{AvbdConstraint, AvbdConstraintState, AvbdSolver, AvbdSolverParams};

// TODO: SAFETY: restrict with bytemuck::Zeroable to make this safe.
pub unsafe fn reset_buffer<T>(buffer: &mut Vec<T>, len: usize) {
    buffer.clear();
    buffer.reserve(len);

    unsafe {
        // NOTE: writing zeros is faster than u8::MAX.
        buffer.as_mut_ptr().write_bytes(0, len);
        buffer.set_len(len);
    }
}
