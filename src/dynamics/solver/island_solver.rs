#[cfg(not(feature = "solver_avbd"))]
mod impulse {
    use crate::counters::Counters;
    use crate::dynamics::IslandManager;
    use crate::dynamics::solver::contact_constraint::ContactConstraintsSet;
    use crate::dynamics::solver::{JointConstraintsSet, VelocitySolver};
    use crate::dynamics::{IntegrationParameters, JointGraphEdge, JointIndex, RigidBodySet};
    use crate::geometry::{ContactManifold, ContactManifoldIndex};
    use crate::prelude::MultibodyJointSet;
    use parry::math::Real;

    pub struct IslandSolver {
        contact_constraints: ContactConstraintsSet,
        joint_constraints: JointConstraintsSet,
        velocity_solver: VelocitySolver,
    }

    impl Default for IslandSolver {
        fn default() -> Self {
            Self::new()
        }
    }

    impl IslandSolver {
        pub fn new() -> Self {
            Self {
                contact_constraints: ContactConstraintsSet::new(),
                joint_constraints: JointConstraintsSet::new(),
                velocity_solver: VelocitySolver::new(),
            }
        }

        #[profiling::function]
        #[allow(clippy::too_many_arguments)]
        pub fn init_and_solve(
            &mut self,
            island_id: usize,
            counters: &mut Counters,
            base_params: &IntegrationParameters,
            islands: &IslandManager,
            bodies: &mut RigidBodySet,
            manifolds: &mut [&mut ContactManifold],
            manifold_indices: &[ContactManifoldIndex],
            impulse_joints: &mut [JointGraphEdge],
            joint_indices: &[JointIndex],
            multibodies: &mut MultibodyJointSet,
        ) {
            counters.solver.velocity_assembly_time.resume();
            let num_solver_iterations = base_params.num_solver_iterations
                + islands.active_island_additional_solver_iterations(island_id);

            let mut params = *base_params;
            params.dt /= num_solver_iterations as Real;

            self.velocity_solver
                .init_solver_velocities_and_solver_bodies(
                    base_params.dt,
                    &params,
                    island_id,
                    islands,
                    bodies,
                    multibodies,
                );

            self.velocity_solver.init_constraints(
                island_id,
                islands,
                bodies,
                multibodies,
                manifolds,
                manifold_indices,
                impulse_joints,
                joint_indices,
                &mut self.contact_constraints,
                &mut self.joint_constraints,
                #[cfg(feature = "dim3")]
                params.friction_model,
            );
            counters.solver.velocity_assembly_time.pause();

            counters.solver.velocity_resolution_time.resume();
            self.velocity_solver.solve_constraints(
                &params,
                num_solver_iterations,
                bodies,
                multibodies,
                &mut self.contact_constraints,
                &mut self.joint_constraints,
            );
            counters.solver.velocity_resolution_time.pause();

            counters.solver.velocity_writeback_time.resume();
            self.joint_constraints.writeback_impulses(impulse_joints);
            self.contact_constraints.writeback_impulses(manifolds);
            self.velocity_solver.writeback_bodies(
                base_params,
                islands,
                island_id,
                bodies,
                multibodies,
            );
            counters.solver.velocity_writeback_time.pause();
        }
    }
}

#[cfg(not(feature = "solver_avbd"))]
pub(crate) use impulse::IslandSolver;

#[cfg(feature = "solver_avbd")]
mod avbd_impl {
    use crate::counters::Counters;
    use crate::dynamics::IslandManager;
    use crate::dynamics::solver::avbd::{
        AvbdAnyConstraint, AvbdConstraint, AvbdContactConstraint, AvbdSolver,
    };
    use crate::dynamics::{IntegrationParameters, JointGraphEdge, JointIndex, RigidBodySet};
    use crate::geometry::{ContactManifold, ContactManifoldIndex};
    use crate::prelude::MultibodyJointSet;
    use parry::math::Real;

    pub struct IslandSolver {
        solver: AvbdSolver,
        constraints: Vec<AvbdAnyConstraint>,
    }

    impl Default for IslandSolver {
        fn default() -> Self {
            Self::new()
        }
    }

    impl IslandSolver {
        pub fn new() -> Self {
            Self {
                solver: AvbdSolver::new(Default::default()),
                constraints: Vec::new(),
            }
        }

        #[profiling::function]
        #[allow(clippy::too_many_arguments)]
        pub fn init_and_solve(
            &mut self,
            island_id: usize,
            counters: &mut Counters,
            base_params: &IntegrationParameters,
            islands: &IslandManager,
            bodies: &mut RigidBodySet,
            manifolds: &mut [&mut ContactManifold],
            manifold_indices: &[ContactManifoldIndex],
            impulse_joints: &mut [JointGraphEdge],
            _joint_indices: &[JointIndex],
            _multibodies: &mut MultibodyJointSet,
        ) {
            let mut solver_params = base_params.avbd_solver;
            solver_params.iterations = base_params.num_solver_iterations
                + islands.active_island_additional_solver_iterations(island_id);
            *self.solver.params_mut() = solver_params;

            counters.solver.velocity_assembly_time.resume();
            self.constraints.clear();
            self.build_contact_constraints(
                &*bodies,
                manifolds,
                manifold_indices,
                solver_params.stiffness_min,
            );
            counters.solver.velocity_assembly_time.pause();

            counters.solver.velocity_resolution_time.resume();
            let dt = if solver_params.iterations > 0 {
                base_params.dt / solver_params.iterations as Real
            } else {
                base_params.dt
            };
            self.solver
                .solve(bodies, &mut self.constraints[..], dt.max(Real::EPSILON));
            counters.solver.velocity_resolution_time.pause();

            counters.solver.velocity_writeback_time.resume();
            self.writeback_contacts(manifolds);
            counters.solver.velocity_writeback_time.pause();
            let _ = impulse_joints;
        }

        fn build_contact_constraints(
            &mut self,
            bodies: &RigidBodySet,
            manifolds: &mut [&mut ContactManifold],
            manifold_indices: &[ContactManifoldIndex],
            stiffness: Real,
        ) {
            for &manifold_id in manifold_indices {
                if let Some(manifold) = manifolds.get(manifold_id) {
                    let manifold_ref: &ContactManifold = &**manifold;
                    for contact_index in 0..manifold_ref.data.solver_contacts.len() {
                        let solver_contact = &manifold_ref.data.solver_contacts[contact_index];
                        if let Some(constraint) = AvbdContactConstraint::new(
                            manifold_id,
                            contact_index,
                            manifold_ref,
                            solver_contact,
                            bodies,
                            stiffness,
                        ) {
                            self.constraints
                                .push(AvbdAnyConstraint::Contact(constraint));
                        }
                    }
                }
            }
        }

        fn writeback_contacts(&mut self, manifolds: &mut [&mut ContactManifold]) {
            for constraint in &self.constraints {
                if let Some(contact) = constraint.contact() {
                    if let Some(manifold) = manifolds.get_mut(contact.manifold_index()) {
                        if let Some(solver_contact) = (*manifold)
                            .data
                            .solver_contacts
                            .get_mut(contact.solver_contact_index())
                        {
                            solver_contact.warmstart_impulse = contact.state().lambda;
                        }
                    }
                }
            }
        }
    }
}

#[cfg(feature = "solver_avbd")]
pub(crate) use avbd_impl::IslandSolver;
