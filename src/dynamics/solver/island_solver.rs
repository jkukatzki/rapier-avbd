#[cfg(any(not(feature = "solver_avbd"), test))]
mod impulse_impl {
    use crate::counters::Counters;
    use crate::dynamics::IslandManager;
    use crate::dynamics::solver::contact_constraint::ContactConstraintsSet;
    use crate::dynamics::solver::joint_constraint::JointConstraintsSet;
    use crate::dynamics::solver::velocity_solver::VelocitySolver;
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
pub(crate) use impulse_impl::IslandSolver;

#[cfg(feature = "solver_avbd")]
mod avbd_impl {
    use crate::counters::Counters;
    use crate::dynamics::IslandManager;
    #[cfg(any(not(feature = "solver_avbd"), test))]
    use crate::dynamics::SolverBackend;
    use crate::dynamics::solver::avbd::{
        AvbdAnyConstraint, AvbdConstraint, AvbdContactConstraint, AvbdContactFrictionConstraint,
        AvbdSolver,
    };
    use crate::dynamics::{
        IntegrationParameters, JointGraphEdge, JointIndex, RigidBodySet, RigidBodyType,
    };
    use crate::geometry::{ContactManifold, ContactManifoldIndex};
    use crate::prelude::{MultibodyJointSet, RigidBodyVelocity};
    use parry::math::Real;

    #[cfg(any(not(feature = "solver_avbd"), test))]
    use super::impulse_impl;

    pub struct IslandSolver {
        solver: AvbdSolver,
        constraints: Vec<AvbdAnyConstraint>,
        #[cfg(any(not(feature = "solver_avbd"), test))]
        impulse: impulse_impl::IslandSolver,
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
                #[cfg(any(not(feature = "solver_avbd"), test))]
                impulse: impulse_impl::IslandSolver::new(),
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
            #[cfg(any(not(feature = "solver_avbd"), test))]
            if base_params.solver_backend == SolverBackend::Impulse {
                self.constraints.clear();
                self.impulse.init_and_solve(
                    island_id,
                    counters,
                    base_params,
                    islands,
                    bodies,
                    manifolds,
                    manifold_indices,
                    impulse_joints,
                    joint_indices,
                    multibodies,
                );
                return;
            }

            #[cfg(all(feature = "solver_avbd", not(test)))]
            let _ = (impulse_joints, joint_indices, multibodies);

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
            if self.constraints.is_empty() {
                self.integrate_unconstrained_bodies(island_id, islands, bodies, base_params);
                counters.solver.velocity_resolution_time.pause();
                return;
            }

            let dt = base_params.dt.max(Real::EPSILON);
            self.solver.solve(bodies, &mut self.constraints[..], dt);
            counters.solver.velocity_resolution_time.pause();

            counters.solver.velocity_writeback_time.resume();
            self.writeback_contacts(manifolds);
            counters.solver.velocity_writeback_time.pause();
            #[cfg(all(feature = "solver_avbd", not(test)))]
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
                        if let Some(contact_constraint) = AvbdContactConstraint::new(
                            manifold_id,
                            contact_index,
                            manifold_ref,
                            solver_contact,
                            bodies,
                            stiffness,
                        ) {
                            let boxed = Box::new(contact_constraint);
                            let friction_constraints = AvbdContactFrictionConstraint::from_contact(
                                boxed.as_ref(),
                                solver_contact,
                                stiffness,
                            );
                            self.constraints.push(AvbdAnyConstraint::Contact(boxed));
                            self.constraints.extend(
                                friction_constraints
                                    .into_iter()
                                    .map(AvbdAnyConstraint::ContactFriction),
                            );
                        }
                    }
                }
            }
        }

        fn writeback_contacts(&mut self, manifolds: &mut [&mut ContactManifold]) {
            let dt = self.solver.timestep();
            for constraint in &self.constraints {
                match constraint {
                    AvbdAnyConstraint::Contact(contact_box) => {
                        let contact = contact_box.as_ref();
                        if let Some(manifold) = manifolds.get_mut(contact.manifold_index()) {
                            let lambda = contact.state().lambda;
                            let impulse = if dt > 0.0 { lambda / dt } else { 0.0 };
                            if let Some(solver_contact) = (*manifold)
                                .data
                                .solver_contacts
                                .get_mut(contact.solver_contact_index())
                            {
                                solver_contact.warmstart_impulse = lambda;
                                solver_contact.is_new = 0.0;
                            }

                            if let Some(point) =
                                manifold.points.get_mut(contact.contact_point_index())
                            {
                                point.data.impulse = impulse.max(0.0);
                            }
                        }
                    }
                    AvbdAnyConstraint::ContactFriction(friction) => {
                        if let Some(manifold) = manifolds.get_mut(friction.manifold_index()) {
                            let lambda = friction.state().lambda;
                            let impulse = if dt > 0.0 { lambda / dt } else { 0.0 };
                            if let Some(solver_contact) = (*manifold)
                                .data
                                .solver_contacts
                                .get_mut(friction.solver_contact_index())
                            {
                                let mut warmstart = solver_contact.warmstart_tangent_impulse;
                                warmstart[friction.tangent_component()] = lambda;
                                solver_contact.warmstart_tangent_impulse = warmstart;
                                solver_contact.is_new = 0.0;
                            }

                            if let Some(point) =
                                manifold.points.get_mut(friction.contact_point_index())
                            {
                                let mut tangent = point.data.tangent_impulse;
                                tangent[friction.tangent_component()] = impulse;
                                point.data.tangent_impulse = tangent;

                                let mut warmstart = point.data.warmstart_tangent_impulse;
                                warmstart[friction.tangent_component()] = lambda;
                                point.data.warmstart_tangent_impulse = warmstart;
                            }
                        }
                    }
                }
            }
        }

        fn integrate_unconstrained_bodies(
            &mut self,
            island_id: usize,
            islands: &IslandManager,
            bodies: &mut RigidBodySet,
            params: &IntegrationParameters,
        ) {
            let _ = self;
            let dt = params.dt;
            let inv_dt = params.inv_dt();

            for handle in islands.active_island(island_id) {
                let rb = bodies.index_mut_internal(*handle);

                if rb.body_type != RigidBodyType::Dynamic || !rb.is_enabled() {
                    continue;
                }

                let mut new_vels = rb.forces.integrate(dt, &rb.vels, &rb.mprops);
                new_vels = new_vels.apply_damping(dt, &rb.damping);
                rb.vels = new_vels;
                rb.pos.next_position =
                    new_vels.integrate(dt, &rb.pos.position, &rb.mprops.local_mprops.local_com);

                if rb.ccd.ccd_enabled {
                    rb.ccd_vels = rb
                        .pos
                        .interpolate_velocity(inv_dt, rb.local_center_of_mass());
                } else {
                    rb.ccd_vels = RigidBodyVelocity::zero();
                }
            }
        }
    }
}

#[cfg(feature = "solver_avbd")]
pub(crate) use avbd_impl::IslandSolver;
