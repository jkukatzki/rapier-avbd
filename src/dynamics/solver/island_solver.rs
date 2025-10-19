use crate::counters::Counters;
use crate::dynamics::{
    IntegrationParameters, IslandManager, JointGraphEdge, JointIndex, MultibodyJointSet,
    RigidBodySet, SolverBackend,
};
use crate::geometry::{ContactManifold, ContactManifoldIndex};
use parry::math::Real;

#[cfg(feature = "solver_avbd")]
use super::avbd::{
    AVBD_DOF, AvbdConstraint, AvbdConstraintState, AvbdSolveReport, AvbdSolver, AvbdSolverParams,
};
#[cfg(feature = "solver_impulse")]
use super::{JointConstraintsSet, VelocitySolver};
#[cfg(feature = "solver_avbd")]
use crate::dynamics::RigidBodyHandle;
#[cfg(feature = "solver_impulse")]
use crate::dynamics::solver::contact_constraint::ContactConstraintsSet;

pub struct IslandSolver {
    #[cfg(feature = "solver_impulse")]
    contact_constraints: ContactConstraintsSet,
    #[cfg(feature = "solver_impulse")]
    joint_constraints: JointConstraintsSet,
    #[cfg(feature = "solver_impulse")]
    velocity_solver: VelocitySolver,
    #[cfg(feature = "solver_avbd")]
    avbd_solver: AvbdIslandSolver,
}

impl Default for IslandSolver {
    fn default() -> Self {
        Self::new()
    }
}

impl IslandSolver {
    pub fn new() -> Self {
        Self {
            #[cfg(feature = "solver_impulse")]
            contact_constraints: ContactConstraintsSet::new(),
            #[cfg(feature = "solver_impulse")]
            joint_constraints: JointConstraintsSet::new(),
            #[cfg(feature = "solver_impulse")]
            velocity_solver: VelocitySolver::new(),
            #[cfg(feature = "solver_avbd")]
            avbd_solver: AvbdIslandSolver::new(),
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
        match base_params.solver_backend {
            SolverBackend::Impulse => {
                #[cfg(feature = "solver_impulse")]
                {
                    self.solve_with_impulse(
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
                }

                #[cfg(not(feature = "solver_impulse"))]
                {
                    panic!(
                        "`solver_impulse` feature disabled but impulse solver backend requested"
                    );
                }
            }
            #[cfg(feature = "solver_avbd")]
            SolverBackend::Avbd => {
                self.solve_with_avbd(
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
            }
        }
    }

    #[cfg(feature = "solver_impulse")]
    #[allow(clippy::too_many_arguments)]
    fn solve_with_impulse(
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
        self.velocity_solver
            .writeback_bodies(base_params, islands, island_id, bodies, multibodies);
        counters.solver.velocity_writeback_time.pause();
    }

    #[cfg(feature = "solver_avbd")]
    pub(crate) fn take_avbd_report(&mut self) -> Option<AvbdSolveReport> {
        self.avbd_solver.take_report()
    }

    #[cfg(feature = "solver_avbd")]
    #[allow(clippy::too_many_arguments)]
    fn solve_with_avbd(
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
        let dt = base_params.dt;

        self.avbd_solver.apply_params(&base_params.avbd_params);
        counters.solver.velocity_assembly_time.resume();
        self.avbd_solver.rebuild(
            island_id,
            islands,
            manifolds,
            manifold_indices,
            impulse_joints,
            joint_indices,
        );
        counters.solver.velocity_assembly_time.pause();

        counters.solver.velocity_resolution_time.resume();
        self.avbd_solver.solve(bodies, dt);
        counters.solver.velocity_resolution_time.pause();

        counters.solver.velocity_writeback_time.resume();
        self.avbd_solver.post_solve(bodies, multibodies);
        counters.solver.velocity_writeback_time.pause();
    }
}

#[cfg(feature = "solver_avbd")]
struct AvbdIslandSolver {
    solver: AvbdSolver,
    island_bodies: Vec<RigidBodyHandle>,
    constraints: Vec<PlaceholderConstraint>,
    last_report: Option<AvbdSolveReport>,
}

#[cfg(feature = "solver_avbd")]
impl AvbdIslandSolver {
    fn new() -> Self {
        Self {
            solver: AvbdSolver::new(Default::default()),
            island_bodies: Vec::new(),
            constraints: Vec::new(),
            last_report: None,
        }
    }

    fn apply_params(&mut self, params: &AvbdSolverParams) {
        *self.solver.params_mut() = *params;
    }

    fn rebuild(
        &mut self,
        island_id: usize,
        islands: &IslandManager,
        manifolds: &mut [&mut ContactManifold],
        manifold_indices: &[ContactManifoldIndex],
        impulse_joints: &mut [JointGraphEdge],
        joint_indices: &[JointIndex],
    ) {
        self.island_bodies.clear();
        self.island_bodies
            .extend_from_slice(islands.active_island(island_id));

        self.constraints.clear();
        let stiffness_seed = self.solver.params().stiffness_min;

        for &manifold_id in manifold_indices {
            if let Some(manifold) = manifolds.get(manifold_id) {
                if let (Some(b1), Some(b2)) = (manifold.data.rigid_body1, manifold.data.rigid_body2)
                {
                    self.constraints
                        .push(PlaceholderConstraint::from_pair(b1, b2, stiffness_seed));
                }
            }
        }

        for &joint_id in joint_indices {
            if let Some(edge) = impulse_joints.get(joint_id) {
                let joint = &edge.weight;
                self.constraints.push(PlaceholderConstraint::from_pair(
                    joint.body1,
                    joint.body2,
                    stiffness_seed,
                ));
            }
        }
    }

    fn solve(&mut self, bodies: &mut RigidBodySet, dt: Real) {
        self.solver
            .solve(bodies, &self.island_bodies, &mut self.constraints, dt);
        self.last_report = Some(self.solver.take_report());
    }

    fn post_solve(&mut self, _bodies: &mut RigidBodySet, _multibodies: &mut MultibodyJointSet) {}

    fn take_report(&mut self) -> Option<AvbdSolveReport> {
        self.last_report.take()
    }
}

#[cfg(feature = "solver_avbd")]
struct PlaceholderConstraint {
    bodies: Vec<RigidBodyHandle>,
    state: AvbdConstraintState,
}

#[cfg(feature = "solver_avbd")]
impl PlaceholderConstraint {
    fn from_pair(body1: RigidBodyHandle, body2: RigidBodyHandle, stiffness: Real) -> Self {
        Self {
            bodies: vec![body1, body2],
            state: AvbdConstraintState::new(0.0, stiffness),
        }
    }
}

#[cfg(feature = "solver_avbd")]
impl AvbdConstraint for PlaceholderConstraint {
    fn bodies(&self) -> &[RigidBodyHandle] {
        &self.bodies
    }

    fn state_mut(&mut self) -> &mut AvbdConstraintState {
        &mut self.state
    }

    fn state(&self) -> &AvbdConstraintState {
        &self.state
    }

    fn evaluate(&self, _bodies: &RigidBodySet) -> Real {
        0.0
    }

    fn gradient(&self, _bodies: &RigidBodySet, _body: RigidBodyHandle, out: &mut [Real; AVBD_DOF]) {
        out.fill(0.0);
    }

    fn hessian(
        &self,
        _bodies: &RigidBodySet,
        _body: RigidBodyHandle,
        out: &mut [[Real; AVBD_DOF]; AVBD_DOF],
    ) {
        for row in out.iter_mut() {
            row.fill(0.0);
        }
    }
}

#[cfg(all(test, feature = "solver_avbd"))]
mod tests {
    use super::*;
    use crate::dynamics::{RigidBodyBuilder, RigidBodySet};

    #[test]
    fn avbd_backend_collects_island_artifacts() {
        let mut bodies = RigidBodySet::new();
        let h1 = bodies.insert(RigidBodyBuilder::dynamic().build());
        let h2 = bodies.insert(RigidBodyBuilder::dynamic().build());

        let mut islands = IslandManager::new();
        islands.active_set = vec![h1, h2];
        islands.active_islands = vec![0, 2];
        islands.active_islands_additional_solver_iterations = vec![0, 0];

        let mut manifold = ContactManifold::default();
        manifold.data.rigid_body1 = Some(h1);
        manifold.data.rigid_body2 = Some(h2);
        let mut manifolds_storage = vec![manifold];
        let mut manifolds: Vec<&mut ContactManifold> = manifolds_storage.iter_mut().collect();
        let manifold_indices = vec![0usize];

        let mut impulse_joints: Vec<JointGraphEdge> = Vec::new();
        let joint_indices: Vec<JointIndex> = Vec::new();
        let mut multibodies = MultibodyJointSet::new();
        let mut counters = Counters::new(false);

        let mut solver = IslandSolver::new();
        let mut params = IntegrationParameters::default();
        params.solver_backend = SolverBackend::Avbd;

        solver.init_and_solve(
            0,
            &mut counters,
            &params,
            &islands,
            &mut bodies,
            &mut manifolds[..],
            &manifold_indices,
            &mut impulse_joints[..],
            &joint_indices,
            &mut multibodies,
        );

        assert_eq!(solver.avbd_solver.island_bodies.len(), 2);
        assert_eq!(solver.avbd_solver.constraints.len(), 1);
    }
}
