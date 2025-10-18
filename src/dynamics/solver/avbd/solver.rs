use std::collections::HashMap;

use crate::dynamics::{RigidBodyHandle, RigidBodySet, RigidBodyType};
use crate::math::{ANG_DIM, AngVector, AngularInertia, Isometry, Real, SPATIAL_DIM, Vector};
use crate::na;
#[cfg(feature = "dim3")]
use crate::utils::SimdAngularInertia;

use super::{AVBD_DOF, AvbdConstraint, AvbdSolverParams};

const LINEAR_DOF: usize = SPATIAL_DIM - ANG_DIM;

#[derive(Clone)]
struct BodyEntry {
    handle: RigidBodyHandle,
    inv_mass: Vector<Real>,
    inv_inertia: AngularInertia<Real>,
    initial_pose: Isometry<Real>,
}

/// Augmented Vertex Block Descent solver implementation.
pub struct AvbdSolver {
    params: AvbdSolverParams,
    workspace: SolverWorkspace,
}

impl AvbdSolver {
    /// Creates a new solver with the provided parameters.
    pub fn new(params: AvbdSolverParams) -> Self {
        Self {
            params,
            workspace: SolverWorkspace::default(),
        }
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
    pub fn solve<C>(&mut self, bodies: &mut RigidBodySet, constraints: &mut [C], dt: Real)
    where
        C: AvbdConstraint,
    {
        if constraints.is_empty() || bodies.len() == 0 || dt <= 0.0 {
            return;
        }

        self.workspace.initialize(bodies, constraints, dt);
        let warm_lambda = self.params.alpha * self.params.gamma;
        let stiffness_decay = self.params.gamma;

        for constraint in constraints.iter_mut() {
            let state = constraint.state_mut();
            state.lambda *= warm_lambda;
            state.stiffness = clamp_stiffness(state.stiffness * stiffness_decay, &self.params);
        }

        for _ in 0..self.params.iterations {
            for constraint in constraints.iter_mut() {
                self.solve_constraint(bodies, constraint);
            }
        }

        self.workspace.writeback(bodies, dt);
    }

    fn solve_constraint<C>(&mut self, bodies: &mut RigidBodySet, constraint: &mut C)
    where
        C: AvbdConstraint,
    {
        let handles = constraint.bodies();
        if handles.is_empty() {
            return;
        }

        let mut gradients = Vec::with_capacity(handles.len());
        let mut minv_gradients = Vec::with_capacity(handles.len());
        let mut entries = Vec::with_capacity(handles.len());

        for handle in handles.iter().copied() {
            if let Some(entry_index) = self.workspace.body_map.get(&handle).copied() {
                let mut grad = [0.0; AVBD_DOF];
                constraint.gradient(bodies, handle, &mut grad);
                let minv_grad = self.workspace.mass_inverse(entry_index, &grad);
                gradients.push(grad);
                minv_gradients.push(minv_grad);
                entries.push(entry_index);
            }
        }

        if entries.is_empty() {
            return;
        }

        let compliance = {
            let state = constraint.state();
            if state.stiffness <= 0.0 {
                0.0
            } else {
                1.0 / state.stiffness
            }
        };

        let mut denominator = compliance;
        for (grad, minv_grad) in gradients.iter().zip(minv_gradients.iter()) {
            let mut acc = 0.0;
            for i in 0..AVBD_DOF {
                acc += grad[i] * minv_grad[i];
            }
            denominator += acc;
        }

        if denominator <= 0.0 {
            return;
        }

        let c_val = constraint.evaluate(bodies);
        let (delta_lambda, new_lambda, new_stiffness) = {
            let state = constraint.state();
            let alpha_tilde = compliance;
            let delta_lambda = -(c_val + alpha_tilde * state.lambda) / denominator;
            let updated_lambda = state.lambda + delta_lambda;
            let updated_stiffness =
                clamp_stiffness(state.stiffness * self.params.beta, &self.params);
            (delta_lambda, updated_lambda, updated_stiffness)
        };

        for (entry_index, minv_grad) in entries.iter().zip(minv_gradients.iter()) {
            let mut delta = [0.0; AVBD_DOF];
            for i in 0..AVBD_DOF {
                delta[i] = minv_grad[i] * delta_lambda;
            }
            self.workspace
                .apply_body_delta(*entry_index, &delta, bodies);
        }

        let state = constraint.state_mut();
        state.lambda = new_lambda;
        state.stiffness = new_stiffness;
    }
}

fn clamp_stiffness(value: Real, params: &AvbdSolverParams) -> Real {
    let clamped = value.max(params.stiffness_min).min(params.stiffness_max);
    if clamped.is_finite() {
        clamped
    } else {
        params.stiffness_max
    }
}

#[derive(Default)]
struct SolverWorkspace {
    body_map: HashMap<RigidBodyHandle, usize>,
    bodies: Vec<BodyEntry>,
}

impl SolverWorkspace {
    fn initialize<C>(&mut self, bodies: &mut RigidBodySet, constraints: &[C], dt: Real)
    where
        C: AvbdConstraint,
    {
        self.body_map.clear();
        self.bodies.clear();

        for constraint in constraints {
            for handle in constraint.bodies() {
                if self.body_map.contains_key(handle) {
                    continue;
                }

                let is_dynamic = bodies
                    .get(*handle)
                    .map(|rb| rb.body_type == RigidBodyType::Dynamic && rb.is_enabled())
                    .unwrap_or(false);

                if !is_dynamic {
                    continue;
                }

                let rb = bodies.index_mut_internal(*handle);
                let initial_pose = rb.pos.position;
                let predicted_pose = rb
                    .pos
                    .integrate_forces_and_velocities(dt, &rb.forces, &rb.vels, &rb.mprops);

                rb.pos.position = predicted_pose;
                rb.pos.next_position = predicted_pose;

                let entry = BodyEntry {
                    handle: *handle,
                    inv_mass: rb.mprops.effective_inv_mass,
                    inv_inertia: rb.mprops.effective_world_inv_inertia,
                    initial_pose,
                };

                let index = self.bodies.len();
                self.body_map.insert(*handle, index);
                self.bodies.push(entry);
            }
        }
    }

    fn mass_inverse(&self, entry_index: usize, grad: &[Real; AVBD_DOF]) -> [Real; AVBD_DOF] {
        let entry = &self.bodies[entry_index];
        let (grad_lin, grad_ang) = split_gradient(grad);
        let lin = grad_lin.component_mul(&entry.inv_mass);

        #[cfg(feature = "dim2")]
        let ang = grad_ang * entry.inv_inertia;

        #[cfg(feature = "dim3")]
        let ang = entry.inv_inertia.transform_vector(grad_ang);

        merge_components(&lin, &ang)
    }

    fn apply_body_delta(
        &mut self,
        entry_index: usize,
        delta: &[Real; AVBD_DOF],
        bodies: &mut RigidBodySet,
    ) {
        if let Some(rb) = bodies.get_mut(self.bodies[entry_index].handle) {
            let (delta_lin, delta_ang) = split_gradient(delta);
            rb.pos.position.translation.vector += delta_lin;
            rb.pos.next_position.translation.vector = rb.pos.position.translation.vector;

            #[cfg(feature = "dim2")]
            {
                let rotation = na::Rotation2::new(delta_ang.x);
                rb.pos.position.rotation = rotation * rb.pos.position.rotation;
                rb.pos.next_position.rotation = rb.pos.position.rotation;
            }

            #[cfg(feature = "dim3")]
            {
                let rot = na::UnitQuaternion::from_scaled_axis(delta_ang);
                rb.pos.position.rotation = rot * rb.pos.position.rotation;
                rb.pos.next_position.rotation = rb.pos.position.rotation;
            }
        }
    }

    fn writeback(&mut self, bodies: &mut RigidBodySet, dt: Real) {
        for entry in &self.bodies {
            if let Some(rb) = bodies.get_mut(entry.handle) {
                let final_pose = rb.pos.position;
                let lin_delta =
                    final_pose.translation.vector - entry.initial_pose.translation.vector;
                let linvel = lin_delta / dt;

                #[cfg(feature = "dim2")]
                let angvel = {
                    let final_angle = final_pose.rotation.angle();
                    let initial_angle = entry.initial_pose.rotation.angle();
                    AngVector::from_element((final_angle - initial_angle) / dt)
                };

                #[cfg(feature = "dim3")]
                let angvel = {
                    let delta = final_pose.rotation * entry.initial_pose.rotation.inverse();
                    delta.scaled_axis() / dt
                };

                rb.vels.linvel = linvel;
                rb.vels.angvel = angvel;
                rb.mprops
                    .update_world_mass_properties(rb.body_type, &rb.pos.position);
                rb.activation.wake_up(true);
            }
        }
    }
}

fn split_gradient(vector: &[Real; AVBD_DOF]) -> (Vector<Real>, AngVector<Real>) {
    let mut lin = Vector::zeros();
    for i in 0..LINEAR_DOF {
        lin[i] = vector[i];
    }

    let mut ang = AngVector::zeros();
    for i in 0..ANG_DIM {
        ang[i] = vector[LINEAR_DOF + i];
    }

    (lin, ang)
}

fn merge_components(linear: &Vector<Real>, angular: &AngVector<Real>) -> [Real; AVBD_DOF] {
    let mut out = [0.0; AVBD_DOF];

    for i in 0..LINEAR_DOF {
        out[i] = linear[i];
    }

    for i in 0..ANG_DIM {
        out[LINEAR_DOF + i] = angular[i];
    }

    out
}

#[cfg(test)]
mod tests {
    use super::super::AvbdConstraintState;
    use super::*;
    use crate::dynamics::{RigidBodyBuilder, RigidBodySet};
    use crate::math::Vector;

    struct DistanceConstraint {
        bodies: [RigidBodyHandle; 2],
        state: AvbdConstraintState,
        target: Real,
        direction: Vector<Real>,
    }

    impl DistanceConstraint {
        fn new(
            handle1: RigidBodyHandle,
            handle2: RigidBodyHandle,
            target: Real,
            direction: Vector<Real>,
        ) -> Self {
            Self {
                bodies: [handle1, handle2],
                state: AvbdConstraintState::new(0.0, 1000.0),
                target,
                direction,
            }
        }
    }

    impl AvbdConstraint for DistanceConstraint {
        fn bodies(&self) -> &[RigidBodyHandle] {
            &self.bodies
        }

        fn state_mut(&mut self) -> &mut AvbdConstraintState {
            &mut self.state
        }

        fn state(&self) -> &AvbdConstraintState {
            &self.state
        }

        fn evaluate(&self, bodies: &RigidBodySet) -> Real {
            let p1 = bodies[self.bodies[0]].pos.position.translation.vector;
            let p2 = bodies[self.bodies[1]].pos.position.translation.vector;
            (self.direction.dot(&(p2 - p1))) - self.target
        }

        fn gradient(
            &self,
            _bodies: &RigidBodySet,
            body: RigidBodyHandle,
            out: &mut [Real; AVBD_DOF],
        ) {
            out.fill(0.0);
            if body == self.bodies[0] {
                for i in 0..LINEAR_DOF {
                    out[i] = -self.direction[i];
                }
            } else if body == self.bodies[1] {
                for i in 0..LINEAR_DOF {
                    out[i] = self.direction[i];
                }
            }
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

    #[test]
    fn pulls_bodies_towards_target_distance() {
        let mut bodies = RigidBodySet::new();
        let h1 = bodies.insert(
            RigidBodyBuilder::dynamic()
                .translation(Vector::zeros())
                .additional_mass(1.0)
                .build(),
        );
        let h2 = bodies.insert(
            RigidBodyBuilder::dynamic()
                .translation(Vector::repeat(2.0))
                .additional_mass(1.0)
                .build(),
        );

        if let Some(rb) = bodies.get_mut(h1) {
            rb.mprops.effective_inv_mass = Vector::repeat(1.0);
        }
        if let Some(rb) = bodies.get_mut(h2) {
            rb.mprops.effective_inv_mass = Vector::repeat(1.0);
        }

        let direction = Vector::x_axis().into_inner();
        let mut constraint = DistanceConstraint::new(h1, h2, 1.0, direction);

        let mut solver = AvbdSolver::new(AvbdSolverParams {
            iterations: 10,
            ..Default::default()
        });

        solver.solve(&mut bodies, std::slice::from_mut(&mut constraint), 0.01);

        let p1 = bodies[h1].pos.position.translation.vector.x;
        let p2 = bodies[h2].pos.position.translation.vector.x;
        let error = (p2 - p1 - 1.0).abs();
        assert!(error < 5.0e-2, "residual error {error}, p1 {p1}, p2 {p2}");
    }
}
