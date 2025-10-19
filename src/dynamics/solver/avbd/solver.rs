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
    pub fn solve<C>(
        &mut self,
        bodies: &mut RigidBodySet,
        island_bodies: &[RigidBodyHandle],
        constraints: &mut [C],
        dt: Real,
    ) where
        C: AvbdConstraint,
    {
        if island_bodies.is_empty() || bodies.len() == 0 || dt <= 0.0 {
            return;
        }

        self.workspace
            .initialize(bodies, island_bodies, constraints, dt);
        let warm_lambda = self.params.gamma;
        let stiffness_decay = self.params.gamma;

        for constraint in constraints.iter_mut() {
            let state = constraint.state_mut();
            state.lambda *= warm_lambda;
            state.stiffness = clamp_stiffness(state.stiffness * stiffness_decay, &self.params);
        }

        let constraint_ptr = constraints.as_mut_ptr();
        let bucket_schedule = self.workspace.constraint_buckets.clone();

        for _ in 0..self.params.iterations {
            for bucket in &bucket_schedule {
                for &constraint_index in bucket {
                    // SAFETY: constraint indices inside a bucket are unique and
                    // disjoint across buckets. We only access a single
                    // constraint at a time in this sequential implementation.
                    let constraint = unsafe { &mut *constraint_ptr.add(constraint_index) };
                    self.solve_constraint(bodies, constraint);
                }
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

        let mut scratch = self.workspace.take_scratch();
        scratch.ensure_capacity(handles.len());
        scratch.indices.clear();

        for handle in handles.iter().copied() {
            if let Some(entry_index) = self.workspace.body_map.get(&handle).copied() {
                let slot = scratch.indices.len();
                let grad = &mut scratch.gradients[slot];
                grad.fill(0.0);
                constraint.gradient(bodies, handle, grad);

                scratch.minv_gradients[slot] = self.workspace.mass_inverse(entry_index, grad);

                let mut local_hessian = [[0.0; AVBD_DOF]; AVBD_DOF];
                constraint.hessian(bodies, handle, &mut local_hessian);
                scratch.hessian_products[slot] = apply_hessian(&local_hessian, grad);

                scratch.indices.push(entry_index);
            }
        }

        if scratch.indices.is_empty() {
            self.workspace.restore_scratch(scratch);
            return;
        }

        let active_len = scratch.indices.len();

        let compliance = {
            let state = constraint.state();
            if state.stiffness <= 0.0 {
                0.0
            } else {
                1.0 / state.stiffness
            }
        };

        let mut denominator = compliance;
        for idx in 0..active_len {
            let grad = &scratch.gradients[idx];
            let minv_grad = &scratch.minv_gradients[idx];
            let hess_grad = &scratch.hessian_products[idx];
            let mut acc = 0.0;
            for i in 0..AVBD_DOF {
                acc += grad[i] * (minv_grad[i] + hess_grad[i]);
            }
            denominator += acc;
        }

        if denominator <= 0.0 {
            self.workspace.restore_scratch(scratch);
            return;
        }

        let c_val = constraint.evaluate(bodies);
        let (delta_lambda, new_lambda, new_stiffness) = {
            let state = constraint.state();
            let residual = c_val + compliance * state.lambda;
            let raw_delta_lambda = -residual / denominator;
            let relaxed_delta_lambda = self.params.alpha * raw_delta_lambda;
            let updated_lambda = state.lambda + relaxed_delta_lambda;
            let updated_stiffness =
                clamp_stiffness(state.stiffness * self.params.beta, &self.params);
            (relaxed_delta_lambda, updated_lambda, updated_stiffness)
        };

        for (entry_index, minv_grad) in scratch
            .indices
            .iter()
            .zip(scratch.minv_gradients[..active_len].iter())
        {
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

        self.workspace.restore_scratch(scratch);
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
    constraint_buckets: Vec<Vec<usize>>,
    body_bucket_usage: Vec<Vec<usize>>,
    gradients: Vec<[Real; AVBD_DOF]>,
    minv_gradients: Vec<[Real; AVBD_DOF]>,
    hessian_products: Vec<[Real; AVBD_DOF]>,
    constraint_body_indices: Vec<usize>,
}

impl SolverWorkspace {
    fn initialize<C>(
        &mut self,
        bodies: &mut RigidBodySet,
        island_bodies: &[RigidBodyHandle],
        constraints: &[C],
        dt: Real,
    ) where
        C: AvbdConstraint,
    {
        self.body_map.clear();
        self.bodies.clear();
        self.constraint_buckets.clear();
        self.body_bucket_usage.clear();

        for handle in island_bodies {
            self.ensure_body_entry(*handle, bodies, dt);
        }

        for constraint in constraints {
            for handle in constraint.bodies() {
                self.ensure_body_entry(*handle, bodies, dt);
            }
        }

        self.body_bucket_usage
            .resize_with(self.bodies.len(), Vec::new);
        for usage in &mut self.body_bucket_usage {
            usage.clear();
        }

        for (constraint_index, constraint) in constraints.iter().enumerate() {
            let handles = constraint.bodies();
            if handles.is_empty() {
                continue;
            }

            let mut color = 0usize;
            loop {
                if self.constraint_buckets.len() <= color {
                    self.constraint_buckets.push(Vec::new());
                }

                let mut conflict = false;
                for handle in handles.iter().copied() {
                    if let Some(&entry_index) = self.body_map.get(&handle) {
                        if self.body_bucket_usage[entry_index].contains(&color) {
                            conflict = true;
                            break;
                        }
                    }
                }

                if conflict {
                    color += 1;
                    continue;
                }

                for handle in handles.iter().copied() {
                    if let Some(&entry_index) = self.body_map.get(&handle) {
                        self.body_bucket_usage[entry_index].push(color);
                    }
                }

                self.constraint_buckets[color].push(constraint_index);
                break;
            }
        }
    }

    fn ensure_body_entry(&mut self, handle: RigidBodyHandle, bodies: &mut RigidBodySet, dt: Real) {
        if self.body_map.contains_key(&handle) {
            return;
        }

        let is_dynamic = bodies
            .get(handle)
            .map(|rb| rb.body_type == RigidBodyType::Dynamic && rb.is_enabled())
            .unwrap_or(false);

        if !is_dynamic {
            return;
        }

        let rb = bodies.index_mut_internal(handle);
        let initial_pose = rb.pos.position;
        let predicted_pose = rb
            .pos
            .integrate_forces_and_velocities(dt, &rb.forces, &rb.vels, &rb.mprops);

        rb.pos.position = predicted_pose;
        rb.pos.next_position = predicted_pose;

        let entry = BodyEntry {
            handle,
            inv_mass: rb.mprops.effective_inv_mass,
            inv_inertia: rb.mprops.effective_world_inv_inertia,
            initial_pose,
        };

        let index = self.bodies.len();
        self.body_map.insert(handle, index);
        self.bodies.push(entry);
    }

    fn take_scratch(&mut self) -> ConstraintScratchBuffers {
        ConstraintScratchBuffers {
            gradients: std::mem::take(&mut self.gradients),
            minv_gradients: std::mem::take(&mut self.minv_gradients),
            hessian_products: std::mem::take(&mut self.hessian_products),
            indices: std::mem::take(&mut self.constraint_body_indices),
        }
    }

    fn restore_scratch(&mut self, mut scratch: ConstraintScratchBuffers) {
        self.gradients = std::mem::take(&mut scratch.gradients);
        self.minv_gradients = std::mem::take(&mut scratch.minv_gradients);
        self.hessian_products = std::mem::take(&mut scratch.hessian_products);
        self.constraint_body_indices = std::mem::take(&mut scratch.indices);
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

struct ConstraintScratchBuffers {
    gradients: Vec<[Real; AVBD_DOF]>,
    minv_gradients: Vec<[Real; AVBD_DOF]>,
    hessian_products: Vec<[Real; AVBD_DOF]>,
    indices: Vec<usize>,
}

impl ConstraintScratchBuffers {
    fn ensure_capacity(&mut self, capacity: usize) {
        if self.gradients.len() < capacity {
            self.gradients.resize(capacity, [0.0; AVBD_DOF]);
        }
        if self.minv_gradients.len() < capacity {
            self.minv_gradients.resize(capacity, [0.0; AVBD_DOF]);
        }
        if self.hessian_products.len() < capacity {
            self.hessian_products.resize(capacity, [0.0; AVBD_DOF]);
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

fn apply_hessian(
    hessian: &[[Real; AVBD_DOF]; AVBD_DOF],
    grad: &[Real; AVBD_DOF],
) -> [Real; AVBD_DOF] {
    let mut out = [0.0; AVBD_DOF];
    for i in 0..AVBD_DOF {
        let mut acc = 0.0;
        for j in 0..AVBD_DOF {
            acc += hessian[i][j] * grad[j];
        }
        out[i] = acc;
    }
    out
}

#[cfg(test)]
impl AvbdSolver {
    pub(crate) fn debug_constraint_buckets(&self) -> &[Vec<usize>] {
        &self.workspace.constraint_buckets
    }
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

        let island_handles = vec![h1, h2];
        solver.solve(
            &mut bodies,
            &island_handles,
            std::slice::from_mut(&mut constraint),
            0.01,
        );

        let p1 = bodies[h1].pos.position.translation.vector.x;
        let p2 = bodies[h2].pos.position.translation.vector.x;
        let error = (p2 - p1 - 1.0).abs();
        assert!(error < 5.0e-2, "residual error {error}, p1 {p1}, p2 {p2}");
    }

    #[test]
    fn constraint_coloring_separates_conflicting_pairs() {
        let mut bodies = RigidBodySet::new();
        let h1 = bodies.insert(RigidBodyBuilder::dynamic().build());
        let h2 = bodies.insert(RigidBodyBuilder::dynamic().build());
        let h3 = bodies.insert(RigidBodyBuilder::dynamic().build());

        let direction = Vector::x_axis().into_inner();
        let mut c12 = DistanceConstraint::new(h1, h2, 0.0, direction);
        let mut c23 = DistanceConstraint::new(h2, h3, 0.0, direction);

        let mut solver = AvbdSolver::new(Default::default());
        let island_handles = vec![h1, h2, h3];
        let mut constraints = [c12, c23];

        solver.solve(&mut bodies, &island_handles, &mut constraints, 0.01);

        let buckets = solver.debug_constraint_buckets();
        let non_empty: Vec<_> = buckets
            .iter()
            .filter(|bucket| !bucket.is_empty())
            .map(|bucket| bucket.clone())
            .collect();
        assert_eq!(non_empty.len(), 2);
        assert_eq!(non_empty[0], vec![0]);
        assert_eq!(non_empty[1], vec![1]);
    }

    #[test]
    fn augmented_updates_raise_stiffness() {
        let mut bodies = RigidBodySet::new();
        let h1 = bodies.insert(RigidBodyBuilder::dynamic().build());
        let h2 = bodies.insert(
            RigidBodyBuilder::dynamic()
                .translation(Vector::repeat(1.0))
                .build(),
        );

        let direction = Vector::x_axis().into_inner();
        let mut constraint = DistanceConstraint::new(h1, h2, 0.0, direction);
        let initial_stiffness = constraint.state.stiffness;

        let mut solver = AvbdSolver::new(AvbdSolverParams {
            iterations: 3,
            beta: 2.0,
            ..Default::default()
        });

        let island_handles = vec![h1, h2];
        solver.solve(
            &mut bodies,
            &island_handles,
            std::slice::from_mut(&mut constraint),
            0.01,
        );

        assert!(constraint.state.lambda.abs() > 0.0);
        assert!(constraint.state.stiffness >= initial_stiffness);
    }
}
