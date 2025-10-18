use std::collections::HashMap;

use crate::dynamics::{RigidBodyHandle, RigidBodySet, RigidBodyType};
use crate::math::{ANG_DIM, AngVector, AngularInertia, Isometry, Real, SPATIAL_DIM, Vector};
use crate::na;
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
        let color_schedule = self.workspace.color_schedule.clone();
        let warm_lambda = self.params.alpha * self.params.gamma;
        let stiffness_decay = self.params.gamma;

        for constraint in constraints.iter_mut() {
            let state = constraint.state_mut();
            state.lambda *= warm_lambda;
            state.stiffness = clamp_stiffness(state.stiffness * stiffness_decay, &self.params);
        }

        for _ in 0..self.params.iterations {
            for bucket in &color_schedule {
                for &constraint_index in bucket {
                    if let Some(constraint) = constraints.get_mut(constraint_index) {
                        self.solve_constraint(bodies, constraint_index, constraint);
                    }
                }
            }
        }

        self.workspace.writeback(bodies, dt);
    }

    fn solve_constraint<C>(
        &mut self,
        bodies: &mut RigidBodySet,
        constraint_index: usize,
        constraint: &mut C,
    ) where
        C: AvbdConstraint,
    {
        let entry_indices = match self.workspace.constraint_entries.get(constraint_index) {
            Some(entries) if !entries.is_empty() => entries.clone(),
            _ => return,
        };

        let workspace = &mut self.workspace;
        workspace.ensure_local_buffers(entry_indices.len());

        for (slot, entry_index) in entry_indices.iter().enumerate() {
            let handle = workspace.bodies[*entry_index].handle;

            {
                let grad = &mut workspace.gradients[slot];
                grad.fill(0.0);
                constraint.gradient(bodies, handle, grad);
            }

            {
                let hessian = &mut workspace.hessians[slot];
                for row in hessian.iter_mut() {
                    row.fill(0.0);
                }
                constraint.hessian(bodies, handle, hessian);
            }

            let grad_ref = &workspace.gradients[slot];
            let hessian_ref = &workspace.hessians[slot];
            workspace.minv_gradients[slot] = workspace.solve_local_system(
                *entry_index,
                grad_ref,
                hessian_ref,
                self.params.regularization,
            );
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
        for (grad, minv_grad) in workspace
            .gradients
            .iter()
            .take(entry_indices.len())
            .zip(workspace.minv_gradients.iter().take(entry_indices.len()))
        {
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

        for (slot, entry_index) in entry_indices.iter().enumerate() {
            let minv_grad = workspace.minv_gradients[slot];
            let mut delta = [0.0; AVBD_DOF];
            for i in 0..AVBD_DOF {
                delta[i] = minv_grad[i] * delta_lambda;
            }
            workspace.apply_body_delta(*entry_index, &delta, bodies);
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
    gradients: Vec<[Real; AVBD_DOF]>,
    hessians: Vec<[[Real; AVBD_DOF]; AVBD_DOF]>,
    minv_gradients: Vec<[Real; AVBD_DOF]>,
    constraint_entries: Vec<Vec<usize>>,
    constraint_colors: Vec<usize>,
    body_constraints: Vec<Vec<usize>>,
    color_buckets: Vec<Vec<usize>>,
    color_schedule: Vec<Vec<usize>>,
    active_constraints: Vec<usize>,
    tmp_color_marks: Vec<bool>,
    tmp_used_colors: Vec<usize>,
    dt: Real,
}

impl SolverWorkspace {
    fn initialize<C>(&mut self, bodies: &mut RigidBodySet, constraints: &[C], dt: Real)
    where
        C: AvbdConstraint,
    {
        self.dt = dt;
        self.body_map.clear();
        self.bodies.clear();
        self.color_schedule.clear();

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

        self.body_constraints.truncate(self.bodies.len());
        self.body_constraints
            .resize_with(self.bodies.len(), Vec::new);
        for list in &mut self.body_constraints {
            list.clear();
        }

        self.constraint_entries.truncate(constraints.len());
        self.constraint_entries
            .resize_with(constraints.len(), Vec::new);
        for entries in &mut self.constraint_entries {
            entries.clear();
        }

        self.constraint_colors.resize(constraints.len(), usize::MAX);
        self.active_constraints.clear();

        for (constraint_index, constraint) in constraints.iter().enumerate() {
            let entries = &mut self.constraint_entries[constraint_index];
            for handle in constraint.bodies() {
                if let Some(&entry_index) = self.body_map.get(handle) {
                    entries.push(entry_index);
                }
            }
            entries.sort_unstable();
            entries.dedup();

            if entries.is_empty() {
                continue;
            }

            self.active_constraints.push(constraint_index);
            for &entry_index in entries.iter() {
                if let Some(body_constraints) = self.body_constraints.get_mut(entry_index) {
                    body_constraints.push(constraint_index);
                }
            }
        }

        self.compute_constraint_coloring();
    }

    fn ensure_local_buffers(&mut self, count: usize) {
        if self.gradients.len() < count {
            self.gradients.resize(count, [0.0; AVBD_DOF]);
        }
        if self.hessians.len() < count {
            self.hessians.resize(count, [[0.0; AVBD_DOF]; AVBD_DOF]);
        }
        if self.minv_gradients.len() < count {
            self.minv_gradients.resize(count, [0.0; AVBD_DOF]);
        }
    }

    fn compute_constraint_coloring(&mut self) {
        for color in &mut self.constraint_colors {
            *color = usize::MAX;
        }

        for bucket in &mut self.color_buckets {
            bucket.clear();
        }

        self.tmp_color_marks.clear();
        self.tmp_used_colors.clear();

        for &constraint_index in &self.active_constraints {
            let entries = &self.constraint_entries[constraint_index];
            self.tmp_used_colors.clear();

            for &entry_index in entries {
                if let Some(neighbors) = self.body_constraints.get(entry_index) {
                    for &neighbor in neighbors {
                        if neighbor == constraint_index {
                            continue;
                        }
                        let color = self.constraint_colors[neighbor];
                        if color != usize::MAX {
                            if color >= self.tmp_color_marks.len() {
                                self.tmp_color_marks.resize(color + 1, false);
                            }
                            if !self.tmp_color_marks[color] {
                                self.tmp_color_marks[color] = true;
                                self.tmp_used_colors.push(color);
                            }
                        }
                    }
                }
            }

            let mut color = 0;
            loop {
                if color >= self.tmp_color_marks.len() {
                    self.tmp_color_marks.push(false);
                    break;
                }
                if !self.tmp_color_marks[color] {
                    break;
                }
                color += 1;
            }

            self.constraint_colors[constraint_index] = color;
            if self.color_buckets.len() <= color {
                self.color_buckets.resize_with(color + 1, Vec::new);
            }
            self.color_buckets[color].push(constraint_index);

            for used_color in self.tmp_used_colors.drain(..) {
                self.tmp_color_marks[used_color] = false;
            }
        }

        self.color_schedule.clear();
        for bucket in &self.color_buckets {
            if bucket.is_empty() {
                continue;
            }
            self.color_schedule.push(bucket.clone());
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

    fn solve_local_system(
        &self,
        entry_index: usize,
        grad: &[Real; AVBD_DOF],
        hessian: &[[Real; AVBD_DOF]; AVBD_DOF],
        regularization: Real,
    ) -> [Real; AVBD_DOF] {
        let entry = &self.bodies[entry_index];
        let dt_sq = if self.dt > 0.0 {
            self.dt * self.dt
        } else {
            1.0
        };

        let mut local = na::SMatrix::<Real, AVBD_DOF, AVBD_DOF>::zeros();

        for i in 0..LINEAR_DOF {
            let inv_mass = entry.inv_mass[i];
            if inv_mass > 0.0 {
                local[(i, i)] = (1.0 / inv_mass) / dt_sq;
            } else {
                local[(i, i)] = regularization;
            }
        }

        #[cfg(feature = "dim2")]
        {
            let inertia = entry.inv_inertia.inverse();
            local[(LINEAR_DOF, LINEAR_DOF)] += inertia / dt_sq;
        }

        #[cfg(feature = "dim3")]
        {
            let inertia_matrix = entry.inv_inertia.inverse().into_matrix();
            for r in 0..ANG_DIM {
                for c in 0..ANG_DIM {
                    local[(LINEAR_DOF + r, LINEAR_DOF + c)] += inertia_matrix[(r, c)] / dt_sq;
                }
            }
        }

        for row in 0..AVBD_DOF {
            for col in 0..AVBD_DOF {
                local[(row, col)] += hessian[row][col];
            }
        }

        for i in 0..AVBD_DOF {
            local[(i, i)] += regularization;
        }

        let grad_vec = na::SVector::<Real, AVBD_DOF>::from_row_slice(grad);

        if let Some(cholesky) = na::Cholesky::new(local) {
            let solved = cholesky.solve(&grad_vec);
            let mut out = [0.0; AVBD_DOF];
            for i in 0..AVBD_DOF {
                out[i] = solved[i];
            }
            out
        } else {
            self.mass_inverse(entry_index, grad)
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

    fn along_x(value: Real) -> Vector<Real> {
        let mut v = Vector::zeros();
        v[0] = value;
        v
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

    #[test]
    fn resolves_chain_of_constraints_without_conflicts() {
        let mut bodies = RigidBodySet::new();
        let h1 = bodies.insert(
            RigidBodyBuilder::dynamic()
                .translation(along_x(0.0))
                .additional_mass(1.0)
                .build(),
        );
        let h2 = bodies.insert(
            RigidBodyBuilder::dynamic()
                .translation(along_x(3.0))
                .additional_mass(1.0)
                .build(),
        );
        let h3 = bodies.insert(
            RigidBodyBuilder::dynamic()
                .translation(along_x(6.0))
                .additional_mass(1.0)
                .build(),
        );

        for handle in [h1, h2, h3] {
            if let Some(rb) = bodies.get_mut(handle) {
                rb.mprops.effective_inv_mass = Vector::repeat(1.0);
            }
        }

        let direction = Vector::x_axis().into_inner();
        let mut constraints = [
            DistanceConstraint::new(h1, h2, 1.0, direction),
            DistanceConstraint::new(h2, h3, 1.0, direction),
        ];

        let mut solver = AvbdSolver::new(AvbdSolverParams {
            iterations: 12,
            ..Default::default()
        });

        solver.solve(&mut bodies, &mut constraints, 0.01);

        let p1 = bodies[h1].translation().x;
        let p2 = bodies[h2].translation().x;
        let p3 = bodies[h3].translation().x;
        let d12 = (p2 - p1).abs();
        let d23 = (p3 - p2).abs();

        assert!(
            (d12 - 1.0).abs() < 0.1 && (d23 - 1.0).abs() < 0.1,
            "chain distances did not converge sufficiently: d12={d12}, d23={d23}"
        );
    }
}
