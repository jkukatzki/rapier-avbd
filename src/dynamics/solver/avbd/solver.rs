use std::collections::HashMap;
use std::mem;

use crate::dynamics::{RigidBodyHandle, RigidBodySet, RigidBodyType};
use crate::math::{ANG_DIM, AngVector, Real, SPATIAL_DIM, Vector};
use crate::na;
#[cfg(feature = "dim3")]
use crate::utils::SimdAngularInertia;
use rustc_hash::FxHashSet;

use super::workspace::{WorkspaceBody, new_workspace_body};
use super::{AVBD_DOF, AvbdBodySet, AvbdConstraint, AvbdSolverParams};

const LINEAR_DOF: usize = SPATIAL_DIM - ANG_DIM;

/// Augmented Vertex Block Descent solver implementation.
pub struct AvbdSolver {
    params: AvbdSolverParams,
    workspace: SolverWorkspace,
    color_buckets: Vec<Vec<usize>>,
    bucket_body_sets: Vec<FxHashSet<RigidBodyHandle>>,
    constraint_cache: Vec<ConstraintCache>,
    dt: Real,
}

impl AvbdSolver {
    /// Creates a new solver with the provided parameters.
    pub fn new(params: AvbdSolverParams) -> Self {
        Self {
            params,
            workspace: SolverWorkspace::default(),
            color_buckets: Vec::new(),
            bucket_body_sets: Vec::new(),
            constraint_cache: Vec::new(),
            dt: 1.0,
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

    /// Returns the timestep used during the last `solve` call.
    pub fn timestep(&self) -> Real {
        self.dt
    }

    /// Executes an AVBD solve pass for the supplied rigid bodies and constraints.
    pub fn solve<C>(&mut self, bodies: &mut RigidBodySet, constraints: &mut [C], dt: Real)
    where
        C: AvbdConstraint,
    {
        if constraints.is_empty() || bodies.len() == 0 || dt <= 0.0 {
            return;
        }

        self.dt = dt;
        self.workspace.initialize(bodies, constraints, dt);
        let warm_lambda = self.params.alpha * self.params.gamma;
        let stiffness_decay = self.params.gamma;

        for constraint in constraints.iter_mut() {
            let state = constraint.state_mut();
            state.lambda *= warm_lambda;
            state.stiffness = clamp_stiffness(state.stiffness * stiffness_decay, &self.params);
        }

        self.build_color_buckets(constraints);
        self.prepare_constraint_cache(bodies, constraints);

        let buckets = mem::take(&mut self.color_buckets);

        for _ in 0..self.params.iterations {
            for bucket in buckets.iter() {
                for &constraint_index in bucket {
                    let mut cache = mem::take(&mut self.constraint_cache[constraint_index]);
                    {
                        let constraint = &mut constraints[constraint_index];
                        self.solve_constraint(bodies, constraint, &mut cache);
                    }
                    self.constraint_cache[constraint_index] = cache;
                }
            }
        }

        self.color_buckets = buckets;

        self.workspace.writeback(bodies, dt);
    }

    fn solve_constraint<C>(
        &mut self,
        bodies: &mut RigidBodySet,
        constraint: &mut C,
        cache: &mut ConstraintCache,
    ) where
        C: AvbdConstraint,
    {
        let handles = constraint.bodies();
        if handles.is_empty() {
            return;
        }

        if cache.bodies.is_empty() {
            return;
        }

        let bodies_view = &*bodies;
        let (delta_lambda, projected_lambda, new_stiffness) = {
            let body_view = self.workspace.body_set();

            let mut cached_eval = None;
            if let Some(rel_vel) = constraint.relative_normal_velocity(bodies_view, &body_view) {
                const STATIC_RELVEL_EPSILON: Real = 1.0e-6;
                let state = constraint.state();
                let c_val = constraint.evaluate(bodies_view, &body_view);
                cached_eval = Some(c_val);
                if rel_vel.abs() <= STATIC_RELVEL_EPSILON
                    && c_val >= -STATIC_RELVEL_EPSILON
                    && state.lambda.abs() <= STATIC_RELVEL_EPSILON
                    && (state.stiffness - self.params.stiffness_min).abs() <= Real::EPSILON
                {
                    return;
                }
            }

            let compliance = {
                let state = constraint.state();
                if state.stiffness <= 0.0 {
                    0.0
                } else {
                    1.0 / (state.stiffness * self.dt * self.dt)
                }
            };

            let mut denominator = compliance;
            for body_cache in cache.bodies.iter() {
                denominator += body_cache.grad_minv_dot
                    + diag_projection(&body_cache.gradient, &body_cache.spd_diagonal);
            }

            if denominator <= 0.0 {
                return;
            }

            let c_val = cached_eval.unwrap_or_else(|| constraint.evaluate(bodies_view, &body_view));
            let state = constraint.state();
            let alpha_tilde = compliance;
            let raw_delta_lambda = -(c_val + alpha_tilde * state.lambda) / denominator;
            let raw_lambda = state.lambda + raw_delta_lambda;
            let projected_lambda = constraint.project_lambda(raw_lambda);
            let delta_lambda = projected_lambda - state.lambda;
            let updated_stiffness =
                clamp_stiffness(state.stiffness * self.params.beta, &self.params);
            (delta_lambda, projected_lambda, updated_stiffness)
        };

        for body_cache in cache.bodies.iter() {
            self.workspace.apply_body_delta(
                body_cache.entry_index,
                delta_lambda,
                &body_cache.minv_grad,
            );
        }

        let state = constraint.state_mut();
        let previous_stiffness = state.stiffness;
        state.lambda = projected_lambda;
        state.stiffness = new_stiffness;

        if previous_stiffness > 0.0 {
            let ratio = if new_stiffness <= 0.0 {
                0.0
            } else {
                new_stiffness / previous_stiffness
            };
            for body_cache in cache.bodies.iter_mut() {
                for diag_entry in body_cache.spd_diagonal.iter_mut() {
                    *diag_entry *= ratio;
                }
            }
        } else if new_stiffness > 0.0 {
            for body_cache in cache.bodies.iter_mut() {
                body_cache.spd_diagonal = unit_spd_diagonal(&body_cache.gradient);
                for diag_entry in body_cache.spd_diagonal.iter_mut() {
                    *diag_entry *= new_stiffness;
                }
            }
        } else {
            for body_cache in cache.bodies.iter_mut() {
                body_cache.spd_diagonal.fill(0.0);
            }
        }

        cache.stiffness_snapshot = state.stiffness;
    }

    fn build_color_buckets<C>(&mut self, constraints: &[C])
    where
        C: AvbdConstraint,
    {
        self.color_buckets.clear();
        self.bucket_body_sets.clear();

        for (index, constraint) in constraints.iter().enumerate() {
            let bodies = constraint.bodies();

            if bodies.is_empty() {
                if self.color_buckets.is_empty() {
                    self.color_buckets.push(vec![index]);
                    self.bucket_body_sets.push(FxHashSet::default());
                } else {
                    self.color_buckets[0].push(index);
                }
                continue;
            }

            let mut assigned = false;
            for (bucket_index, used_handles) in self.bucket_body_sets.iter_mut().enumerate() {
                if bodies.iter().all(|handle| !used_handles.contains(handle)) {
                    for handle in bodies {
                        used_handles.insert(*handle);
                    }
                    self.color_buckets[bucket_index].push(index);
                    assigned = true;
                    break;
                }
            }

            if !assigned {
                let mut handles = FxHashSet::default();
                for handle in bodies {
                    handles.insert(*handle);
                }
                self.bucket_body_sets.push(handles);
                self.color_buckets.push(vec![index]);
            }
        }
    }

    fn prepare_constraint_cache<C>(&mut self, bodies: &RigidBodySet, constraints: &[C])
    where
        C: AvbdConstraint,
    {
        if self.constraint_cache.len() > constraints.len() {
            self.constraint_cache.truncate(constraints.len());
        } else if self.constraint_cache.len() < constraints.len() {
            self.constraint_cache
                .resize_with(constraints.len(), ConstraintCache::default);
        }

        for cache in self.constraint_cache.iter_mut() {
            cache.bodies.clear();
            cache.stiffness_snapshot = 0.0;
        }

        let bodies_view = bodies;
        let workspace_view = self.workspace.body_set();

        for (index, constraint) in constraints.iter().enumerate() {
            let cache = &mut self.constraint_cache[index];
            cache.stiffness_snapshot = constraint.state().stiffness;

            for handle in constraint.bodies() {
                let Some(entry_index) = self.workspace.body_map.get(handle).copied() else {
                    continue;
                };

                let mut gradient = [0.0; AVBD_DOF];
                constraint.gradient(bodies_view, &workspace_view, *handle, &mut gradient);
                let minv_grad = self.workspace.mass_inverse(entry_index, &gradient);
                let grad_minv_dot = simd_dot(&gradient, &minv_grad);

                let mut hessian = [[0.0; AVBD_DOF]; AVBD_DOF];
                constraint.hessian(bodies_view, &workspace_view, *handle, &mut hessian);
                let mut spd_diagonal = compute_spd_diagonal(&hessian);

                if spd_diagonal
                    .iter()
                    .all(|entry| entry.abs() <= Real::EPSILON)
                {
                    spd_diagonal = unit_spd_diagonal(&gradient);
                    for diag in spd_diagonal.iter_mut() {
                        *diag *= cache.stiffness_snapshot.max(0.0);
                    }
                }

                cache.bodies.push(ConstraintBodyCache {
                    entry_index,
                    gradient,
                    minv_grad,
                    grad_minv_dot,
                    spd_diagonal,
                });
            }
        }
    }
}

#[derive(Clone)]
struct ConstraintBodyCache {
    entry_index: usize,
    gradient: [Real; AVBD_DOF],
    minv_grad: [Real; AVBD_DOF],
    grad_minv_dot: Real,
    spd_diagonal: [Real; AVBD_DOF],
}

#[derive(Default)]
struct ConstraintCache {
    bodies: Vec<ConstraintBodyCache>,
    stiffness_snapshot: Real,
}

fn diag_projection(gradient: &[Real; AVBD_DOF], diag: &[Real; AVBD_DOF]) -> Real {
    let mut acc = 0.0;
    for i in 0..AVBD_DOF {
        acc += diag[i] * gradient[i] * gradient[i];
    }
    acc
}

fn compute_spd_diagonal(matrix: &[[Real; AVBD_DOF]; AVBD_DOF]) -> [Real; AVBD_DOF] {
    let mut diag = [0.0; AVBD_DOF];
    for row in 0..AVBD_DOF {
        let mut sum = 0.0;
        for col in 0..AVBD_DOF {
            sum += matrix[row][col].abs();
        }
        diag[row] = sum;
    }
    diag
}

fn unit_spd_diagonal(gradient: &[Real; AVBD_DOF]) -> [Real; AVBD_DOF] {
    let mut diag = [0.0; AVBD_DOF];
    let total_abs: Real = gradient.iter().map(|g| g.abs()).sum();
    for i in 0..AVBD_DOF {
        diag[i] = gradient[i].abs() * total_abs;
    }
    diag
}

#[cfg(all(feature = "simd-is-enabled", not(feature = "f64")))]
fn simd_dot(lhs: &[Real; AVBD_DOF], rhs: &[Real; AVBD_DOF]) -> Real {
    use wide::f32x4;

    let mut acc = f32x4::from(0.0);
    let mut i = 0;
    while i + 4 <= AVBD_DOF {
        let a = f32x4::from_slice_unaligned(&lhs[i..]);
        let b = f32x4::from_slice_unaligned(&rhs[i..]);
        acc += a * b;
        i += 4;
    }

    let mut sum: Real = acc.to_array().into_iter().sum();
    while i < AVBD_DOF {
        sum += lhs[i] * rhs[i];
        i += 1;
    }
    sum
}

#[cfg(all(feature = "simd-is-enabled", feature = "f64"))]
fn simd_dot(lhs: &[Real; AVBD_DOF], rhs: &[Real; AVBD_DOF]) -> Real {
    use wide::f64x2;

    let mut acc = f64x2::from(0.0);
    let mut i = 0;
    while i + 2 <= AVBD_DOF {
        let a = f64x2::from_slice_unaligned(&lhs[i..]);
        let b = f64x2::from_slice_unaligned(&rhs[i..]);
        acc += a * b;
        i += 2;
    }

    let mut sum: Real = acc.to_array().into_iter().sum();
    while i < AVBD_DOF {
        sum += lhs[i] * rhs[i];
        i += 1;
    }
    sum
}

#[cfg(not(feature = "simd-is-enabled"))]
fn simd_dot(lhs: &[Real; AVBD_DOF], rhs: &[Real; AVBD_DOF]) -> Real {
    let mut sum = 0.0;
    for i in 0..AVBD_DOF {
        sum += lhs[i] * rhs[i];
    }
    sum
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
    bodies: Vec<WorkspaceBody>,
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

                let Some(rb) = bodies.get_mut(*handle) else {
                    continue;
                };

                if rb.body_type != RigidBodyType::Dynamic || !rb.is_enabled() {
                    continue;
                }

                let initial_pose = rb.pos.position;
                let initial_linvel = rb.vels.linvel;
                let predicted_vels = rb.forces.integrate(dt, &rb.vels, &rb.mprops);
                let predicted_pose = predicted_vels.integrate(
                    dt,
                    &rb.pos.position,
                    &rb.mprops.local_mprops.local_com,
                );

                rb.pos.next_position = predicted_pose;

                let entry = new_workspace_body(
                    *handle,
                    rb.mprops.effective_inv_mass,
                    rb.mprops.effective_world_inv_inertia,
                    initial_pose,
                    predicted_pose,
                    initial_linvel,
                    predicted_vels.linvel,
                );

                let index = self.bodies.len();
                self.body_map.insert(*handle, index);
                self.bodies.push(entry);
            }
        }
    }

    fn body_set(&self) -> AvbdBodySet<'_> {
        AvbdBodySet::new(&self.bodies, &self.body_map)
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
        delta_lambda: Real,
        minv_grad: &[Real; AVBD_DOF],
    ) {
        let entry = &mut self.bodies[entry_index];
        let mut scaled = [0.0; AVBD_DOF];
        for i in 0..AVBD_DOF {
            scaled[i] = minv_grad[i] * delta_lambda;
        }

        let (delta_lin, delta_ang) = split_gradient(&scaled);
        entry.pose.translation.vector += delta_lin;

        #[cfg(feature = "dim2")]
        {
            let rotation = na::Rotation2::new(delta_ang);
            entry.pose.rotation = rotation * entry.pose.rotation;
        }

        #[cfg(feature = "dim3")]
        {
            let rot = na::UnitQuaternion::from_scaled_axis(delta_ang);
            entry.pose.rotation = rot * entry.pose.rotation;
        }
    }

    fn writeback(&mut self, bodies: &mut RigidBodySet, dt: Real) {
        for entry in &self.bodies {
            if let Some(rb) = bodies.get_mut(entry.handle) {
                let final_pose = entry.pose;
                let lin_delta =
                    final_pose.translation.vector - entry.initial_pose.translation.vector;
                let linvel = lin_delta / dt;

                #[cfg(feature = "dim2")]
                let angvel = {
                    let final_angle = final_pose.rotation.angle();
                    let initial_angle = entry.initial_pose.rotation.angle();
                    (final_angle - initial_angle) / dt
                };

                #[cfg(feature = "dim3")]
                let angvel = {
                    let delta = final_pose.rotation * entry.initial_pose.rotation.inverse();
                    delta.scaled_axis() / dt
                };

                rb.pos.position = final_pose;
                rb.pos.next_position = final_pose;
                rb.vels.linvel = linvel;
                rb.vels.angvel = angvel;
                rb.mprops
                    .update_world_mass_properties(rb.body_type, &rb.pos.position);
                rb.activation.wake_up(true);
            }
        }
    }
}

#[cfg(feature = "dim2")]
fn split_gradient(vector: &[Real; AVBD_DOF]) -> (Vector<Real>, AngVector<Real>) {
    let mut lin = Vector::zeros();
    for i in 0..LINEAR_DOF {
        lin[i] = vector[i];
    }

    let ang = vector[LINEAR_DOF];

    (lin, ang)
}

#[cfg(feature = "dim3")]
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

#[cfg(feature = "dim2")]
fn merge_components(linear: &Vector<Real>, angular: &AngVector<Real>) -> [Real; AVBD_DOF] {
    let mut out = [0.0; AVBD_DOF];

    for i in 0..LINEAR_DOF {
        out[i] = linear[i];
    }

    out[LINEAR_DOF] = *angular;
    out
}

#[cfg(feature = "dim3")]
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

        fn evaluate(&self, _bodies: &RigidBodySet, workspace: &AvbdBodySet) -> Real {
            let p1 = workspace
                .state(self.bodies[0])
                .map(|state| state.pose().translation.vector)
                .expect("body missing from workspace");
            let p2 = workspace
                .state(self.bodies[1])
                .map(|state| state.pose().translation.vector)
                .expect("body missing from workspace");
            (self.direction.dot(&(p2 - p1))) - self.target
        }

        fn gradient(
            &self,
            _bodies: &RigidBodySet,
            _workspace: &AvbdBodySet,
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
            _workspace: &AvbdBodySet,
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
                .translation({
                    let mut t = Vector::zeros();
                    t[0] = 2.0;
                    t
                })
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

    struct GroundPlaneConstraint {
        body: RigidBodyHandle,
        state: AvbdConstraintState,
        plane_height: Real,
        radius: Real,
    }

    impl GroundPlaneConstraint {
        fn new(body: RigidBodyHandle, plane_height: Real, radius: Real) -> Self {
            Self {
                body,
                state: AvbdConstraintState::new(0.0, 5.0e3),
                plane_height,
                radius,
            }
        }
    }

    impl AvbdConstraint for GroundPlaneConstraint {
        fn bodies(&self) -> &[RigidBodyHandle] {
            std::slice::from_ref(&self.body)
        }

        fn state_mut(&mut self) -> &mut AvbdConstraintState {
            &mut self.state
        }

        fn state(&self) -> &AvbdConstraintState {
            &self.state
        }

        fn evaluate(&self, _bodies: &RigidBodySet, workspace: &AvbdBodySet) -> Real {
            let y = workspace
                .state(self.body)
                .map(|state| state.pose().translation.vector[1])
                .expect("body missing from workspace");
            y - (self.plane_height + self.radius)
        }

        fn gradient(
            &self,
            _bodies: &RigidBodySet,
            _workspace: &AvbdBodySet,
            body: RigidBodyHandle,
            out: &mut [Real; AVBD_DOF],
        ) {
            out.fill(0.0);
            if body == self.body {
                out[1] = 1.0;
            }
        }

        fn hessian(
            &self,
            _bodies: &RigidBodySet,
            _workspace: &AvbdBodySet,
            _body: RigidBodyHandle,
            out: &mut [[Real; AVBD_DOF]; AVBD_DOF],
        ) {
            for row in out.iter_mut() {
                row.fill(0.0);
            }
        }
    }

    fn drop_on_plane() -> (Real, Real) {
        let mut bodies = RigidBodySet::new();
        let mut translation = Vector::zeros();
        translation[1] = 2.0;
        let handle = bodies.insert(
            RigidBodyBuilder::dynamic()
                .translation(translation)
                .additional_mass(1.0)
                .build(),
        );

        if let Some(rb) = bodies.get_mut(handle) {
            rb.mprops.effective_inv_mass = Vector::repeat(1.0);
        }

        let mut constraint = GroundPlaneConstraint::new(handle, 0.0, 0.5);

        let mut solver = AvbdSolver::new(AvbdSolverParams {
            iterations: 12,
            ..Default::default()
        });

        solver.solve(&mut bodies, std::slice::from_mut(&mut constraint), 0.01);

        let y = bodies[handle].pos.position.translation.vector[1];
        let vy = bodies[handle].vels.linvel[1];
        (y, vy)
    }

    #[test]
    fn ground_plane_constraint_prevents_penetration() {
        let (y, _) = drop_on_plane();
        assert!(y >= 0.5 - 1.0e-3, "body dipped below plane: {y}");
    }

    #[test]
    fn solver_is_deterministic() {
        let first = drop_on_plane();
        let second = drop_on_plane();
        assert!((first.0 - second.0).abs() < 1.0e-9);
        assert!((first.1 - second.1).abs() < 1.0e-9);
    }

    fn run_distance_with_avbd(iterations: usize) -> (Real, Real) {
        let mut bodies = RigidBodySet::new();
        let h1 = bodies.insert(
            RigidBodyBuilder::dynamic()
                .translation(Vector::zeros())
                .additional_mass(1.0)
                .build(),
        );
        let h2 = bodies.insert(
            RigidBodyBuilder::dynamic()
                .translation({
                    let mut t = Vector::zeros();
                    t[0] = 2.0;
                    t
                })
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
            iterations,
            ..Default::default()
        });

        solver.solve(&mut bodies, std::slice::from_mut(&mut constraint), 0.01);

        let p1 = bodies[h1].pos.position.translation.vector.x;
        let p2 = bodies[h2].pos.position.translation.vector.x;
        (p1, p2)
    }

    fn run_distance_with_impulse(iterations: usize) -> (Real, Real) {
        let mut bodies = RigidBodySet::new();
        let h1 = bodies.insert(
            RigidBodyBuilder::dynamic()
                .translation(Vector::zeros())
                .additional_mass(1.0)
                .build(),
        );
        let h2 = bodies.insert(
            RigidBodyBuilder::dynamic()
                .translation({
                    let mut t = Vector::zeros();
                    t[0] = 2.0;
                    t
                })
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

        for _ in 0..iterations {
            let p1 = bodies[h1].pos.position.translation.vector;
            let p2 = bodies[h2].pos.position.translation.vector;
            let error = direction.dot(&(p2 - p1)) - 1.0;

            let inv_mass1 = bodies[h1].mprops.effective_inv_mass;
            let inv_mass2 = bodies[h2].mprops.effective_inv_mass;
            let denom = direction.component_mul(&inv_mass1).dot(&direction)
                + direction.component_mul(&inv_mass2).dot(&direction);

            if denom <= 0.0 {
                continue;
            }

            let delta_lambda = -error / denom;
            bodies[h1].pos.position.translation.vector -= direction * delta_lambda;
            bodies[h2].pos.position.translation.vector += direction * delta_lambda;
        }

        let p1 = bodies[h1].pos.position.translation.vector.x;
        let p2 = bodies[h2].pos.position.translation.vector.x;
        (p1, p2)
    }

    #[test]
    fn avbd_matches_simple_impulse_reference() {
        let (avbd_p1, avbd_p2) = run_distance_with_avbd(10);
        let (imp_p1, imp_p2) = run_distance_with_impulse(10);

        let avbd_gap = avbd_p2 - avbd_p1;
        let imp_gap = imp_p2 - imp_p1;
        assert!((avbd_gap - imp_gap).abs() < 5.0e-2);
    }
}
