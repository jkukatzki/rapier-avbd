use crate::dynamics::{RigidBodyHandle, RigidBodySet};
use crate::geometry::{ContactManifold, ContactManifoldIndex, SolverContact};
use crate::math::{ANG_DIM, AngVector, Isometry, Point, Real, SPATIAL_DIM, Vector};

use super::{AvbdBodySet, AvbdConstraint, AvbdConstraintState};

const LINEAR_DOF: usize = SPATIAL_DIM - ANG_DIM;

/// Constraint variant used by the AVBD solver.
pub enum AvbdAnyConstraint {
    /// Non-penetration contact constraint derived from Rapier contact manifolds.
    Contact(AvbdContactConstraint),
}

impl AvbdAnyConstraint {
    pub fn contact(&self) -> Option<&AvbdContactConstraint> {
        match self {
            AvbdAnyConstraint::Contact(c) => Some(c),
        }
    }

    #[allow(dead_code)]
    pub fn contact_mut(&mut self) -> Option<&mut AvbdContactConstraint> {
        match self {
            AvbdAnyConstraint::Contact(c) => Some(c),
        }
    }
}

impl AvbdConstraint for AvbdAnyConstraint {
    fn bodies(&self) -> &[RigidBodyHandle] {
        match self {
            AvbdAnyConstraint::Contact(c) => c.bodies(),
        }
    }

    fn state_mut(&mut self) -> &mut AvbdConstraintState {
        match self {
            AvbdAnyConstraint::Contact(c) => c.state_mut(),
        }
    }

    fn state(&self) -> &AvbdConstraintState {
        match self {
            AvbdAnyConstraint::Contact(c) => c.state(),
        }
    }

    fn evaluate(&self, bodies: &RigidBodySet, workspace: &AvbdBodySet) -> Real {
        match self {
            AvbdAnyConstraint::Contact(c) => c.evaluate(bodies, workspace),
        }
    }

    fn gradient(
        &self,
        bodies: &RigidBodySet,
        workspace: &AvbdBodySet,
        body: RigidBodyHandle,
        out: &mut [Real; SPATIAL_DIM],
    ) {
        match self {
            AvbdAnyConstraint::Contact(c) => c.gradient(bodies, workspace, body, out),
        }
    }

    fn hessian(
        &self,
        bodies: &RigidBodySet,
        workspace: &AvbdBodySet,
        body: RigidBodyHandle,
        out: &mut [[Real; SPATIAL_DIM]; SPATIAL_DIM],
    ) {
        match self {
            AvbdAnyConstraint::Contact(c) => c.hessian(bodies, workspace, body, out),
        }
    }

    fn project_lambda(&self, lambda: Real) -> Real {
        match self {
            AvbdAnyConstraint::Contact(c) => c.project_lambda(lambda),
        }
    }

    fn relative_normal_velocity(
        &self,
        bodies: &RigidBodySet,
        workspace: &AvbdBodySet,
    ) -> Option<Real> {
        match self {
            AvbdAnyConstraint::Contact(c) => c.relative_normal_velocity(bodies, workspace),
        }
    }
}

/// AVBD contact constraint derived from a Rapier contact manifold point.
pub struct AvbdContactConstraint {
    bodies: [RigidBodyHandle; 2],
    body_count: usize,
    state: AvbdConstraintState,
    normal: Vector<Real>,
    base_distance: Real,
    local_points: [Point<Real>; 2],
    is_new_contact: bool,
    contact_point_id: usize,
    manifold_index: ContactManifoldIndex,
    solver_contact_index: usize,
}

impl AvbdContactConstraint {
    pub fn new(
        manifold_index: ContactManifoldIndex,
        solver_contact_index: usize,
        manifold: &ContactManifold,
        contact: &SolverContact,
        bodies: &RigidBodySet,
        stiffness: Real,
    ) -> Option<Self> {
        let handle1 = manifold.data.rigid_body1?;
        let handle2 = manifold.data.rigid_body2?;

        let rb1 = bodies.get(handle1)?;
        let rb2 = bodies.get(handle2)?;

        let world_point: Point<Real> = contact.point;
        let local_p1 = rb1.pos.position.inverse_transform_point(&world_point);
        let local_p2 = rb2.pos.position.inverse_transform_point(&world_point);

        let bodies_array = [handle1, handle2];
        let mut state = AvbdConstraintState::new(contact.warmstart_impulse, stiffness);
        let warmstart_negative = state.lambda < 0.0;
        if warmstart_negative {
            state.lambda = 0.0;
        }
        let contact_point_id = contact.contact_id[0] as usize;

        Some(Self {
            bodies: bodies_array,
            body_count: 2,
            state,
            normal: manifold.data.normal,
            base_distance: contact.dist,
            local_points: [local_p1, local_p2],
            is_new_contact: !warmstart_negative && contact.is_new > 0.5,
            contact_point_id,
            manifold_index,
            solver_contact_index,
        })
    }

    pub fn manifold_index(&self) -> ContactManifoldIndex {
        self.manifold_index
    }

    pub fn solver_contact_index(&self) -> usize {
        self.solver_contact_index
    }

    pub fn contact_point_index(&self) -> usize {
        self.contact_point_id
    }

    fn body_slot(&self, handle: RigidBodyHandle) -> Option<usize> {
        self.bodies
            .iter()
            .copied()
            .take(self.body_count)
            .position(|h| h == handle)
    }

    fn fetch_pose(
        &self,
        handle: RigidBodyHandle,
        bodies: &RigidBodySet,
        workspace: &AvbdBodySet,
    ) -> Option<Isometry<Real>> {
        if let Some(state) = workspace.state(handle) {
            return Some(state.pose().clone());
        }

        bodies.get(handle).map(|rb| rb.pos.position.clone())
    }

    fn world_point(
        &self,
        slot: usize,
        bodies: &RigidBodySet,
        workspace: &AvbdBodySet,
    ) -> Option<Point<Real>> {
        let handle = self.bodies[slot];
        let pose = self.fetch_pose(handle, bodies, workspace)?;
        Some(pose.transform_point(&self.local_points[slot]))
    }

    fn linear_direction(&self, slot: usize) -> Vector<Real> {
        if slot == 0 { -self.normal } else { self.normal }
    }

    fn angular_direction(
        &self,
        slot: usize,
        bodies: &RigidBodySet,
        workspace: &AvbdBodySet,
    ) -> Option<AngVector<Real>> {
        let world_point = self.world_point(slot, bodies, workspace)?;
        let handle = self.bodies[slot];
        let pose = self.fetch_pose(handle, bodies, workspace)?;
        let offset = world_point.coords - pose.translation.vector;

        #[cfg(feature = "dim2")]
        {
            let perp = offset.x * self.normal.y - offset.y * self.normal.x;
            Some(if slot == 0 { -perp } else { perp })
        }

        #[cfg(feature = "dim3")]
        {
            let cross = offset.cross(&self.normal);
            Some(if slot == 0 { -cross } else { cross })
        }
    }
}

impl AvbdConstraint for AvbdContactConstraint {
    fn bodies(&self) -> &[RigidBodyHandle] {
        &self.bodies[..self.body_count]
    }

    fn state_mut(&mut self) -> &mut AvbdConstraintState {
        &mut self.state
    }

    fn state(&self) -> &AvbdConstraintState {
        &self.state
    }

    fn evaluate(&self, bodies: &RigidBodySet, workspace: &AvbdBodySet) -> Real {
        let p1 = self
            .world_point(0, bodies, workspace)
            .map(|p| p.coords)
            .unwrap_or(Vector::zeros());

        let p2 = if self.body_count > 1 {
            self.world_point(1, bodies, workspace)
                .map(|p| p.coords)
                .unwrap_or(Vector::zeros())
        } else {
            p1
        };

        (p2 - p1).dot(&self.normal) + self.base_distance
    }

    fn gradient(
        &self,
        bodies: &RigidBodySet,
        workspace: &AvbdBodySet,
        body: RigidBodyHandle,
        out: &mut [Real; SPATIAL_DIM],
    ) {
        out.fill(0.0);
        let Some(slot) = self.body_slot(body) else {
            return;
        };

        let linear = self.linear_direction(slot);
        for i in 0..LINEAR_DOF {
            out[i] = linear[i];
        }

        if let Some(angular) = self.angular_direction(slot, bodies, workspace) {
            #[cfg(feature = "dim2")]
            {
                out[LINEAR_DOF] = angular;
            }

            #[cfg(feature = "dim3")]
            {
                for i in 0..ANG_DIM {
                    out[LINEAR_DOF + i] = angular[i];
                }
            }
        }
    }

    fn hessian(
        &self,
        _bodies: &RigidBodySet,
        _workspace: &AvbdBodySet,
        body: RigidBodyHandle,
        out: &mut [[Real; SPATIAL_DIM]; SPATIAL_DIM],
    ) {
        if self.body_slot(body).is_none() {
            return;
        }

        for row in out.iter_mut() {
            for entry in row.iter_mut() {
                *entry = 0.0;
            }
        }
    }

    fn project_lambda(&self, lambda: Real) -> Real {
        lambda.max(0.0)
    }

    fn relative_normal_velocity(
        &self,
        bodies: &RigidBodySet,
        workspace: &AvbdBodySet,
    ) -> Option<Real> {
        if !self.is_new_contact {
            return None;
        }

        let linvel1 = workspace
            .state(self.bodies[0])
            .map(|s| s.predicted_linvel().clone())
            .or_else(|| bodies.get(self.bodies[0]).map(|rb| rb.linvel().clone()))?;

        let linvel2 = if self.body_count > 1 {
            workspace
                .state(self.bodies[1])
                .map(|s| s.predicted_linvel().clone())
                .or_else(|| bodies.get(self.bodies[1]).map(|rb| rb.linvel().clone()))
                .unwrap_or(Vector::zeros())
        } else {
            Vector::zeros()
        };

        Some((linvel2 - linvel1).dot(&self.normal))
    }
}
