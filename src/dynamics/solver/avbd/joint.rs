use crate::dynamics::{JointGraphEdge, JointIndex, RigidBodyHandle, RigidBodySet};
use crate::math::{ANG_DIM, AngVector, DIM, Isometry, Point, Real, SPATIAL_DIM, Vector};

use super::{AvbdBodySet, AvbdConstraint, AvbdConstraintState};

const LINEAR_DOF: usize = SPATIAL_DIM - ANG_DIM;

#[derive(Clone)]
pub struct AvbdJointConstraint {
    bodies: [RigidBodyHandle; 2],
    body_count: usize,
    state: AvbdConstraintState,
    local_frames: [Isometry<Real>; 2],
    axis: AvbdJointAxis,
    joint_index: JointIndex,
    impulse_component: usize,
}

#[derive(Clone, Copy)]
enum AvbdJointAxis {
    Linear(usize),
    Angular(usize),
}

impl AvbdJointConstraint {
    pub fn new_linear(
        joint_index: JointIndex,
        handles: [RigidBodyHandle; 2],
        local_frames: [Isometry<Real>; 2],
        axis: usize,
        warmstart: Real,
        stiffness: Real,
    ) -> Option<Self> {
        if axis >= DIM {
            return None;
        }

        Some(Self {
            bodies: handles,
            body_count: handles.len(),
            state: AvbdConstraintState::new(warmstart, stiffness),
            local_frames,
            axis: AvbdJointAxis::Linear(axis),
            joint_index,
            impulse_component: axis,
        })
    }

    pub fn new_angular(
        joint_index: JointIndex,
        handles: [RigidBodyHandle; 2],
        local_frames: [Isometry<Real>; 2],
        axis: usize,
        warmstart: Real,
        stiffness: Real,
    ) -> Option<Self> {
        if axis >= ANG_DIM {
            return None;
        }

        Some(Self {
            bodies: handles,
            body_count: handles.len(),
            state: AvbdConstraintState::new(warmstart, stiffness),
            local_frames,
            axis: AvbdJointAxis::Angular(axis),
            joint_index,
            impulse_component: DIM + axis,
        })
    }

    fn body_slot(&self, handle: RigidBodyHandle) -> Option<usize> {
        self.bodies
            .iter()
            .copied()
            .take(self.body_count)
            .position(|h| h == handle)
    }

    fn body_pose(
        &self,
        slot: usize,
        bodies: &RigidBodySet,
        workspace: &AvbdBodySet,
    ) -> Option<Isometry<Real>> {
        let handle = self.bodies[slot];
        if let Some(state) = workspace.state(handle) {
            Some(state.pose().clone())
        } else {
            bodies.get(handle).map(|rb| rb.pos.position)
        }
    }

    fn world_frame(
        &self,
        slot: usize,
        bodies: &RigidBodySet,
        workspace: &AvbdBodySet,
    ) -> Option<Isometry<Real>> {
        let base_pose = self.body_pose(slot, bodies, workspace)?;
        Some(base_pose * self.local_frames[slot])
    }

    fn anchor_point(
        &self,
        slot: usize,
        bodies: &RigidBodySet,
        workspace: &AvbdBodySet,
    ) -> Option<Point<Real>> {
        let frame = self.world_frame(slot, bodies, workspace)?;
        Some(Point::from(frame.translation.vector))
    }

    fn linear_axis_world(
        &self,
        bodies: &RigidBodySet,
        workspace: &AvbdBodySet,
        axis: usize,
    ) -> Option<Vector<Real>> {
        let frame1 = self.world_frame(0, bodies, workspace)?;
        let local_axis = unit_linear(axis);
        Some(frame1.rotation * local_axis)
    }

    #[cfg(feature = "dim2")]
    fn angular_axis_world(
        &self,
        bodies: &RigidBodySet,
        workspace: &AvbdBodySet,
        _axis: usize,
    ) -> Option<AngVector<Real>> {
        let _ = (bodies, workspace);
        Some(1.0)
    }

    #[cfg(feature = "dim3")]
    fn angular_axis_world(
        &self,
        bodies: &RigidBodySet,
        workspace: &AvbdBodySet,
        axis: usize,
    ) -> Option<AngVector<Real>> {
        let frame1 = self.world_frame(0, bodies, workspace)?;
        let mut axis_vec = AngVector::zeros();
        axis_vec[axis] = 1.0;
        Some(frame1.rotation * axis_vec)
    }

    fn angular_direction_linear(
        &self,
        slot: usize,
        axis_world: &Vector<Real>,
        bodies: &RigidBodySet,
        workspace: &AvbdBodySet,
    ) -> Option<AngVector<Real>> {
        let world_point = self.anchor_point(slot, bodies, workspace)?;
        let body_pose = self.body_pose(slot, bodies, workspace)?;
        let offset = world_point.coords - body_pose.translation.vector;

        #[cfg(feature = "dim2")]
        {
            let perp = offset.x * axis_world.y - offset.y * axis_world.x;
            return Some(if slot == 0 { -perp } else { perp });
        }

        #[cfg(feature = "dim3")]
        {
            let cross = offset.cross(axis_world);
            return Some(if slot == 0 { -cross } else { cross });
        }
    }

    fn angular_direction_angular(
        &self,
        slot: usize,
        axis_world: &AngVector<Real>,
    ) -> AngVector<Real> {
        #[cfg(feature = "dim2")]
        {
            if slot == 0 { -*axis_world } else { *axis_world }
        }

        #[cfg(feature = "dim3")]
        {
            if slot == 0 { -*axis_world } else { *axis_world }
        }
    }

    pub fn writeback(&self, joints: &mut [JointGraphEdge]) {
        if let Some(joint) = joints.get_mut(self.joint_index) {
            joint.weight.impulses[self.impulse_component] = self.state.lambda;
        }
    }
}

impl AvbdConstraint for AvbdJointConstraint {
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
        match self.axis {
            AvbdJointAxis::Linear(axis) => {
                let frame1 = match self.world_frame(0, bodies, workspace) {
                    Some(f) => f,
                    None => return 0.0,
                };
                let frame2 = match self.world_frame(1, bodies, workspace) {
                    Some(f) => f,
                    None => return 0.0,
                };
                let axis_world = match self.linear_axis_world(bodies, workspace, axis) {
                    Some(a) => a,
                    None => return 0.0,
                };
                let diff = frame2.translation.vector - frame1.translation.vector;
                diff.dot(&axis_world)
            }
            AvbdJointAxis::Angular(axis_idx) => {
                let _ = axis_idx;
                let frame1 = match self.world_frame(0, bodies, workspace) {
                    Some(f) => f,
                    None => return 0.0,
                };
                let frame2 = match self.world_frame(1, bodies, workspace) {
                    Some(f) => f,
                    None => return 0.0,
                };

                #[cfg(feature = "dim2")]
                {
                    let angle1 = frame1.rotation.angle();
                    let angle2 = frame2.rotation.angle();
                    return angle2 - angle1;
                }

                #[cfg(feature = "dim3")]
                {
                    let axis_world = match self.angular_axis_world(bodies, workspace, axis_idx) {
                        Some(a) => a,
                        None => return 0.0,
                    };
                    let relative = frame1.rotation.inverse() * frame2.rotation;
                    let phi = relative.scaled_axis();
                    return phi.dot(&axis_world);
                }
            }
        }
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

        match self.axis {
            AvbdJointAxis::Linear(axis) => {
                let axis_world = match self.linear_axis_world(bodies, workspace, axis) {
                    Some(a) => a,
                    None => return,
                };
                let sign = if slot == 0 { -1.0 } else { 1.0 };
                for i in 0..LINEAR_DOF {
                    out[i] = sign * axis_world[i];
                }

                if let Some(angular) =
                    self.angular_direction_linear(slot, &axis_world, bodies, workspace)
                {
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
            AvbdJointAxis::Angular(axis_idx) => {
                let _ = axis_idx;
                #[cfg(feature = "dim2")]
                let axis_world = {
                    let _axis = axis_idx;
                    match self.angular_axis_world(bodies, workspace, 0) {
                        Some(v) => v,
                        None => return,
                    }
                };

                #[cfg(feature = "dim3")]
                let axis_world = match self.angular_axis_world(bodies, workspace, axis_idx) {
                    Some(v) => v,
                    None => return,
                };

                let angular = self.angular_direction_angular(slot, &axis_world);

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
}

fn unit_linear(axis: usize) -> Vector<Real> {
    let mut basis = Vector::zeros();
    basis[axis] = 1.0;
    basis
}

pub fn build_joint_constraints(
    constraints: &mut Vec<super::AvbdAnyConstraint>,
    joints: &[JointGraphEdge],
    joint_indices: &[JointIndex],
    stiffness: Real,
) {
    for &joint_index in joint_indices {
        let joint_edge = &joints[joint_index];
        let joint = &joint_edge.weight;

        if !joint.data.is_enabled() {
            continue;
        }

        let handles = [joint.body1, joint.body2];
        let local_frames = [joint.data.local_frame1, joint.data.local_frame2];
        let locked = joint.data.locked_axes.bits();

        for axis in 0..DIM {
            if locked & (1 << axis) == 0 {
                continue;
            }

            let warmstart = joint.impulses[axis];
            if let Some(constraint) = AvbdJointConstraint::new_linear(
                joint_index,
                handles,
                local_frames,
                axis,
                warmstart,
                stiffness,
            ) {
                constraints.push(super::AvbdAnyConstraint::Joint(Box::new(constraint)));
            }
        }

        for axis in 0..ANG_DIM {
            if locked & (1 << (DIM + axis)) == 0 {
                continue;
            }

            let warmstart = joint.impulses[DIM + axis];
            if let Some(constraint) = AvbdJointConstraint::new_angular(
                joint_index,
                handles,
                local_frames,
                axis,
                warmstart,
                stiffness,
            ) {
                constraints.push(super::AvbdAnyConstraint::Joint(Box::new(constraint)));
            }
        }
    }
}
