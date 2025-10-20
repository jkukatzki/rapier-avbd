use std::collections::HashMap;

use crate::dynamics::RigidBodyHandle;
use crate::math::{AngularInertia, Isometry, Real, Vector};

pub(crate) struct WorkspaceBody {
    pub handle: RigidBodyHandle,
    pub inv_mass: Vector<Real>,
    pub inv_inertia: AngularInertia<Real>,
    pub initial_pose: Isometry<Real>,
    pub pose: Isometry<Real>,
    pub initial_linvel: Vector<Real>,
    pub predicted_linvel: Vector<Real>,
}

pub struct AvbdBodyState<'a> {
    entry: &'a WorkspaceBody,
}

impl<'a> AvbdBodyState<'a> {
    pub fn handle(&self) -> RigidBodyHandle {
        self.entry.handle
    }

    pub fn pose(&self) -> &Isometry<Real> {
        &self.entry.pose
    }

    pub fn initial_pose(&self) -> &Isometry<Real> {
        &self.entry.initial_pose
    }

    pub fn initial_linvel(&self) -> &Vector<Real> {
        &self.entry.initial_linvel
    }

    pub fn predicted_linvel(&self) -> &Vector<Real> {
        &self.entry.predicted_linvel
    }

    pub fn inv_mass(&self) -> &Vector<Real> {
        &self.entry.inv_mass
    }

    pub fn inv_inertia(&self) -> &AngularInertia<Real> {
        &self.entry.inv_inertia
    }
}

pub struct AvbdBodySet<'a> {
    entries: &'a [WorkspaceBody],
    map: &'a HashMap<RigidBodyHandle, usize>,
}

impl<'a> AvbdBodySet<'a> {
    pub(crate) fn new(
        entries: &'a [WorkspaceBody],
        map: &'a HashMap<RigidBodyHandle, usize>,
    ) -> Self {
        Self { entries, map }
    }

    pub fn state(&self, handle: RigidBodyHandle) -> Option<AvbdBodyState<'a>> {
        self.map.get(&handle).copied().map(|index| AvbdBodyState {
            entry: &self.entries[index],
        })
    }

    pub fn len(&self) -> usize {
        self.entries.len()
    }
}

pub(crate) fn new_workspace_body(
    handle: RigidBodyHandle,
    inv_mass: Vector<Real>,
    inv_inertia: AngularInertia<Real>,
    initial_pose: Isometry<Real>,
    pose: Isometry<Real>,
    initial_linvel: Vector<Real>,
    predicted_linvel: Vector<Real>,
) -> WorkspaceBody {
    WorkspaceBody {
        handle,
        inv_mass,
        inv_inertia,
        initial_pose,
        pose,
        initial_linvel,
        predicted_linvel,
    }
}
