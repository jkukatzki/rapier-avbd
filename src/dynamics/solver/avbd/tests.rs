#![cfg(all(test, feature = "solver_avbd", feature = "dim3"))]

use approx::assert_relative_eq;

use crate::dynamics::{
    CCDSolver, ImpulseJointSet, IntegrationParameters, IslandManager, MultibodyJointSet,
    RigidBodyBuilder, RigidBodyHandle, RigidBodySet, SolverBackend,
};
use crate::geometry::{BroadPhaseBvh, ColliderBuilder, ColliderHandle, ColliderSet, NarrowPhase};
use crate::math::{AngVector, Real, Vector};
use crate::pipeline::PhysicsPipeline;

#[cfg(feature = "dim3")]
use crate::geometry::ContactManifoldExt;

#[cfg(feature = "dim3")]
#[test]
fn avbd_cube_stays_above_ground() {
    let outcome = run_drop_test(120, SolverBackend::Avbd);
    assert!(outcome.translation.y >= -1.0e-4, "cube penetrated plane");

    let mass = outcome.mass;
    let gravity_mag = (-outcome.gravity.y).abs();
    let expected_impulse = mass * gravity_mag * outcome.dt;
    assert_relative_eq!(
        outcome.normal_impulse,
        expected_impulse,
        epsilon = 5.0e-2,
        max_relative = 5.0e-2
    );
}

#[cfg(feature = "dim3")]
#[test]
fn avbd_drop_is_deterministic() {
    let outcome_a = run_drop_test(90, SolverBackend::Avbd);
    let outcome_b = run_drop_test(90, SolverBackend::Avbd);

    assert_relative_eq!(
        outcome_a.translation,
        outcome_b.translation,
        epsilon = 1.0e-6,
        max_relative = 1.0e-6,
    );

    assert_relative_eq!(
        outcome_a.angvel,
        outcome_b.angvel,
        epsilon = 1.0e-6,
        max_relative = 1.0e-6,
    );

    assert_relative_eq!(
        outcome_a.normal_impulse,
        outcome_b.normal_impulse,
        epsilon = 1.0e-6,
        max_relative = 1.0e-6,
    );
}

#[cfg(feature = "dim3")]
#[test]
fn avbd_matches_impulse_baseline() {
    let impulse_outcome = run_drop_test(120, SolverBackend::Impulse);
    let avbd_outcome = run_drop_test(120, SolverBackend::Avbd);

    assert_relative_eq!(
        avbd_outcome.translation,
        impulse_outcome.translation,
        epsilon = 5.0e-3,
        max_relative = 5.0e-3,
    );

    assert_relative_eq!(
        avbd_outcome.angvel,
        impulse_outcome.angvel,
        epsilon = 5.0e-3,
        max_relative = 5.0e-3,
    );

    assert_relative_eq!(
        avbd_outcome.normal_impulse,
        impulse_outcome.normal_impulse,
        epsilon = 5.0e-2,
        max_relative = 5.0e-2,
    );
}

#[cfg(feature = "dim3")]
struct DropOutcome {
    translation: Vector<Real>,
    angvel: AngVector<Real>,
    normal_impulse: Real,
    mass: Real,
    gravity: Vector<Real>,
    dt: Real,
}

#[cfg(feature = "dim3")]
fn run_drop_test(steps: usize, backend: SolverBackend) -> DropOutcome {
    let mut pipeline = PhysicsPipeline::new();
    let gravity = Vector::new(0.0, -9.81, 0.0);
    let mut integration_parameters = IntegrationParameters::default();
    integration_parameters.dt = 1.0 / 60.0;
    integration_parameters.solver_backend = backend;

    let mut islands = IslandManager::new();
    let mut broad_phase = BroadPhaseBvh::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();
    let mut ccd_solver = CCDSolver::new();

    let (_ground_body, ground_collider) = create_ground(&mut bodies, &mut colliders);
    let (cube_body, cube_collider) = create_cube(&mut bodies, &mut colliders);

    for _ in 0..steps {
        pipeline.step(
            &gravity,
            &integration_parameters,
            &mut islands,
            &mut broad_phase,
            &mut narrow_phase,
            &mut bodies,
            &mut colliders,
            &mut impulse_joints,
            &mut multibody_joints,
            &mut ccd_solver,
            &(),
            &(),
        );
    }

    let cube_rb = &bodies[cube_body];
    let translation = *cube_rb.translation();
    let angvel = *cube_rb.angvel();
    let mass = cube_rb.mass();
    let normal_impulse = narrow_phase
        .contact_pair(ground_collider, cube_collider)
        .map(|pair| {
            pair.manifolds
                .iter()
                .map(ContactManifoldExt::total_impulse)
                .sum()
        })
        .unwrap_or(0.0);

    DropOutcome {
        translation,
        angvel,
        normal_impulse,
        mass,
        gravity,
        dt: integration_parameters.dt,
    }
}

#[cfg(feature = "dim3")]
fn create_ground(
    bodies: &mut RigidBodySet,
    colliders: &mut ColliderSet,
) -> (RigidBodyHandle, ColliderHandle) {
    let ground_body = bodies.insert(RigidBodyBuilder::fixed().build());
    let collider = ColliderBuilder::cuboid(20.0, 0.5, 20.0).build();
    let ground_collider = colliders.insert_with_parent(collider, ground_body, bodies);
    (ground_body, ground_collider)
}

#[cfg(feature = "dim3")]
fn create_cube(
    bodies: &mut RigidBodySet,
    colliders: &mut ColliderSet,
) -> (RigidBodyHandle, ColliderHandle) {
    let rigid_body = bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.0, 3.0, 0.0))
            .build(),
    );
    let collider = ColliderBuilder::cuboid(0.5, 0.5, 0.5).build();
    let collider_handle = colliders.insert_with_parent(collider, rigid_body, bodies);
    (rigid_body, collider_handle)
}
