#![cfg(feature = "solver_avbd")]

use rapier2d::prelude::*;
use std::time::{Duration, Instant};

struct BenchResult {
    total_time: Duration,
    final_height: Real,
}

fn run_solver(backend: SolverBackend) -> BenchResult {
    let gravity = Vector::new(0.0, -9.81);
    let mut pipeline = PhysicsPipeline::new();
    let mut integration_params = IntegrationParameters::default();

    integration_params.solver_backend = backend;
    integration_params.num_solver_iterations = 6;
    integration_params.warmstart_coefficient = 1.0;

    let mut islands = IslandManager::new();
    let mut broad_phase = DefaultBroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();
    let mut ccd_solver = CCDSolver::new();

    let ground_body = RigidBodyBuilder::fixed()
        .translation(Vector::new(0.0, -1.0))
        .build();
    let ground_handle = bodies.insert(ground_body);
    let ground_collider = ColliderBuilder::cuboid(5.0, 1.0).build();
    colliders.insert_with_parent(ground_collider, ground_handle, &mut bodies);

    let dynamic_body = RigidBodyBuilder::dynamic()
        .translation(Vector::new(0.0, 5.0))
        .build();
    let dynamic_handle = bodies.insert(dynamic_body);
    let dynamic_collider = ColliderBuilder::cuboid(0.5, 0.5).build();
    colliders.insert_with_parent(dynamic_collider, dynamic_handle, &mut bodies);

    let mut total_time = Duration::default();

    for _ in 0..120 {
        let start = Instant::now();

        pipeline.step(
            &gravity,
            &integration_params,
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

        total_time += start.elapsed();
    }

    let final_height = bodies
        .get(dynamic_handle)
        .map(|body| body.translation().y)
        .unwrap_or_default();

    BenchResult {
        total_time,
        final_height,
    }
}

#[test]
fn avbd_and_impulse_step_with_similar_outcome_2d() {
    let impulse = run_solver(SolverBackend::Impulse);
    let avbd = run_solver(SolverBackend::Avbd);

    println!(
        "[bench-2d] impulse: {:?}, avbd: {:?}",
        impulse.total_time, avbd.total_time
    );
    println!(
        "[bench-2d] final heights -> impulse: {:.4}, avbd: {:.4}",
        impulse.final_height, avbd.final_height
    );

    let height_delta = (impulse.final_height - avbd.final_height).abs();
    assert!(
        height_delta < 0.25,
        "expected final heights to be close, got delta {height_delta}"
    );
}
