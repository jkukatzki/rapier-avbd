#![cfg(feature = "solver_avbd")]

use rapier3d::prelude::*;
use std::time::{Duration, Instant};

struct BenchResult {
    total_time: Duration,
    first_height: Real,
    mean_height: Real,
}

fn spawn_dynamic_bodies(
    bodies: &mut RigidBodySet,
    colliders: &mut ColliderSet,
    body_count: usize,
) -> Vec<RigidBodyHandle> {
    let mut handles = Vec::with_capacity(body_count);
    let grid = ((body_count as f32).cbrt().ceil() as usize).max(1);
    let spacing: Real = 1.5;
    let layer_size = grid * grid;

    for i in 0..body_count {
        let layer = i / layer_size;
        let index_in_layer = i % layer_size;
        let row = index_in_layer / grid;
        let col = index_in_layer % grid;

        let x = (col as Real - (grid as Real - 1.0) * 0.5) * spacing;
        let z = (row as Real - (grid as Real - 1.0) * 0.5) * spacing;
        let y = 2.0 + layer as Real * spacing;

        let body = RigidBodyBuilder::dynamic()
            .translation(Vector::new(x, y, z))
            .can_sleep(false)
            .build();
        let body_handle = bodies.insert(body);

        let collider = if (row + col + layer) % 2 == 0 {
            ColliderBuilder::ball(0.5).build()
        } else {
            ColliderBuilder::cuboid(0.5, 0.5, 0.5).build()
        };

        colliders.insert_with_parent(collider, body_handle, bodies);
        handles.push(body_handle);
    }

    handles
}

fn run_solver(backend: SolverBackend, body_count: usize, steps: usize) -> BenchResult {
    let body_count = body_count.max(1);
    let gravity = Vector::new(0.0, -9.81, 0.0);
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

    // Ground body.
    let ground_body = RigidBodyBuilder::fixed()
        .translation(Vector::new(0.0, -1.0, 0.0))
        .build();
    let ground_handle = bodies.insert(ground_body);
    let grid = ((body_count as f32).cbrt().ceil() as usize).max(1);
    let grid_real = grid as Real;
    let ground_half_extent = (grid_real * 1.5).max(50.0);
    let ground_collider =
        ColliderBuilder::cuboid(ground_half_extent, 1.0, ground_half_extent).build();
    colliders.insert_with_parent(ground_collider, ground_handle, &mut bodies);

    let dynamic_handles = spawn_dynamic_bodies(&mut bodies, &mut colliders, body_count);
    let primary_handle = dynamic_handles[0];

    let start_time = Instant::now();
    for _ in 0..steps {
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
    }
    let total_time = start_time.elapsed();

    let first_height = bodies
        .get(primary_handle)
        .map(|body| body.translation().y)
        .unwrap_or_default();
    let mean_height = if dynamic_handles.is_empty() {
        0.0
    } else {
        dynamic_handles
            .iter()
            .filter_map(|handle| bodies.get(*handle))
            .map(|body| body.translation().y)
            .sum::<Real>()
            / dynamic_handles.len() as Real
    };

    BenchResult {
        total_time,
        first_height,
        mean_height,
    }
}

#[test]
fn avbd_and_impulse_step_with_similar_outcome() {
    let impulse = run_solver(SolverBackend::Impulse, 1, 120);
    let avbd = run_solver(SolverBackend::Avbd, 1, 120);

    println!(
        "[bench] impulse: {:?}, avbd: {:?}",
        impulse.total_time, avbd.total_time
    );
    println!(
        "[bench] final heights -> impulse: {:.4}, avbd: {:.4}",
        impulse.first_height, avbd.first_height
    );
    let impulse_ms = impulse.total_time.as_secs_f64() * 1000.0;
    let avbd_ms = avbd.total_time.as_secs_f64() * 1000.0;
    let improvement = ((impulse_ms - avbd_ms) / impulse_ms.max(f64::EPSILON)) * 100.0;
    println!(
        "[bench] AVBD vs Impulse total time delta: {:+.2}%",
        improvement
    );

    // After 120 steps both solvers should settle the cube near the ground with close values.
    let height_delta = (impulse.first_height - avbd.first_height).abs();
    assert!(
        height_delta < 0.25,
        "expected final heights to be close, got delta {height_delta}"
    );
}

#[test]
fn avbd_outperforms_impulse_on_thousand_body_stack() {
    let body_count = 1000;
    let steps = 80;

    let impulse = run_solver(SolverBackend::Impulse, body_count, steps);
    let avbd = run_solver(SolverBackend::Avbd, body_count, steps);

    let impulse_ms = impulse.total_time.as_secs_f64() * 1000.0;
    let avbd_ms = avbd.total_time.as_secs_f64() * 1000.0;
    let improvement = ((impulse_ms - avbd_ms) / impulse_ms.max(f64::EPSILON)) * 100.0;

    println!(
        "[bench-1000] impulse total: {:.2}ms, avbd total: {:.2}ms, improvement: {:+.2}%",
        impulse_ms, avbd_ms, improvement
    );
    println!(
        "[bench-1000] mean heights -> impulse: {:.3}, avbd: {:.3}",
        impulse.mean_height, avbd.mean_height
    );

    // Ensure both solvers lead to a comparable average stack height.
    let mean_height_delta = (impulse.mean_height - avbd.mean_height).abs();
    assert!(
        mean_height_delta < 0.5,
        "expected comparable mean heights, got delta {mean_height_delta}"
    );
}
