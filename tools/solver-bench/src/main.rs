use std::{
    env,
    time::{SystemTime, UNIX_EPOCH},
};

use anyhow::{Context, Result, bail};
use rapier3d::{na, prelude::*};
use serde::Serialize;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum SolverChoice {
    Avbd,
    Impulse,
}

impl SolverChoice {
    fn as_str(self) -> &'static str {
        match self {
            SolverChoice::Avbd => "avbd",
            SolverChoice::Impulse => "impulse",
        }
    }

    fn from_str(value: &str) -> Result<Self> {
        match value {
            "avbd" => Ok(SolverChoice::Avbd),
            "impulse" | "pgs" => Ok(SolverChoice::Impulse),
            other => bail!("unsupported solver '{other}'"),
        }
    }
}

#[derive(Debug)]
struct Config {
    solver: SolverChoice,
    steps: usize,
    stack_height: usize,
    dt: Real,
}

impl Config {
    fn from_args() -> Result<Self> {
        let mut solver = SolverChoice::Avbd;
        let mut steps: usize = 120;
        let mut stack_height: usize = 20;
        let mut dt: Real = na::convert(1.0 / 60.0);

        let mut args = env::args().skip(1);
        while let Some(arg) = args.next() {
            match arg.as_str() {
                "--solver" => {
                    let value = args
                        .next()
                        .context("expected value after --solver (avbd|impulse)")?;
                    solver = SolverChoice::from_str(&value)?;
                }
                "--steps" => {
                    let value = args
                        .next()
                        .context("expected numeric value after --steps")?;
                    steps = value.parse().context("unable to parse --steps")?;
                }
                "--stack-height" => {
                    let value = args
                        .next()
                        .context("expected numeric value after --stack-height")?;
                    stack_height = value.parse().context("unable to parse --stack-height")?;
                }
                "--dt" => {
                    let value = args.next().context("expected numeric value after --dt")?;
                    dt = value.parse().context("unable to parse --dt")?;
                }
                "--help" | "-h" => {
                    print_usage();
                    std::process::exit(0);
                }
                other => bail!("unknown argument '{other}'"),
            }
        }

        if steps == 0 {
            bail!("--steps must be greater than zero");
        }
        if stack_height == 0 {
            bail!("--stack-height must be greater than zero");
        }
        if dt <= 0.0 {
            bail!("--dt must be positive");
        }

        Ok(Config {
            solver,
            steps,
            stack_height,
            dt,
        })
    }
}

#[derive(Serialize)]
struct StageTotals {
    total_ms: f64,
    solver_ms: f64,
    assembly_ms: f64,
    resolution_ms: f64,
    writeback_ms: f64,
}

#[derive(Serialize)]
struct BenchResult {
    solver: &'static str,
    steps: usize,
    stack_height: usize,
    dt: Real,
    scenario: &'static str,
    timestamp_ms: u128,
    totals: StageTotals,
    averages: StageTotals,
    constraints: usize,
    contacts: usize,
    body_count: usize,
}

fn main() -> Result<()> {
    let config = Config::from_args()?;
    let result = run_benchmark(&config)?;
    let json = serde_json::to_string_pretty(&result)?;
    println!("{}", json);
    Ok(())
}

fn run_benchmark(config: &Config) -> Result<BenchResult> {
    ensure_profiler_support()?;

    let solver_backend = match config.solver {
        SolverChoice::Avbd => select_avbd_backend()?,
        SolverChoice::Impulse => select_impulse_backend()?,
    };

    let mut pipeline = PhysicsPipeline::new();
    let mut integration_parameters = IntegrationParameters::default();
    integration_parameters.dt = config.dt;
    integration_parameters.solver_backend = solver_backend;

    let gravity = vector![0.0, -9.81, 0.0];
    let mut islands = IslandManager::new();
    let mut broad_phase = DefaultBroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();
    let mut ccd_solver = CCDSolver::new();

    build_stack_scene(config.stack_height, &mut bodies, &mut colliders);

    let mut total_step_ms = 0.0;
    let mut total_solver_ms = 0.0;
    let mut total_assembly_ms = 0.0;
    let mut total_resolution_ms = 0.0;
    let mut total_writeback_ms = 0.0;
    let mut total_constraints = 0usize;
    let mut total_contacts = 0usize;

    for _ in 0..config.steps {
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

        let counters = &pipeline.counters;
        total_step_ms += counters.step_time_ms();
        total_solver_ms += counters.solver.velocity_resolution_time.time_ms();
        total_assembly_ms += counters.solver.velocity_assembly_time.time_ms();
        total_resolution_ms += counters.solver.velocity_update_time.time_ms();
        total_writeback_ms += counters.solver.velocity_writeback_time.time_ms();
        total_constraints += counters.solver.nconstraints;
        total_contacts += counters.solver.ncontacts;
    }

    let steps = config.steps as f64;
    let totals = StageTotals {
        total_ms: total_step_ms,
        solver_ms: total_solver_ms,
        assembly_ms: total_assembly_ms,
        resolution_ms: total_resolution_ms,
        writeback_ms: total_writeback_ms,
    };
    let averages = StageTotals {
        total_ms: total_step_ms / steps,
        solver_ms: total_solver_ms / steps,
        assembly_ms: total_assembly_ms / steps,
        resolution_ms: total_resolution_ms / steps,
        writeback_ms: total_writeback_ms / steps,
    };
    let timestamp_ms = current_timestamp_ms()?;

    Ok(BenchResult {
        solver: config.solver.as_str(),
        steps: config.steps,
        stack_height: config.stack_height,
        dt: config.dt,
        scenario: "stack_drop",
        timestamp_ms,
        totals,
        averages,
        constraints: total_constraints / config.steps,
        contacts: total_contacts / config.steps,
        body_count: bodies.len(),
    })
}

fn build_stack_scene(stack_height: usize, bodies: &mut RigidBodySet, colliders: &mut ColliderSet) {
    let ground_body = RigidBodyBuilder::fixed()
        .translation(vector![0.0, -0.5, 0.0])
        .build();
    let ground_handle = bodies.insert(ground_body);
    let ground_collider = ColliderBuilder::cuboid(10.0, 0.5, 10.0)
        .friction(0.7)
        .restitution(0.2)
        .build();
    colliders.insert_with_parent(ground_collider, ground_handle, bodies);

    let cube_collider = ColliderBuilder::cuboid(0.5, 0.5, 0.5)
        .friction(0.6)
        .restitution(0.1)
        .build();

    for i in 0..stack_height {
        let y = 1.0 + (i as Real) * 1.05;
        let body = RigidBodyBuilder::dynamic()
            .translation(vector![0.0, y, 0.0])
            .can_sleep(false)
            .build();
        let handle = bodies.insert(body);
        colliders.insert_with_parent(cube_collider.clone(), handle, bodies);
    }
}

fn ensure_profiler_support() -> Result<()> {
    if !cfg!(feature = "profiler") {
        bail!("solver-bench must be compiled with the 'profiler' feature for accurate timings");
    }
    Ok(())
}

fn select_avbd_backend() -> Result<SolverBackend> {
    if !cfg!(feature = "solver_avbd") {
        bail!("binary built without solver_avbd feature");
    }
    #[allow(unreachable_code)]
    {
        #[cfg(feature = "solver_avbd")]
        {
            return Ok(SolverBackend::Avbd);
        }
        #[cfg(not(feature = "solver_avbd"))]
        unreachable!()
    }
}

fn select_impulse_backend() -> Result<SolverBackend> {
    if !cfg!(feature = "solver_impulse") {
        bail!("binary built without solver_impulse feature");
    }
    Ok(SolverBackend::Impulse)
}

fn current_timestamp_ms() -> Result<u128> {
    let now = SystemTime::now();
    let duration = now
        .duration_since(UNIX_EPOCH)
        .context("system clock set before UNIX epoch")?;
    Ok(duration.as_millis())
}

fn print_usage() {
    eprintln!("solver-bench usage:");
    eprintln!(
        "  cargo run --package solver-bench -- [--solver avbd|impulse] [--steps N] [--stack-height N] [--dt seconds]"
    );
    eprintln!("defaults: --solver avbd --steps 120 --stack-height 20 --dt 0.0166667");
}
