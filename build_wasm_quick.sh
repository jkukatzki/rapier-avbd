#!/bin/bash

# Quick start script for building Rapier WASM packages with selectable solver backends.
# This creates a minimal WASM build that you can test immediately.

set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
RAPIER_ROOT="$SCRIPT_DIR"

DIM="3"
SOLVER="avbd"
BUILD_MODE="release"

while [[ $# -gt 0 ]]; do
    case "$1" in
        2|3)
            DIM="$1"
            shift
            ;;
        --solver)
            SOLVER="$2"
            shift 2
            ;;
        --solver=*)
            SOLVER="${1#*=}"
            shift
            ;;
        --release)
            BUILD_MODE="release"
            shift
            ;;
        --dev)
            BUILD_MODE="dev"
            shift
            ;;
        --help|-h)
            cat <<'USAGE'
Usage: ./build_wasm_quick.sh [2|3] [--solver avbd|impulse] [--release|--dev]

Builds a minimal Rapier WASM package configured with the requested solver backend.
  2|3            Dimension to target (defaults to 3).
  --solver       Either `avbd` (default) or `impulse` for the legacy solver.
  --release      Build optimized artifacts (default).
  --dev          Build debug artifacts.
USAGE
            exit 0
            ;;
        *)
            echo "Unknown argument: $1" >&2
            exit 1
            ;;
    esac
done

case "$SOLVER" in
    avbd|impulse)
        ;;
    *)
        echo "Unsupported solver '$SOLVER'. Use 'avbd' or 'impulse'." >&2
        exit 1
        ;;
esac

WASM_DIR="$RAPIER_ROOT/rapier-wasm-$SOLVER"

echo "🚀 Building Rapier WASM with $SOLVER solver for ${DIM}D ($BUILD_MODE mode)..."

if ! command -v wasm-pack &> /dev/null; then
    echo "❌ wasm-pack not found. Installing..."
    cargo install wasm-pack
fi

if [ ! -d "$WASM_DIR" ]; then
    echo "📁 Creating WASM package directory at $WASM_DIR..."
    mkdir -p "$WASM_DIR/src"
fi

if [ "$SOLVER" = "avbd" ]; then
    RAPIER_FEATURES='"solver_avbd", "profiler"'
    PACKAGE_NAME="rapier-wasm-avbd"
else
    RAPIER_FEATURES='"solver_impulse", "profiler"'
    PACKAGE_NAME="rapier-wasm-impulse"
fi

PROFILE_BLOCK=""
if [ "$BUILD_MODE" = "release" ]; then
    PROFILE_BLOCK=$'\n[profile.release]\nopt-level = 3\nlto = true'
fi

echo "📝 Writing Cargo.toml for $PACKAGE_NAME..."
cat > "$WASM_DIR/Cargo.toml" << EOF
[package]
name = "$PACKAGE_NAME"
version = "0.1.0"
edition = "2024"

[lib]
crate-type = ["cdylib", "rlib"]

[dependencies]
wasm-bindgen = "0.2"
serde = { version = "1", features = ["derive"] }
js-sys = "0.3"
console_error_panic_hook = "0.1"

[dependencies.rapier${DIM}d]
path = "../crates/rapier${DIM}d"
default-features = false
features = ["dim${DIM}", "f32", $RAPIER_FEATURES]

[features]
default = []
dim${DIM} = []
$PROFILE_BLOCK
EOF

echo "📝 Creating lib.rs..."
if [ "$DIM" = "3" ]; then
    if [ "$SOLVER" = "avbd" ]; then
        if [ ! -f "$WASM_DIR/src/lib.rs" ]; then
            cp "$RAPIER_ROOT/rapier-wasm-avbd/src/lib.rs" "$WASM_DIR/src/lib.rs"
        fi
        echo "ℹ️ Using AVBD 3D bindings in $WASM_DIR/src/lib.rs"
    else
        cat > "$WASM_DIR/src/lib.rs" << 'EOFLIB3'
use wasm_bindgen::prelude::*;
use rapier3d::prelude::*;

#[wasm_bindgen]
pub fn init_panic_hook() {
    console_error_panic_hook::set_once();
}

#[wasm_bindgen]
pub fn version() -> String {
    env!("CARGO_PKG_VERSION").to_string()
}

#[wasm_bindgen]
pub fn is_avbd_available() -> bool {
    false
}

#[wasm_bindgen]
pub struct RapierWorld {
    physics_pipeline: PhysicsPipeline,
    integration_parameters: IntegrationParameters,
    islands: IslandManager,
    broad_phase: DefaultBroadPhase,
    narrow_phase: NarrowPhase,
    impulse_joint_set: ImpulseJointSet,
    multibody_joint_set: MultibodyJointSet,
    ccd_solver: CCDSolver,
    rigid_body_set: RigidBodySet,
    collider_set: ColliderSet,
    gravity: Vector<Real>,
}

#[wasm_bindgen]
impl RapierWorld {
    #[wasm_bindgen(constructor)]
    pub fn new(gravity_x: f32, gravity_y: f32, gravity_z: f32) -> RapierWorld {
        let mut integration_parameters = IntegrationParameters::default();
        integration_parameters.solver_backend = SolverBackend::Impulse;

        RapierWorld {
            physics_pipeline: PhysicsPipeline::new(),
            integration_parameters,
            islands: IslandManager::new(),
            broad_phase: DefaultBroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            impulse_joint_set: ImpulseJointSet::new(),
            multibody_joint_set: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            rigid_body_set: RigidBodySet::new(),
            collider_set: ColliderSet::new(),
            gravity: Vector::new(gravity_x, gravity_y, gravity_z),
        }
    }

    pub fn step(&mut self) {
        self.physics_pipeline.step(
            &self.gravity,
            &self.integration_parameters,
            &mut self.islands,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.rigid_body_set,
            &mut self.collider_set,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            &mut self.ccd_solver,
            &(),
            &(),
        );
    }

    pub fn num_bodies(&self) -> usize {
        self.rigid_body_set.len()
    }

    pub fn num_colliders(&self) -> usize {
        self.collider_set.len()
    }

    pub fn get_solver_backend(&self) -> String {
        format!("{:?}", self.integration_parameters.solver_backend)
    }
}
EOFLIB3
        echo "ℹ️ Generated impulse 3D bindings in $WASM_DIR/src/lib.rs"
    fi
else
cat > "$WASM_DIR/src/lib.rs" << 'EOFLIB'
use wasm_bindgen::prelude::*;
use rapier2d::{counters::Counters, prelude::*};

#[wasm_bindgen]
pub fn init_panic_hook() {
    console_error_panic_hook::set_once();
}

#[wasm_bindgen]
pub fn version() -> String {
    env!("CARGO_PKG_VERSION").to_string()
}

#[wasm_bindgen]
pub fn is_avbd_available() -> bool {
    true // Always true since solver_avbd is enabled in Cargo.toml
}

#[wasm_bindgen]
pub struct StepMetrics {
    total_ms: f64,
    collision_ms: f64,
    island_ms: f64,
    solver_ms: f64,
    solver_assembly_ms: f64,
    solver_resolution_ms: f64,
    solver_writeback_ms: f64,
}

impl StepMetrics {
    fn from_counters(counters: &Counters) -> Self {
        Self {
            total_ms: counters.step_time_ms(),
            collision_ms: counters.collision_detection_time_ms(),
            island_ms: counters.island_construction_time_ms(),
            solver_ms: counters.solver_time_ms(),
            solver_assembly_ms: counters.solver.velocity_assembly_time.time_ms(),
            solver_resolution_ms: counters.solver.velocity_resolution_time.time_ms(),
            solver_writeback_ms: counters.solver.velocity_writeback_time.time_ms(),
        }
    }
}

#[wasm_bindgen]
impl StepMetrics {
    pub fn total_ms(&self) -> f64 {
        self.total_ms
    }

    pub fn collision_ms(&self) -> f64 {
        self.collision_ms
    }

    pub fn island_ms(&self) -> f64 {
        self.island_ms
    }

    pub fn solver_ms(&self) -> f64 {
        self.solver_ms
    }

    pub fn solver_assembly_ms(&self) -> f64 {
        self.solver_assembly_ms
    }

    pub fn solver_resolution_ms(&self) -> f64 {
        self.solver_resolution_ms
    }

    pub fn solver_writeback_ms(&self) -> f64 {
        self.solver_writeback_ms
    }
}

#[wasm_bindgen]
pub struct RapierWorld {
    physics_pipeline: PhysicsPipeline,
    integration_parameters: IntegrationParameters,
    islands: IslandManager,
    broad_phase: DefaultBroadPhase,
    narrow_phase: NarrowPhase,
    impulse_joint_set: ImpulseJointSet,
    multibody_joint_set: MultibodyJointSet,
    ccd_solver: CCDSolver,
    rigid_body_set: RigidBodySet,
    collider_set: ColliderSet,
    gravity: Vector<Real>,
}

#[wasm_bindgen]
impl RapierWorld {
    #[wasm_bindgen(constructor)]
    pub fn new(gravity_x: f32, gravity_y: f32, use_avbd: bool) -> RapierWorld {
        let mut integration_parameters = IntegrationParameters::default();

        integration_parameters.solver_backend = if use_avbd {
            SolverBackend::Avbd
        } else {
            SolverBackend::Impulse
        };

        RapierWorld {
            physics_pipeline: PhysicsPipeline::new(),
            integration_parameters,
            islands: IslandManager::new(),
            broad_phase: DefaultBroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            impulse_joint_set: ImpulseJointSet::new(),
            multibody_joint_set: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            rigid_body_set: RigidBodySet::new(),
            collider_set: ColliderSet::new(),
            gravity: Vector::new(gravity_x, gravity_y),
        }
    }

    pub fn step(&mut self) {
        self.physics_pipeline.step(
            &self.gravity,
            &self.integration_parameters,
            &mut self.islands,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.rigid_body_set,
            &mut self.collider_set,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            &mut self.ccd_solver,
            &(),
            &(),
        );
    }

    pub fn step_with_metrics(&mut self) -> StepMetrics {
        self.step();
        StepMetrics::from_counters(&self.physics_pipeline.counters)
    }
}
EOFLIB
fi

cd "$WASM_DIR"

if [ "$BUILD_MODE" = "release" ]; then
    wasm-pack build --release --target web
else
    wasm-pack build --dev --target web
fi

echo "✅ Finished building $PACKAGE_NAME for ${DIM}D"
