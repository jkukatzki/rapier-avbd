#!/bin/bash

# Quick start script for building Rapier WASM with AVBD solver
# This creates a minimal WASM build that you can test immediately

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
RAPIER_ROOT="$SCRIPT_DIR"
WASM_DIR="$RAPIER_ROOT/rapier-wasm-avbd"

echo "🚀 Building Rapier WASM with AVBD solver..."
echo

# Check if wasm-pack is installed
if ! command -v wasm-pack &> /dev/null; then
    echo "❌ wasm-pack not found. Installing..."
    cargo install wasm-pack
fi

# Create WASM package directory if it doesn't exist
if [ ! -d "$WASM_DIR" ]; then
    echo "📁 Creating WASM package directory..."
    mkdir -p "$WASM_DIR/src"
fi

# Detect dimension (default to 3D)
DIM=${1:-3}
echo "📐 Building for ${DIM}D"

# Create Cargo.toml
echo "📝 Creating Cargo.toml..."
cat > "$WASM_DIR/Cargo.toml" << EOF
[package]
name = "rapier-wasm-avbd"
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
features = ["solver_avbd", "profiler"]

[features]
default = []
dim${DIM} = []

[profile.release]
opt-level = 3
lto = true
EOF

# Create basic lib.rs with AVBD-enabled world - dimension specific
echo "📝 Creating lib.rs..."
if [ "$DIM" = "3" ]; then
    echo "ℹ️ Using existing 3D bindings in rapier-wasm-avbd/src/lib.rs"
else
# 2D version
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
        
        if use_avbd {
            integration_parameters.solver_backend = SolverBackend::Avbd;
        }
        
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

    pub fn reset(&mut self) {
        self.islands = IslandManager::new();
        self.broad_phase = DefaultBroadPhase::new();
        self.narrow_phase = NarrowPhase::new();
        self.impulse_joint_set = ImpulseJointSet::new();
        self.multibody_joint_set = MultibodyJointSet::new();
        self.ccd_solver = CCDSolver::new();
        self.rigid_body_set = RigidBodySet::new();
        self.collider_set = ColliderSet::new();
        self.physics_pipeline.counters.reset();
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

    pub fn solver_iterations(&self) -> u32 {
        self.integration_parameters.num_solver_iterations as u32
    }

    pub fn set_solver_iterations(&mut self, iterations: u32) {
        self.integration_parameters.num_solver_iterations = iterations.max(1) as usize;
    }

    pub fn warmstart_coefficient(&self) -> f32 {
        self.integration_parameters.warmstart_coefficient as f32
    }

    pub fn set_warmstart_coefficient(&mut self, value: f32) {
        let clamped = value.clamp(0.0, 1.0);
        self.integration_parameters.warmstart_coefficient = clamped as Real;
    }

    /// Create a dynamic rigid body and return its handle
    pub fn create_dynamic_body(&mut self, x: f32, y: f32) -> u32 {
        let rigid_body = RigidBodyBuilder::dynamic()
            .translation(Vector::new(x, y))
            .build();
        let handle = self.rigid_body_set.insert(rigid_body);
        handle.into_raw_parts().0
    }
    
    /// Create a fixed (static) rigid body and return its handle
    pub fn create_fixed_body(&mut self, x: f32, y: f32) -> u32 {
        let rigid_body = RigidBodyBuilder::fixed()
            .translation(Vector::new(x, y))
            .build();
        let handle = self.rigid_body_set.insert(rigid_body);
        handle.into_raw_parts().0
    }
    
    /// Create a ball collider attached to a rigid body
    pub fn create_ball_collider(&mut self, body_handle: u32, radius: f32) -> u32 {
        let handle = RigidBodyHandle::from_raw_parts(body_handle, 0);
        let collider = ColliderBuilder::ball(radius).build();
        let collider_handle = self.collider_set.insert_with_parent(
            collider,
            handle,
            &mut self.rigid_body_set,
        );
        collider_handle.into_raw_parts().0
    }
    
    /// Create a cuboid collider attached to a rigid body
    pub fn create_cuboid_collider(&mut self, body_handle: u32, hx: f32, hy: f32) -> u32 {
        let handle = RigidBodyHandle::from_raw_parts(body_handle, 0);
        let collider = ColliderBuilder::cuboid(hx, hy).build();
        let collider_handle = self.collider_set.insert_with_parent(
            collider,
            handle,
            &mut self.rigid_body_set,
        );
        collider_handle.into_raw_parts().0
    }
    
    /// Get the translation (position) of a rigid body
    pub fn get_body_translation(&self, body_handle: u32) -> Vec<f32> {
        let handle = RigidBodyHandle::from_raw_parts(body_handle, 0);
        if let Some(body) = self.rigid_body_set.get(handle) {
            let translation = body.translation();
            vec![translation.x, translation.y]
        } else {
            vec![0.0, 0.0]
        }
    }
}
EOFLIB
fi

echo "🧪 Running Rust AVBD smoke benchmarks before building..."
if [ "$DIM" = "3" ]; then
    (cd "$RAPIER_ROOT" && cargo test -p rapier3d --features solver_avbd --test avbd_bench -- --nocapture)
else
    echo "ℹ️ Skipping solver benchmarks for 2D build (AVBD 2D backend under development)."
fi

cd "$WASM_DIR"

echo "⚙️  Enabling WebAssembly SIMD for optimized builds..."
export RUSTFLAGS="${RUSTFLAGS:-} -C target-feature=+simd128"

echo "🔨 Building WASM package with wasm-pack..."
wasm-pack build --target web --release

echo
echo "✅ Build complete!"
echo
echo "📦 Your WASM package is in: $WASM_DIR/pkg/"
echo
echo "To use it in your web project:"
echo "  import init, { RapierWorld, is_avbd_available } from './rapier-wasm-avbd/pkg/rapier_wasm_avbd.js';"
echo
echo "  async function run() {"
echo "    await init();"
echo "    console.log('AVBD available:', is_avbd_available());"
echo "    const world = new RapierWorld(0.0, -9.81, 0.0, true); // true = use AVBD"
echo "    console.log('Solver:', world.get_solver_backend());"
echo "  }"
echo
