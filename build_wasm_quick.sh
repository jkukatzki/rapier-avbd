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
features = ["solver_avbd"]

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
cat > "$WASM_DIR/src/lib.rs" << 'EOFLIB'
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
    true // Always true since solver_avbd is enabled in Cargo.toml
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
    pub fn new(gravity_x: f32, gravity_y: f32, gravity_z: f32, use_avbd: bool) -> RapierWorld {
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
    
    /// Create a dynamic rigid body and return its handle
    pub fn create_dynamic_body(&mut self, x: f32, y: f32, z: f32) -> u32 {
        let rigid_body = RigidBodyBuilder::dynamic()
            .translation(Vector::new(x, y, z))
            .build();
        let handle = self.rigid_body_set.insert(rigid_body);
        handle.into_raw_parts().0
    }
    
    /// Create a fixed (static) rigid body and return its handle
    pub fn create_fixed_body(&mut self, x: f32, y: f32, z: f32) -> u32 {
        let rigid_body = RigidBodyBuilder::fixed()
            .translation(Vector::new(x, y, z))
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
    pub fn create_cuboid_collider(&mut self, body_handle: u32, hx: f32, hy: f32, hz: f32) -> u32 {
        let handle = RigidBodyHandle::from_raw_parts(body_handle, 0);
        let collider = ColliderBuilder::cuboid(hx, hy, hz).build();
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
            vec![translation.x, translation.y, translation.z]
        } else {
            vec![0.0, 0.0, 0.0]
        }
    }
}
EOFLIB
else
# 2D version
cat > "$WASM_DIR/src/lib.rs" << 'EOFLIB'
use wasm_bindgen::prelude::*;
use rapier2d::prelude::*;

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
    
    pub fn num_bodies(&self) -> usize {
        self.rigid_body_set.len()
    }
    
    pub fn num_colliders(&self) -> usize {
        self.collider_set.len()
    }
    
    pub fn get_solver_backend(&self) -> String {
        format!("{:?}", self.integration_parameters.solver_backend)
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

cd "$WASM_DIR"

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
