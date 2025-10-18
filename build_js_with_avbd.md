# Building JavaScript Bindings with AVBD Feature

This guide explains how to build JavaScript/TypeScript bindings for your forked Rapier with the AVBD solver enabled.

## Prerequisites

1. Install `wasm-pack`:
   ```bash
   cargo install wasm-pack
   ```

2. Install Node.js and npm (if not already installed)

## Method 1: Using rapier.js with Custom Rapier Dependency

### Step 1: Clone rapier.js

```bash
cd /Users/fogi/Documents/workspaces/validvector
git clone https://github.com/dimforge/rapier.js.git rapier-avbd-js
cd rapier-avbd-js
```

### Step 2: Modify Cargo Dependencies

The rapier.js builds use a build system in `builds/prepare_builds/`. You need to modify the templates to:

1. Add your custom rapier dependency
2. Enable the `solver_avbd` feature

Edit `builds/prepare_builds/templates/Cargo.toml.tera`:

Find the dependencies section and modify it to point to your local rapier:

```toml
[dependencies]
wasm-bindgen = "0.2.92"
serde = { version = "1", features = [ "derive" ] }
bincode = "1"
instant = { version = "0.1", features = [ "wasm-bindgen", "inaccurate" ] }
js-sys = "0.3"

# Use your local fork with avbd feature
[dependencies.rapier{{ dimension }}d]
path = "/Users/fogi/Documents/workspaces/validvector/rapier-avbd/crates/rapier{{ dimension }}d"
features = [{% for feature in additional_features %}"{{ feature }}", {% endfor %}"solver_avbd"]

[dependencies.nalgebra]
version = "0.34"
features = [ "serde-serialize" ]
```

### Step 3: Generate Build Configurations

```bash
# Generate build configurations for 2D and 3D with your modifications
cd builds/prepare_builds
cargo run -- -d dim2 -f non-deterministic
cargo run -- -d dim3 -f non-deterministic
```

### Step 4: Build the WASM Packages

For **3D** with AVBD:
```bash
cd ../rapier3d
./build_rust.sh
./build_typescript.sh
```

For **2D** with AVBD:
```bash
cd ../rapier2d
./build_rust.sh
./build_typescript.sh
```

### Step 5: Test Locally

```bash
# Link the package locally for testing
cd ../rapier3d/pkg
npm link

# In your test project
npm link @dimforge/rapier3d
```

## Method 2: Quick Manual Build (Simpler for Testing)

If you just want to quickly build and test without the full rapier.js infrastructure:

### Step 1: Create a Simple WASM Package in Your Rapier Fork

Create a new directory in your rapier fork:

```bash
cd /Users/fogi/Documents/workspaces/validvector/rapier-avbd
mkdir -p rapier-wasm-avbd
cd rapier-wasm-avbd
```

### Step 2: Create Cargo.toml

```toml
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

[dependencies.rapier3d]
path = "../crates/rapier3d"
features = ["solver_avbd"]

# For 2D, use this instead:
# [dependencies.rapier2d]
# path = "../crates/rapier2d"
# features = ["solver_avbd"]

[profile.release]
opt-level = 3
lto = true
```

### Step 3: Create Minimal Rust Wrapper

Create `src/lib.rs`:

```rust
use wasm_bindgen::prelude::*;
use rapier3d::prelude::*;

#[wasm_bindgen]
pub fn init() {
    console_error_panic_hook::set_once();
}

// Re-export key types and functions
#[wasm_bindgen]
pub struct World {
    physics_pipeline: PhysicsPipeline,
    integration_parameters: IntegrationParameters,
    islands: IslandManager,
    broad_phase: DefaultBroadPhase,
    narrow_phase: NarrowPhase,
    impulse_joint_set: ImpulseJointSet,
    multibody_joint_set: MultibodyJointSet,
    ccd_solver: CCDSolver,
    query_pipeline: QueryPipeline,
    physics_hooks: (),
    event_handler: (),
    rigid_body_set: RigidBodySet,
    collider_set: ColliderSet,
    gravity: Vector<Real>,
}

#[wasm_bindgen]
impl World {
    #[wasm_bindgen(constructor)]
    pub fn new(gravity_x: f32, gravity_y: f32, gravity_z: f32) -> World {
        let mut integration_parameters = IntegrationParameters::default();
        
        // Enable AVBD solver
        #[cfg(feature = "solver_avbd")]
        {
            integration_parameters.solver_type = SolverType::Avbd;
        }
        
        World {
            physics_pipeline: PhysicsPipeline::new(),
            integration_parameters,
            islands: IslandManager::new(),
            broad_phase: DefaultBroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            impulse_joint_set: ImpulseJointSet::new(),
            multibody_joint_set: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            query_pipeline: QueryPipeline::new(),
            physics_hooks: (),
            event_handler: (),
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
            Some(&mut self.query_pipeline),
            &self.physics_hooks,
            &self.event_handler,
        );
    }
}

// Add more wrappers as needed for your use case
```

### Step 4: Build with wasm-pack

```bash
wasm-pack build --target web --release
```

This will create a `pkg/` directory with your WASM module and JavaScript bindings.

### Step 5: Use in Your Web Project

```javascript
import init, { World } from './rapier-wasm-avbd/pkg/rapier_wasm_avbd.js';

async function run() {
    await init();
    
    const world = new World(0.0, -9.81, 0.0);
    
    // Use the world
    world.step();
}

run();
```

## Verifying AVBD is Enabled

To confirm the AVBD solver is being used, you can add logging in your Rust code:

```rust
#[wasm_bindgen]
pub fn check_solver_type() -> String {
    #[cfg(feature = "solver_avbd")]
    return "AVBD solver is enabled".to_string();
    
    #[cfg(not(feature = "solver_avbd"))]
    return "AVBD solver is NOT enabled".to_string();
}
```

## Next Steps

1. **For Method 1**: You'll have full TypeScript types and the complete API
2. **For Method 2**: You'll need to manually wrap more Rapier functionality as needed

I recommend Method 2 for initial testing, then switch to Method 1 once you confirm everything works with AVBD.

## Troubleshooting

- If you get linking errors, make sure `solver_avbd` feature is properly enabled in your Cargo.toml
- Check that your rapier fork actually has the AVBD solver code compiled in
- Use `cargo build --features solver_avbd` to test the Rust build before trying WASM
