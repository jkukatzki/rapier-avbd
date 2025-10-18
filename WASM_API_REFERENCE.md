# Rapier WASM AVBD - Quick API Reference

## Basic Usage

```javascript
import init, { RapierWorld, is_avbd_available } from './rapier-wasm-avbd/pkg/rapier_wasm_avbd.js';

async function run() {
    // Initialize WASM
    await init();
    
    // Check AVBD availability
    console.log('AVBD available:', is_avbd_available());
    
    // Create world with AVBD solver
    const world = new RapierWorld(0.0, -9.81, 0.0, true);
    console.log('Solver:', world.get_solver_backend());
    
    // Create a static ground
    const ground = world.create_fixed_body(0.0, -5.0, 0.0);
    world.create_cuboid_collider(ground, 50.0, 0.5, 50.0);
    
    // Create a falling ball
    const ball = world.create_dynamic_body(0.0, 10.0, 0.0);
    world.create_ball_collider(ball, 0.5);
    
    // Simulate
    for (let i = 0; i < 100; i++) {
        world.step();
    }
    
    // Get position
    const pos = world.get_body_translation(ball);
    console.log('Ball position:', pos);
}

run();
```

## API Methods

### World Management

- **`new RapierWorld(gravity_x, gravity_y, gravity_z, use_avbd)`** - Create a new physics world
  - `gravity_x/y/z`: Gravity vector components
  - `use_avbd`: `true` to use AVBD solver, `false` for default impulse solver

- **`step()`** - Advance simulation by one time step

- **`get_solver_backend()`** - Returns the active solver backend ("Avbd" or "Impulse")

- **`num_bodies()`** - Get number of rigid bodies in the world

- **`num_colliders()`** - Get number of colliders in the world

### Rigid Body Creation

- **`create_dynamic_body(x, y, z)`** - Create a dynamic (movable) rigid body
  - Returns: body handle (u32)

- **`create_fixed_body(x, y, z)`** - Create a fixed (static) rigid body
  - Returns: body handle (u32)

- **`get_body_translation(handle)`** - Get position of a rigid body
  - Returns: `[x, y, z]` array

### Collider Creation

All collider methods require a body handle and return a collider handle (u32).

- **`create_ball_collider(body_handle, radius)`** - Attach a sphere collider
  
- **`create_cuboid_collider(body_handle, hx, hy, hz)`** - Attach a box collider
  - `hx/hy/hz`: Half-extents (half-width, half-height, half-depth)

## Example: Falling Spheres

```javascript
const world = new RapierWorld(0.0, -9.81, 0.0, true);

// Create ground
const ground = world.create_fixed_body(0.0, -5.0, 0.0);
world.create_cuboid_collider(ground, 50.0, 0.5, 50.0);

// Create falling spheres
const balls = [];
for (let i = 0; i < 10; i++) {
    const x = (Math.random() - 0.5) * 10.0;
    const y = 10.0 + i * 2.0;
    const z = (Math.random() - 0.5) * 10.0;
    
    const ball = world.create_dynamic_body(x, y, z);
    world.create_ball_collider(ball, 0.5);
    balls.push(ball);
}

// Simulate 100 steps
for (let i = 0; i < 100; i++) {
    world.step();
}

// Check final positions
balls.forEach((handle, i) => {
    const pos = world.get_body_translation(handle);
    console.log(`Ball ${i}: [${pos[0].toFixed(2)}, ${pos[1].toFixed(2)}, ${pos[2].toFixed(2)}]`);
});
```

## Testing

Open `test_wasm_avbd.html` in your browser (via http://localhost:8000) to see an interactive demo with:
- AVBD solver verification
- Body/collider creation
- Step-by-step simulation
- Performance metrics

## AVBD Solver Benefits

The AVBD (Augmented Vertex Block Descent) solver provides:
- Improved convergence for stiff constraints
- Better handling of high mass ratios
- More stable joints and contact constraints
- Parallel-friendly architecture

Use `use_avbd: true` when creating the world to enable it.
