## Repository architecture

The architecture of this repository is a bit unusual because we are using some tricks to have both
the 2D and 3D version of Rapier share the same code-base. Here are the main folders:
- **`build/`**: contains one folder per Rapier crate (for the 2D, 3D, `f32`, and `f64` versions). Each
  crate has its own `Cargo.toml` file that adjusts some cargo features, and reference the `src` folder.
- **`src/`**: contains the actual `.rs` source code of the Rapier physics engine.
- **`src_testbed/`**: contains the `.rs` source code of the Rapier testbed (which our examples are based on).
- **`examples2d/`**: simple 2D scenes showcasing some of Rapier's capabilities.
  Run them with `cargo run --release --bin all_examples2`.
- **`examples3d/`**: simple 3D scenes showcasing some of Rapier's capabilities.
  Run them with `cargo run --release --bin all_examples3`.
- **`benchmarks2d/`**: a set of 2D stress-tests, to see how Rapier performs when it has lots of elements
  to simulate.
- **`benchmarks3d/`**: a set of 3D stress-tests, to see how Rapier performs when it has lots of elements
  to simulate. We use the these benchmarks to track the performances of Rapier after some changes,
  and spot unexpected regressions: https://www.rapier.rs/benchmarks/

## AVBD Solver Design Sketch

The AVBD backend will operate on a body-centric workspace that mirrors the paper’s per-vertex updates:

- **BodyWorkspace** – dense SoA buffers (positions, velocities, inverse masses/inertia) per active rigid body captured before the solve. Backed by `Vec<Real>`/`Vec<AngVector<Real>>` with precomputed strides to minimize cache misses. `nalgebra`’s `Vector`/`Matrix` types provide the block operations needed for 3×3 (2D) or 6×6 (3D) solves.
- **ConstraintBatch** – constraints are partitioned by graph-color buckets so that each batch can run in parallel without conflicting bodies. Each entry stores: handles, gradient cache, Hessian approximation, warm-start λ, adaptive stiffness, and metadata (friction limits, compliance).
- **AugmentedState** – encapsulates dual variables and stiffness escalation (`lambda`, `stiffness`, `alpha/beta/gamma` ramps). Warm-start history is maintained in contiguous buffers for SIMD-friendly decay.
- **Solve loop** – for every iteration the solver traverses color buckets, loads body blocks, computes direction (gradient × mass inverse), updates positions, and records the λ delta. Dual/stiffness updates follow the augmented Lagrangian schedule. Parallel execution hooks (Rayon or Rapier’s thread pool) operate at the bucket level to ensure determinism per color.
- **Writeback** – after iterations finish, body deltas are accumulated back into `RigidBodySet`, recomputing velocities from pose deltas just as the current scaffold does.

Future extensions will add buffer pooling (arena allocator) to reuse `BodyWorkspace` allocations across frames and integrate instrumentation counters for benchmarking.

### Solver configuration surfaces

- **Rust API** – expose `IntegrationParameters::solver_backend` with feature-gated variants plus an `AvbdSolverParams` struct accessible through `PhysicsPipeline` setup or builder helpers. Provide `set_avbd_params(&mut self, AvbdSolverParams)` on integration parameters when the feature is active.
- **WASM bridge** – mirror rapier.js API by exporting `init`, `world.step`, and configuration mutators (`setSolverIterations`, `setAvbdParams`). Provide dual packages: `@validvector/rapier3d-avbd` (local link) for AVBD and upstream `@dimforge/rapier3d-compat` for baseline. Each bundle reports availability via `is_avbd_available()` and surfaces benchmark hooks.
- **Node CLI / MCP** – Node tooling invokes cargo builds with `--features solver_avbd` or `--no-default-features --features solver_avbd` and drives benchmarks via a script that loads the appropriate WASM bridge, capturing metrics for the scoreboard.
