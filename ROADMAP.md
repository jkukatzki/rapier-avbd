# AVBD Integration Roadmap

## Phase 0: Discovery and Requirements Alignment
- [x] Inventory existing constraint solver integration points in the repository (Rust crates, build scripts, JS bridge).
  - Identified AVBD scaffolding under `src/dynamics/solver/avbd`, integration parameters toggles, and current WASM wrapper (`rapier-wasm-avbd`). Existing build scripts (e.g., `build_wasm_quick.sh`) already enable `solver_avbd` but legacy solver paths remain active.
- [x] Review AVBD reference materials (paper digest, Houdini repo digest, 2D implementation digest) and capture implementation notes relevant to Rapier.
  - Paper digest emphasizes augmented Lagrangian updates (λ warm-start, adaptive stiffness α/β/γ) and per-body block solves for stability with high stiffness ratios.【F:llm_digests/avbd_paper_pdf_digest.txt†L1-L120】
  - 2D demo showcases practical loop structure (constraint gradients, Hessian approximation, warmstarting, friction bounds).【F:llm_digests/avbd_2d_demo_digest.txt†L1-L90】【F:llm_digests/avbd_2d_demo_digest.txt†L485-L1156】
  - Houdini implementation highlights graph coloring for parallel constraint processing and OpenCL memory layout strategies for AVBD-style solvers.【F:llm_digests/avbd_houdini_impl_git_repo_digest.txt†L1-L120】
- [x] Evaluate existing WASM build/test infrastructure and identify gaps for AVBD support (including pnpm/Node integration, benchmark runners, MCP linkage requirements).
  - Quick build script generates AVBD-targeted wasm package but hardcodes solver availability and lacks dual-package switching for stock Rapier.【F:build_wasm_quick.sh†L1-L120】
  - Current JS README and package scripts focus on single-package testing with http-server; no benchmark CLI or pnpm workspace separation yet.【F:JS_BUILD_README.md†L1-L120】【F:package.json†L1-L15】
  - Threlte preview links AVBD package locally alongside upstream compat build, indicating need for split frontends and pnpm overrides; MCP/Node automation absent.【F:threlte_preview/package.json†L1-L32】

## Phase 1: Architecture Definition
- [x] Define crate-level feature flags toggling AVBD solver inclusion and removing legacy PGS solver from AVBD builds.
  - Added explicit `solver_impulse` feature with defaults while keeping `solver_avbd` opt-in across rapier crates, enabling `--no-default-features --features solver_avbd` builds that exclude the legacy solver path.【F:crates/rapier3d/Cargo.toml†L33-L53】【F:crates/rapier2d/Cargo.toml†L29-L53】
- [x] Sketch data structures and memory layout for AVBD in Rust, leveraging nalgebra for math primitives and considering parallel bucket coloring.
  - Documented BodyWorkspace, ConstraintBatch, AugmentedState, and solve/writeback flow with planned SoA buffers and bucket-based parallelism.【F:ARCHITECTURE.md†L17-L33】
- [x] Specify interfaces for solver configuration in Rust and parity expectations for the JS/WASM bridge (mirroring Rapier's public API).
  - Documented Rust integration parameter hooks, WASM exports mirroring rapier.js, and Node/MCP entry points for feature toggling and benchmarks.【F:ARCHITECTURE.md†L35-L45】

## Phase 2: Core Solver Scaffolding
- [ ] Implement Rust module scaffolding for AVBD solver with placeholder algorithms, integrating with existing step pipeline hooks.
- [ ] Establish optional compilation path conditioned on the AVBD feature flag and ensure legacy solver is excluded in that configuration.
- [ ] Write initial unit tests covering solver selection logic and compile-time feature enforcement.

## Phase 3: Algorithm Implementation
- [ ] Implement primal AVBD iteration (vertex block updates, constraint projection) using nalgebra structures.
- [ ] Add augmented Lagrangian dual updates and stiffness escalation controls.
- [ ] Introduce color bucket partitioning and parallel execution scaffolding (Rayon/parallel iterators or existing Rapier threadpool hooks).
- [ ] Validate solver stability against baseline scenarios with integration tests.

## Phase 4: Performance & Memory Strategy
- [ ] Profile memory usage patterns and implement pooled buffers / arena allocators optimized for AVBD iterations.
- [ ] Tune cache-friendly data layouts (SoA where beneficial) and ensure minimal allocations per step.
- [ ] Integrate instrumentation hooks for benchmarking (timers, iteration counters) exposed to Rust and JS.

## Phase 5: Tooling, Build, and MCP Integration
- [ ] Update build scripts to include AVBD feature flag handling for Cargo, wasm-bindgen, and pnpm workflows (separate packages for baseline vs AVBD builds).
- [ ] Create Node.js benchmarking CLI with MCP interface enabling remote invocation of builds/tests and capturing performance metrics.
- [ ] Maintain scoreboard persistence (e.g., JSON history) and copy of benchmarked binaries/source snapshots per iteration.

## Phase 6: Web Frontend & Visualization
- [ ] Produce two Threlte frontends: one loading stock Rapier WASM, one loading AVBD WASM via pnpm local replacement.
- [ ] Mirror standard Rapier JS bridge API surface for AVBD build and expose solver configuration/benchmark controls.
- [ ] Implement UI controls for running benchmark scenarios and displaying comparative results (graphs/tables).

## Phase 7: Validation & Documentation
- [ ] Expand integration test suite covering physics scenarios, JS bridge parity, and benchmarking outputs.
- [ ] Document usage in README/BUILD guides, including MCP setup instructions and roadmap adherence.
- [ ] Prepare final benchmarking reports comparing AVBD vs PGS with scoreboard outputs.
