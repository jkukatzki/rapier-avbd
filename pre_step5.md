# Step 5 Planning — Island Integration & Constraint Pooling

## Objectives
- Thread the staged AVBD workspace through Rapier's island solver so contacts and joints operate entirely on cached body states before a single deterministic writeback.【F:post_step4.md†L14-L23】【F:llm_digests/first_attempt_caveats_digest.txt†L16-L27】
- Introduce pooled contact and joint constraint wrappers that reuse gradient, Hessian, and warm-start data without per-iteration allocations, preparing for color-bucket execution.【F:post_step4.md†L24-L30】【F:llm_digests/first_attempt_caveats_digest.txt†L1-L13】
- Extend the build pipeline, wasm packaging, and demo harness to swap the AVBD solver via feature flags while keeping API parity with upstream Rapier.【F:AGENTS.md†L10-L13】【F:scripts/run-avbd-ci.sh†L1-L49】

## Task Checklist
- [x] Feed the island solver with an AVBD path that stages bodies, gathers Rapier contacts/joints into pooled constraint structs, and writes back after solving.
- [x] Build reusable contact constraint pools with nalgebra-backed gradient caching; joint pooling remains pending once the joint path lands.
- [ ] Update Cargo features, build scripts, and the HTML demo so the solver feature cleanly switches between AVBD and legacy PGS builds.
- [ ] Add integration tests exercising ground-plane stabilization, impulse parity, and determinism using the new pooled constraints.
- [ ] Refresh benchmarks to sample the paper's parameter sweep (iteration counts, stiffness ranges) under the AVBD feature flag.

### Immediate focus
- Gate the legacy PGS solver stack behind `cfg(not(feature = "solver_avbd"))` so AVBD-only builds stop compiling unused modules and silence the warning deluge.
- Hoist `MotorParameters` into a shared solver module so public APIs remain available after the PGS modules are gated out.
- Re-export `ContactManifoldExt` from `geometry` for the AVBD tests and update the drop test helpers to use accessor methods (`RigidBody::angvel()`).
- Re-run `cargo check --all-targets --features solver_avbd` to ensure the feature build succeeds without the legacy solver.

## Progress Log
- Initialized Step 5 plan after revisiting the roadmap, caveats digest, and build scripts to scope island integration plus pooling priorities.
- Added a feature-gated AVBD island solver that assembles pooled contact constraints from manifolds, feeds them to the staged workspace, and restores warm-start impulses after solving.【F:src/dynamics/solver/island_solver.rs†L1-L170】【F:src/dynamics/solver/avbd/contact.rs†L1-L222】
- Drafting the solver gating and public API shims required to build cleanly without the legacy PGS modules active.
- Hoisted `MotorParameters` into a shared module, re-exported `ContactManifoldExt`, and wired up the drop tests so `cargo check --all-targets --features solver_avbd` succeeds without compiling the PGS solver path.【F:src/dynamics/solver/motor_parameters.rs†L1-L21】【F:src/geometry/mod.rs†L1-L26】【29b6f6†L1-L3】
