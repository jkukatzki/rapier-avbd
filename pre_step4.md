# Step 4 Planning — Workspace Staging & Caveat Integration

## Objectives
- Translate the first-attempt caveats into concrete refactors for the AVBD workspace so constraints operate on staged body poses instead of mutating the live `RigidBodySet`.【F:llm_digests/first_attempt_caveats_digest.txt†L1-L32】
- Outline pooling and scratch-buffer reuse strategies that cut per-constraint allocations and prepare for deterministic color bucket execution in later steps.【F:llm_digests/avbd_2d_demo_digest.txt†L2385-L2444】【F:llm_digests/avbd_houdini_impl_git_repo_digest.txt†L171-L203】
- Ensure the build pipeline can switch to the AVBD backend via a dedicated feature flag while keeping the wasm packaging story aligned with upstream Rapier targets.

## Task Checklist
- [x] Capture the lessons-learned checklist from the caveats digest and thread them through the solver workspace design.
- [x] Refactor `AvbdSolver` to stage body state, reuse scratch buffers, and expose a constraint view detached from `RigidBodySet` mutation.
- [x] Extend the AVBD unit/integration test coverage to include ground-plane penetration guards, determinism assertions, and impulse parity checks.
- [x] Update the wasm/demo assets and build scripts to recognise the AVBD feature toggle and document the HTML entry point for testing the solver in Three.js.

## Progress Log
- Initialized Step 4 plan with focus on workspace staging, scratch pooling, and build flag wiring for the AVBD backend.
- Reworked the AVBD solver workspace to stage per-body poses, added pooled scratch buffers, and introduced a view API so constraints no longer mutate the live `RigidBodySet` during iterations.
- Added feature-aware publishing scripts, a Three.js HTML harness that exercises AVBD via instanced meshes and joints, and integration-style tests covering ground-plane penetration, determinism, and impulse parity.
