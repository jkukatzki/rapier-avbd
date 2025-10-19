# AVBD Build Pipeline Plan

This plan captures the staged build and packaging flow for the AVBD solver work so each layer (Rust crates, WASM/JS bridge, and benchmarking automation) stays in sync while supporting both the legacy impulse solver and the new backend.

## 1. Rust crate builds

1. **Feature surfaces** – Continue exposing `solver_impulse` and `solver_avbd` as orthogonal flags so integrators can build pure AVBD artifacts with `--no-default-features --features "solver_avbd,dim3,f32"`, or dual-backend binaries when regression coverage is needed.【F:crates/rapier3d/Cargo.toml†L31-L66】【F:src/dynamics/integration_parameters.rs†L36-L71】
2. **Determinism gates** – Keep the `enhanced-determinism` and upcoming parallel toggles independent; AVBD stays sequential by default and flips parallel execution only when both the `parallel` feature and the `allow_parallelism` parameter are enabled. Report telemetry through `AvbdSolver::take_report` so CI can assert determinism before enabling parallel runs.【F:src/dynamics/solver/avbd/solver.rs†L20-L197】
3. **Test cadence** – Standardize on `cargo test -p rapier3d --lib` for default builds and `cargo check -p rapier3d --no-default-features --features "solver_avbd,dim3,f32"` to guard AVBD-only builds. Additional determinism/result-matching tests cover parity with the reference solver and must stay green before shipping parallel variants.【F:src/dynamics/solver/avbd/solver.rs†L577-L702】

## 2. WASM packaging

1. **Build scripts** – Extend `build_wasm_quick.sh` so it accepts a `--solver avbd|impulse` flag that forwards the right feature set to Cargo and generates discrete wasm-bindgen packages (`rapier-wasm-avbd` vs. `rapier-wasm-impulse`).【F:build_wasm_quick.sh†L1-L210】
2. **Dual packages** – Maintain two pnpm workspaces: the upstream Rapier JS bundle and an AVBD override built via local file replacement (mirroring the guidance already captured in `build_js_with_avbd.md`). Each workspace uses the same API surface so the Threlte demos can swap packages without code changes.【F:build_js_with_avbd.md†L1-L130】【F:threlte_preview/package.json†L1-L32】
3. **Digest alignment** – Pull configuration hints from the Houdini/OpenCL digest when tuning memory layouts before exporting to WASM to avoid regressing GPU friendliness later in the roadmap.【F:llm_digests/avbd_houdini_impl_git_repo_digest.txt†L1-L180】

## 3. Benchmark & MCP automation

1. **Node CLI** – Ship a reusable harness (`tools/mcp/server.js`) that shells out to the dedicated `solver-bench` crate, captures timing data, and keeps the scoreboard JSON in sync for historical comparison.【F:tools/mcp/server.js†L1-L214】【F:tools/solver-bench/src/main.rs†L1-L213】
2. **MCP bridge** – Expose the CLI through MCP commands (`build-wasm`, `run-bench`, `get-scoreboard`) and surface pnpm shortcuts so remote triggers can drive builds and benchmarking runs from a single entrypoint.【F:tools/mcp/server.js†L134-L210】【F:package.json†L12-L21】
3. **Result parity** – Gate any Rayon-based execution path behind the benchmark harness once determinism parity assertions pass; the harness should fail fast if AVBD diverges from the impulse baseline beyond an epsilon threshold, leveraging the new result-matching tests as fixtures.【F:src/dynamics/solver/avbd/solver.rs†L577-L702】

## 4. Reporting & documentation

1. **Roadmap sync** – Update `ROADMAP.md` as phases land so contributors know which build stages are production ready (Phase 4 is now tracked through the new instrumentation references).【F:ROADMAP.md†L62-L88】
2. **Frontend alignment** – Document the dual-Threlte setup in the forthcoming web phase, ensuring both builds expose the same API surface per the WASM reference guide.【F:WASM_API_REFERENCE.md†L1-L120】
3. **Digest references** – Continue folding insights from the AVBD paper and discussion digests into this plan when revisiting warm starts, parallel bucket execution, and GPU scheduling strategies.【F:llm_digests/avbd_paper_pdf_digest.txt†L1-L180】【F:llm_digests/gpt5_discussion_digest.txt†L720-L810】

This blueprint keeps the build and validation flow transparent while leaving room to slot in the MCP integration and Rayon experiments once determinism guarantees are locked in.
