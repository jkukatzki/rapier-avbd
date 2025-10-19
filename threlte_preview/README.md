# Threlte Viewer Workspace

This workspace hosts two dedicated SvelteKit/Threlte applications used to visualise
Rapier physics builds and exercise interactive benchmarks:

- `apps/impulse-viewer` renders the upstream Rapier solver using the published
  `@dimforge/rapier3d-compat` package.
- `apps/avbd-viewer` renders the AVBD solver build by overriding the Rapier
  dependency with the locally compiled `rapier-wasm-avbd` package.

Use `pnpm dev:impulse` or `pnpm dev:avbd` from this directory to launch either
viewer. Each viewer exposes identical controls so results can be compared side by
side while retaining fully isolated dependency graphs.

Run `pnpm test` to execute shared workspace smoke tests that guard the
simulation-state stores powering both viewers.

> **Note:** The AVBD viewer expects `rapier-wasm-avbd/pkg` to be generated via the
> root `build_wasm_quick.sh` workflow. Run `pnpm install` inside this workspace
> after building the WASM package to wire the local link dependency.
