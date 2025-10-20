#!/usr/bin/env bash

set -euo pipefail

if ! command -v cargo >/dev/null 2>&1; then
  echo "cargo is required to run this script" >&2
  exit 1
fi

if ! command -v wasm-pack >/dev/null 2>&1; then
  echo "wasm-pack is required to build the wasm package" >&2
  exit 1
fi

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

FEATURE_SET=${RAPIER_FEATURES:-solver_avbd}
SOLVER_PACKAGES=(rapier2d rapier2d-f64 rapier3d rapier3d-f64)
WASM_CRATE_DIR=${WASM_CRATE_DIR:-rapier-wasm-avbd}
WASM_TARGET=${WASM_TARGET:-web}
WASM_PROFILE=${WASM_PROFILE:-release}
WASM_OUT_DIR=${WASM_OUT_DIR:-pkg}
RUN_BENCHMARKS=${RUN_BENCHMARKS:-0}

if [[ ! -d "$WASM_CRATE_DIR" ]]; then
  echo "wasm crate directory '$WASM_CRATE_DIR' does not exist" >&2
  exit 1
fi

run() {
  echo "+ $*"
  "$@"
}

echo "Running default feature test suite"
run cargo test --workspace --all-targets

echo "Running solver feature test suites"
for package in "${SOLVER_PACKAGES[@]}"; do
  run cargo test -p "$package" --all-targets --features "$FEATURE_SET"
  echo
done

if [[ "$RUN_BENCHMARKS" == "1" ]]; then
  echo "Running benchmark harnesses"
  run cargo bench -p rapier-benchmarks-2d
  run cargo bench -p rapier-benchmarks-3d
fi

WASM_PACK_ARGS=(build "$WASM_CRATE_DIR" --target "$WASM_TARGET")
if [[ "$WASM_PROFILE" == "release" ]]; then
  WASM_PACK_ARGS+=(--release)
fi
if [[ -n "$WASM_OUT_DIR" ]]; then
  WASM_PACK_ARGS+=(--out-dir "$WASM_OUT_DIR")
fi

if [[ -n "$FEATURE_SET" ]]; then
  WASM_PACK_ARGS+=(-- --features "$FEATURE_SET")
fi

echo "Building wasm package"
run wasm-pack "${WASM_PACK_ARGS[@]}"
