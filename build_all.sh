#!/usr/bin/env bash

set -euo pipefail

if ! command -v cargo >/dev/null 2>&1; then
  echo "cargo is required to build the Rust crates" >&2
  exit 1
fi

if ! command -v wasm-pack >/dev/null 2>&1; then
  echo "wasm-pack is required to build the WebAssembly bindings" >&2
  exit 1
fi

if ! command -v npm >/dev/null 2>&1; then
  echo "npm is required to install and run the JavaScript testbed" >&2
  exit 1
fi

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUST_FEATURES=${RAPIER_FEATURES:-solver_avbd}
RUST_PROFILE=${RUST_PROFILE:-release}
CARGO_FLAGS=()
if [[ "$RUST_PROFILE" == "release" ]]; then
  CARGO_FLAGS+=(--release)
elif [[ -n "$RUST_PROFILE" ]]; then
  CARGO_FLAGS+=(--profile "$RUST_PROFILE")
fi

RUST_CRATES=(rapier2d rapier3d)
WASM_PACKAGES=(rapier2d rapier3d)
JS_REPO_DIR=${JS_REPO_DIR:-rapier-wasm-avbd}
WASM_TARGET=${WASM_TARGET:-bundler}
WASM_PROFILE=${WASM_PROFILE:-release}
WASM_OUT_DIR=${WASM_OUT_DIR:-pkg}
RUN_TESTBED=${RUN_TESTBED:-1}
TESTBED_DIMENSION=${TESTBED_DIMENSION:-3d}

run() {
  echo "+ $*"
  "$@"
}

if [[ ! -d "$JS_REPO_DIR" ]]; then
  echo "JavaScript bindings repository '$JS_REPO_DIR' was not found." >&2
  echo "Clone https://github.com/dimforge/rapier.js next to this repository or point JS_REPO_DIR to it." >&2
  exit 1
fi

pushd "$ROOT_DIR" >/dev/null

for crate in "${RUST_CRATES[@]}"; do
  echo "Building $crate with features '$RUST_FEATURES'"
  run cargo build -p "$crate" "${CARGO_FLAGS[@]}" --features "$RUST_FEATURES"
  echo
done

for package in "${WASM_PACKAGES[@]}"; do
  PACKAGE_DIR="$JS_REPO_DIR/$package"
  if [[ ! -d "$PACKAGE_DIR" ]]; then
    echo "Skipping WebAssembly build for '$package' (directory '$PACKAGE_DIR' not found)." >&2
    continue
  fi

  pushd "$PACKAGE_DIR" >/dev/null
  run npm install

  WASM_ARGS=(build --target "$WASM_TARGET")
  if [[ "$WASM_PROFILE" == "release" ]]; then
    WASM_ARGS+=(--release)
  fi
  if [[ -n "$WASM_OUT_DIR" ]]; then
    WASM_ARGS+=(--out-dir "$WASM_OUT_DIR")
  fi

  if [[ -n "$RUST_FEATURES" ]]; then
    WASM_ARGS+=(-- --features "$RUST_FEATURES")
  fi

  echo "Building WebAssembly bindings for $package"
  run wasm-pack "${WASM_ARGS[@]}"
  popd >/dev/null
  echo

done

if [[ "$RUN_TESTBED" == "1" ]]; then
  TESTBED_DIR_KEY="testbed${TESTBED_DIMENSION}"
  PACKAGE_KEY="rapier${TESTBED_DIMENSION}"
  TESTBED_DIR="$JS_REPO_DIR/$TESTBED_DIR_KEY"
  PACKAGE_DIR="$JS_REPO_DIR/$PACKAGE_KEY"

  if [[ ! -d "$TESTBED_DIR" ]]; then
    echo "Testbed directory '$TESTBED_DIR' not found; skipping testbed startup." >&2
  elif [[ ! -d "$PACKAGE_DIR" ]]; then
    echo "Package directory '$PACKAGE_DIR' not found; skipping testbed startup." >&2
  else
    pushd "$TESTBED_DIR" >/dev/null
    run npm install
    if [[ -d "$PACKAGE_DIR/pkg" ]]; then
      run npm link "$PACKAGE_DIR/pkg"
    else
      echo "Local package output '$PACKAGE_DIR/pkg' not found; skipping npm link." >&2
    fi
    echo "Starting npm testbed (Ctrl+C to stop)"
    run npm run start
    popd >/dev/null
  fi
fi

popd >/dev/null
