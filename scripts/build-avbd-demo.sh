#!/usr/bin/env bash

set -euo pipefail

if ! command -v wasm-pack >/dev/null 2>&1; then
  echo "wasm-pack is required to build the AVBD demo package" >&2
  exit 1
fi

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WASM_DIR="$ROOT_DIR/rapier-wasm-avbd"
OUT_DIR="$WASM_DIR/pkg"
ASSETS_DIR="$ROOT_DIR/assets/pkg"

if [[ ! -d "$WASM_DIR" ]]; then
  echo "Expected wasm crate directory at '$WASM_DIR'" >&2
  exit 1
fi

echo "Building rapier-wasm-avbd with AVBD solver"
wasm-pack build "$WASM_DIR" --release --target web --out-dir pkg -- --features solver_avbd

echo "Preparing assets/pkg for local demo imports"
rm -rf "$ASSETS_DIR"
mkdir -p "$ASSETS_DIR"
cp -R "$OUT_DIR"/* "$ASSETS_DIR"/

cat <<'EOM'
Local AVBD WebAssembly bundle ready.

Update assets/avbd_three_demo.html by uncommenting the local import:

  // import initRapier, { ... } from '../rapier-wasm-avbd/pkg/rapier_wasm_avbd.js';

and replace it with:

  import initRapier, { ... } from './pkg/rapier_wasm_avbd.js';

Open the HTML file in a static server (e.g. `python -m http.server`) so the browser can load the WASM module.
EOM
