#!/bin/bash

set -e

tmp=$(mktemp -d)

echo "$tmp"

PUBLISH_ARGS=()
if [[ -n "$DRY_RUN" ]]; then
    PUBLISH_ARGS+=("$DRY_RUN")
fi

if [[ -n "$RAPIER_FEATURES" ]]; then
    PUBLISH_ARGS+=(--features "$RAPIER_FEATURES")
fi

cp -r src "$tmp"/.
cp -r LICENSE README.md "$tmp"/.

publish_crate() {
    local manifest="$1"
    sed 's#\.\./\.\./src#src#g' "$manifest" > "$tmp"/Cargo.toml
    cp -r LICENSE README.md "$tmp"/.
    (cd "$tmp" && cargo publish "${PUBLISH_ARGS[@]}")
}

echo "Publishing rapier2d"
publish_crate "crates/rapier2d/Cargo.toml"

echo "Publishing rapier3d"
publish_crate "crates/rapier3d/Cargo.toml"

echo "Publishing rapier2d-f64"
publish_crate "crates/rapier2d-f64/Cargo.toml"

echo "Publishing rapier3d-f64"
publish_crate "crates/rapier3d-f64/Cargo.toml"

rm -rf "$tmp"
