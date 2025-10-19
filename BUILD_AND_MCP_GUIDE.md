# Build Pipeline and MCP Server Guide for Rapier AVBD

## Overview

This guide explains the **AVBD (Augmented Vertex Block Descent)** build pipeline and how to use the **MCP (Model Context Protocol) server** to run automated benchmarks and builds.

---

## 1. Build Pipeline Architecture

### Dual-Solver System

Rapier AVBD implements two physics solvers that can be selected via Cargo features:

- **`solver_impulse`** - Traditional sequential impulse (PGS) solver (default, mature)
- **`solver_avbd`** - New AVBD solver (experimental, better convergence)

Both solvers:
- Share the same Rapier API surface
- Are **mutually exclusive** (only one can be enabled at a time)
- Support the same platforms (native, WASM)
- Are compatible with the same integration parameters

### Build Outputs

The pipeline produces three types of artifacts:

1. **Native Rust builds** - For local testing and benchmarking
2. **WASM packages** - For web deployment (JavaScript/TypeScript)
3. **Benchmark results** - Performance metrics in `benchmarks3d/scoreboard.json`

---

## 2. Quick Start - MCP Server Setup

### Prerequisites

1. **Rust toolchain** (via [rustup](https://rustup.rs/))
2. **wasm-pack** - For WASM builds:
   ```bash
   cargo install wasm-pack
   ```
3. **Node.js 18+** and **pnpm**
4. **Project dependencies**:
   ```bash
   cd /Users/fogi/Documents/workspaces/validvector/rapier-avbd
   pnpm install
   ```

### MCP Server Commands

The MCP server provides three main tools accessible via CLI or MCP protocol:

#### 1. Build WASM Package

```bash
# Build AVBD solver for 3D (default)
node tools/mcp/server.js build

# Build impulse solver for 3D
node tools/mcp/server.js build --solver impulse

# Build AVBD solver for 2D
node tools/mcp/server.js build --solver avbd --dim 2

# Development (debug) build
node tools/mcp/server.js build --solver avbd --dev
```

**What it does:**
- Creates a WASM package in `rapier-wasm-{solver}/pkg/`
- Generates TypeScript bindings
- Optimizes for size in release mode

#### 2. Run Benchmark

```bash
# Benchmark AVBD solver for 60 steps (default is 120)
node tools/mcp/server.js bench --solver avbd --steps 60

# Benchmark impulse solver for 180 steps
node tools/mcp/server.js bench --solver impulse --steps 180
```

**What it does:**
- Runs the `solver-bench` harness (stack drop scenario)
- Records metrics to `benchmarks3d/scoreboard.json`
- Copies binary and metrics to `benchmarks3d/artifacts/{timestamp}-{solver}/`
- Outputs JSON results

**Sample output:**
```json
{
  "solver": "avbd",
  "steps": 60,
  "stack_height": 20,
  "dt": 0.016666668,
  "scenario": "stack_drop",
  "timestamp_ms": 1760883122920,
  "totals": {
    "total_ms": 0.707,
    "solver_ms": 0.177,
    "assembly_ms": 0.005,
    "resolution_ms": 0,
    "writeback_ms": 0.002
  },
  "averages": {
    "total_ms": 0.0118,
    "solver_ms": 0.0030,
    "assembly_ms": 0.0001,
    "resolution_ms": 0,
    "writeback_ms": 0.0000
  },
  "constraints": 0,
  "contacts": 0,
  "body_count": 21,
  "commit": "2a8dbbbaa26fda7dfb3195a2f27a6975fdda27c5",
  "artifactPath": "benchmarks3d/artifacts/1760883122920-avbd"
}
```

#### 3. Get Scoreboard

```bash
# View all benchmark results
cat benchmarks3d/scoreboard.json | jq
```

Or programmatically through pnpm:
```bash
pnpm bench:avbd      # Run AVBD benchmark
pnpm bench:impulse   # Run impulse benchmark
```

---

## 3. MCP Server Integration

### Start MCP Server

The MCP server can be run in **stdio mode** for integration with tools like Claude Desktop or other MCP clients:

```bash
pnpm run mcp:server
# or
node tools/mcp/server.js serve
```

This starts the server listening on stdio using the MCP protocol.

### MCP Tools Available

Once connected, the following tools are available:

1. **`build-wasm`**
   - Parameters: `solver` (avbd|impulse), `dim` (2|3), `release` (boolean)
   - Returns: Build output text
   
2. **`run-bench`**
   - Parameters: `solver` (avbd|impulse), `steps` (integer)
   - Returns: Benchmark metrics JSON
   
3. **`get-scoreboard`**
   - Parameters: None
   - Returns: Full scoreboard history JSON

### Example MCP Integration

For Claude Desktop, add to your MCP config (`~/Library/Application Support/Claude/claude_desktop_config.json`):

```json
{
  "mcpServers": {
    "rapier-avbd": {
      "command": "node",
      "args": ["/Users/fogi/Documents/workspaces/validvector/rapier-avbd/tools/mcp/server.js", "serve"],
      "cwd": "/Users/fogi/Documents/workspaces/validvector/rapier-avbd"
    }
  }
}
```

---

## 4. Build Pipeline Details

### Native Rust Builds

Build the solver-bench harness for benchmarking:

```bash
# Build with impulse solver
cargo build --release -p solver-bench \
  --no-default-features \
  --features solver_impulse,profiler

# Build with AVBD solver
cargo build --release -p solver-bench \
  --no-default-features \
  --features solver_avbd,profiler

# Run directly
./target/release/solver-bench --solver avbd --steps 120
```

### WASM Builds

The `build_wasm_quick.sh` script handles WASM packaging:

```bash
# 3D AVBD (default)
./build_wasm_quick.sh 3 --solver avbd --release

# 2D impulse
./build_wasm_quick.sh 2 --solver impulse --dev

# Help
./build_wasm_quick.sh --help
```

**Script does:**
1. Creates `rapier-wasm-{solver}` directory
2. Generates `Cargo.toml` with correct feature flags
3. Generates `lib.rs` with WASM bindings
4. Runs `wasm-pack build --target web`
5. Creates TypeScript definitions

**Output:** `rapier-wasm-{solver}/pkg/` with:
- `rapier_wasm_{solver}.js` - JavaScript module
- `rapier_wasm_{solver}_bg.wasm` - WASM binary
- `rapier_wasm_{solver}.d.ts` - TypeScript definitions

### Feature Flags

Key features in `crates/rapier3d/Cargo.toml`:

- `solver_impulse` - Enable impulse solver (default)
- `solver_avbd` - Enable AVBD solver
- `dim3` / `dim2` - Dimension
- `f32` / `f64` - Floating point precision
- `parallel` - Enable Rayon parallelization (AVBD only, gated)
- `profiler` - Enable performance instrumentation
- `enhanced-determinism` - Stricter floating-point determinism

---

## 5. Benchmark Scenarios

### Current Scenario: Stack Drop

The `solver-bench` harness implements a **stack drop** scenario:

- **Bodies:** 21 rigid bodies (20 in a vertical stack + 1 ground plane)
- **Simulation:** Bodies drop under gravity and settle
- **Duration:** Configurable steps (default 120 @ 60Hz)
- **Metrics:**
  - Total time per step
  - Solver time (constraint solving)
  - Assembly time (constraint graph building)
  - Resolution time (contact/joint resolution)
  - Writeback time (applying impulses to bodies)

### Benchmark Interpretation

**AVBD typically shows:**
- Lower solver time (faster convergence)
- Comparable assembly time
- Better stability with stiff constraints

**Impulse typically shows:**
- Higher solver time (more iterations needed)
- Mature, well-optimized code paths
- Predictable behavior

**Zero constraints/contacts:**
- This is expected during free-fall before bodies collide
- Once bodies make contact, constraint counts increase

---

## 6. Directory Structure

```
rapier-avbd/
├── benchmarks3d/
│   ├── scoreboard.json           # All benchmark results
│   └── artifacts/                # Timestamped benchmark binaries
│       ├── {timestamp}-avbd/
│       │   ├── solver-bench      # Binary snapshot
│       │   └── metrics.json      # Metrics snapshot
│       └── {timestamp}-impulse/
├── crates/
│   └── rapier3d/                 # Core physics engine
│       ├── Cargo.toml            # Feature definitions
│       └── src/
│           └── dynamics/
│               └── solver/
│                   ├── avbd/     # AVBD implementation
│                   └── impulse/  # Impulse implementation
├── tools/
│   ├── mcp/
│   │   └── server.js            # MCP server implementation
│   └── solver-bench/
│       ├── Cargo.toml
│       └── src/
│           └── main.rs          # Benchmark harness
├── rapier-wasm-avbd/            # Generated WASM package (AVBD)
│   └── pkg/                     # Published package
└── rapier-wasm-impulse/         # Generated WASM package (impulse)
    └── pkg/                     # Published package
```

---

## 7. Workflow Examples

### Compare Solvers

```bash
# Build both solvers
cargo build --release -p solver-bench --no-default-features --features solver_impulse,profiler
cargo build --release -p solver-bench --no-default-features --features solver_avbd,profiler

# Run benchmarks
node tools/mcp/server.js bench --solver impulse --steps 120
node tools/mcp/server.js bench --solver avbd --steps 120

# View results
cat benchmarks3d/scoreboard.json | jq '.[] | {solver, total_ms: .totals.total_ms}'
```

### Build WASM for Web

```bash
# Build AVBD WASM
node tools/mcp/server.js build --solver avbd --dim 3 --release

# Test in browser
cd rapier-wasm-avbd/pkg
python3 -m http.server 8000
# Open test_wasm_avbd.html in browser
```

### CI/CD Integration

```bash
#!/bin/bash
# Example CI pipeline

# 1. Build both solvers
pnpm run build:wasm:avbd
pnpm run build:wasm:impulse

# 2. Run benchmarks
pnpm run bench:avbd
pnpm run bench:impulse

# 3. Compare results (fail if AVBD regresses)
# ... add comparison script ...
```

---

## 8. Troubleshooting

### Build Failures

**Error: "Can't have both solver_impulse and solver_avbd"**
- Solution: Use `--no-default-features` and specify only one solver

**Error: "wasm-pack not found"**
- Solution: `cargo install wasm-pack`

### Benchmark Issues

**Zero constraints/contacts in results**
- This is normal during initial free-fall
- Increase `--steps` to see post-collision behavior

**Performance variance**
- Run multiple times and average
- Disable background processes
- Use `--release` builds only

### MCP Server Issues

**Module not found errors**
- Run `pnpm install` in project root
- Ensure Node.js 18+ is installed

**JSON parsing errors**
- Check `solver-bench` output format
- Ensure latest version of server.js

---

## 9. Next Steps

### Planned Enhancements

1. **More scenarios** - Add joint chains, high mass ratios, complex contacts
2. **Parallel AVBD** - Enable Rayon-based parallelization once determinism is validated
3. **GPU offload** - Explore WGPU/OpenCL integration per Houdini digest
4. **Web benchmarking** - Add browser-based benchmark suite
5. **CI integration** - Automated regression testing

### Contributing

See `ROADMAP.md` and `BUILD_PIPELINE_PLAN.md` for development priorities.

---

## 10. Summary

**To run a benchmark yourself:**
```bash
pnpm bench:avbd
pnpm bench:impulse
cat benchmarks3d/scoreboard.json | jq
```

**To enable MCP integration:**
- Configure your MCP client to run `node tools/mcp/server.js serve`
- Use tools: `build-wasm`, `run-bench`, `get-scoreboard`

**Key files:**
- MCP server: `tools/mcp/server.js`
- Benchmark harness: `tools/solver-bench/src/main.rs`
- Build script: `build_wasm_quick.sh`
- Scoreboard: `benchmarks3d/scoreboard.json`

For more details, see:
- `BUILD_PIPELINE_PLAN.md` - Overall architecture
- `BENCHMARK_README.md` - WASM browser benchmarks
- `build_js_with_avbd.md` - JavaScript integration
- `MCP_SETUP.md` - MCP server setup
