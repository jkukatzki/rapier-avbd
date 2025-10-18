# AVBD vs Impulse Solver Benchmark

## Overview

This benchmark compares the performance and behavior of Rapier's two solver backends:
- **AVBD (Augmented Vertex Block Descent)** - Experimental solver with improved convergence
- **Impulse (PGS)** - Classic sequential impulse / Projected Gauss-Seidel solver

## Benchmark Setup

The benchmark creates two identical physics worlds with:
- **100 rigid bodies** (50 balls + 50 boxes)
- **100 colliders** (one per body)
- **1 static ground plane**
- **100 simulation steps** per test

Both worlds have identical:
- Gravity (-9.81 m/s² on Y axis)
- Body positions (randomized but same seed)
- Collider shapes and sizes
- Integration parameters

## How to Run

1. Make sure the server is running:
   ```bash
   python3 -m http.server 8000
   ```

2. Open in browser: http://localhost:8000/test_wasm_avbd.html

3. Click the **🏁 Run Benchmark (100 Bodies)** button

## What Gets Measured

### Performance Metrics
- **Total time**: Total milliseconds for 100 simulation steps
- **Average per step**: Time per individual step in milliseconds
- **Steps per second**: Simulation frequency (Hz)
- **Relative performance**: Percentage faster/slower

### Correctness Checks
- Body counts match
- Collider counts match
- Final positions are logged for comparison
- Both solvers should produce physically plausible results

## Expected Results

### AVBD Solver Characteristics
- **Better for**: Stiff constraints, high mass ratios, complex joint setups
- **Convergence**: Generally faster constraint convergence
- **Stability**: More stable with difficult configurations
- **Parallelization**: More parallel-friendly architecture

### Impulse Solver Characteristics
- **Better for**: Simple scenarios, fewer constraints
- **Maturity**: Battle-tested, well-optimized
- **Predictability**: Established behavior patterns
- **Performance**: May be faster for simple scenes

## Interpreting Results

### Performance
The relative performance depends on:
- Scene complexity (constraint count, mass ratios)
- Hardware (CPU, memory bandwidth)
- WASM optimization level
- Browser JIT compiler

**Note**: In WASM builds, the performance difference may be smaller than in native builds due to the overhead of the WASM boundary and browser optimizations.

### Accuracy
Both solvers should produce stable, realistic physics. Compare:
- Final body positions (should be similar but not identical)
- Constraint satisfaction (no excessive penetration or drift)
- Energy conservation (bodies should settle properly)

## Understanding the Output

The benchmark logs show:
```
🏁 Starting benchmark with 100 bodies and 100 colliders...
════════════════════════════════════════════════════════════
AVBD: Created 101 bodies, 101 colliders
Impulse: Created 101 bodies, 101 colliders
════════════════════════════════════════════════════════════
📊 BENCHMARK RESULTS:
════════════════════════════════════════════════════════════
🔷 AVBD Solver:
   Total time: XXX.XXms
   Average per step: X.XXXms
   Steps per second: XXX Hz

🔶 Impulse Solver:
   Total time: XXX.XXms
   Average per step: X.XXXms
   Steps per second: XXX Hz
════════════════════════════════════════════════════════════
✨ AVBD is X.X% FASTER than Impulse
(or: ⚠️ AVBD is X.X% slower than Impulse)
════════════════════════════════════════════════════════════
```

## Advanced Testing

You can also:
1. **Run Simple Test** - Creates 5 bodies for quick testing
2. **Step Once** - Steps both worlds simultaneously for debugging
3. **Clear Output** - Clear the log to start fresh

## Implementation Details

Both worlds are:
- Created from the same WASM module
- Run in the same JavaScript thread
- Measured using `performance.now()` for high precision
- Benchmarked sequentially (AVBD first, then Impulse) to avoid interference

## Limitations

- **Single-threaded**: No parallel execution in JavaScript
- **No rendering**: Pure simulation benchmark (no visualization overhead)
- **Fixed scene**: Always the same 100-body setup
- **Cold start**: First run may include JIT compilation effects

## Next Steps

To extend this benchmark:
1. Add more complex scenarios (joints, ropes, chains)
2. Test with different body counts
3. Add stress tests (1000+ bodies)
4. Compare accuracy metrics (constraint error, energy drift)
5. Implement visual rendering to see the differences

## Source Code

- Build script: `build_wasm_quick.sh`
- Test page: `test_wasm_avbd.html`
- WASM source: `rapier-wasm-avbd/src/lib.rs` (generated)
- Rust implementation: `crates/rapier3d/` (with `solver_avbd` feature)
