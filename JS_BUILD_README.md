# Building Rapier JavaScript Package with AVBD Solver

This guide shows you how to build JavaScript/WebAssembly bindings for your Rapier fork with the AVBD (Augmented Vertex Block Descent) solver enabled.

## Quick Start (Recommended)

The fastest way to build and test:

```bash
# Build for 3D (default)
./build_wasm_quick.sh

# Or build for 2D
./build_wasm_quick.sh 2
```

This will:
1. Create a `rapier-wasm-avbd/` directory with necessary files
2. Build the WASM module with `solver_avbd` feature enabled
3. Generate JavaScript bindings in `rapier-wasm-avbd/pkg/`

## Testing Your Build

After building, you can test it with the included HTML test page:

```bash
# Serve the test page with a local server (required for WASM)
python3 -m http.server 8000

# Or using Node.js
npx http-server -p 8000
```

Then open http://localhost:8000/test_wasm_avbd.html in your browser.

The test page will:
- ✅ Verify AVBD solver is available
- ✅ Show which solver backend is active
- ✅ Let you run simulation steps
- ✅ Display performance metrics

## What Gets Built

The build creates:
- `rapier-wasm-avbd/pkg/rapier_wasm_avbd.js` - JavaScript bindings
- `rapier-wasm-avbd/pkg/rapier_wasm_avbd_bg.wasm` - WASM module
- `rapier-wasm-avbd/pkg/rapier_wasm_avbd.d.ts` - TypeScript definitions

## Using in Your Project

### Browser (ES6 Modules)

```javascript
import init, { RapierWorld, is_avbd_available } from './rapier-wasm-avbd/pkg/rapier_wasm_avbd.js';

async function run() {
    // Initialize the WASM module
    await init();
    
    // Check if AVBD is available
    console.log('AVBD available:', is_avbd_available());
    
    // Create world with AVBD solver (last parameter = true)
    const world = new RapierWorld(0.0, -9.81, 0.0, true);
    
    console.log('Using solver:', world.get_solver_backend());
    
    // Run simulation
    world.step();
}

run();
```

### Node.js

```javascript
const { RapierWorld, is_avbd_available } = require('./rapier-wasm-avbd/pkg/rapier_wasm_avbd');

// Create world
const world = new RapierWorld(0.0, -9.81, 0.0, true);
world.step();
```

## Full rapier.js Integration (Advanced)

For a complete implementation matching the official rapier.js API:

1. Clone rapier.js:
```bash
cd ..
git clone https://github.com/dimforge/rapier.js.git rapier-avbd-js
cd rapier-avbd-js
```

2. Modify `builds/prepare_builds/templates/Cargo.toml.tera` to use your fork:

```toml
[dependencies.rapier{{ dimension }}d]
path = "/Users/fogi/Documents/workspaces/validvector/rapier-avbd/crates/rapier{{ dimension }}d"
features = [{% for feature in additional_features %}"{{ feature }}", {% endfor %}"solver_avbd"]
```

3. Generate and build:
```bash
cd builds/prepare_builds
cargo run -- -d dim3 -f non-deterministic
cd ../rapier3d
./build_rust.sh
./build_typescript.sh
```

## Verifying AVBD is Enabled

The build script creates code that lets you verify AVBD is working:

```javascript
// Check availability
console.log(is_avbd_available());  // Should print: true

// Check active solver
const world = new RapierWorld(0.0, -9.81, 0.0, true);
console.log(world.get_solver_backend());  // Should print: "Avbd"
```

## Common Issues

### "AVBD not available" despite building with feature
- Make sure `solver_avbd` is in the features list in Cargo.toml
- Clean build: `cd rapier-wasm-avbd && cargo clean && cd .. && ./build_wasm_quick.sh`

### WASM doesn't load in browser
- Use a local server (not `file://` protocol)
- Check browser console for CORS errors
- Verify the WASM file path is correct

### Build fails with linking errors
- Ensure all dependencies in rapier2d/rapier3d Cargo.toml are available
- Check that the `solver_avbd` feature compiles: `cd crates/rapier3d && cargo check --features solver_avbd`

## Next Steps

1. **Test the quick build** to verify AVBD works
2. **Add more API wrappers** in `rapier-wasm-avbd/src/lib.rs` as needed
3. **Integrate with your application** using the generated JavaScript module
4. **Consider full rapier.js integration** for complete TypeScript API

## Files Created

- `build_wasm_quick.sh` - Automated build script
- `build_js_with_avbd.md` - Detailed documentation
- `test_wasm_avbd.html` - Interactive test page
- `rapier-wasm-avbd/` - WASM package directory (created on first build)

## Additional Resources

- [wasm-pack documentation](https://rustwasm.github.io/wasm-pack/)
- [Official rapier.js repository](https://github.com/dimforge/rapier.js)
- [Rapier physics engine](https://rapier.rs/)
- AVBD paper digest: `llm_digests/avbd_paper_pdf_digest.txt`
