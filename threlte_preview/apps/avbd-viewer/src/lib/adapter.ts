import type {
  BenchmarkContext,
  BenchmarkResult,
  SimulationAdapter,
  SimulationConfig,
  SpawnedHandles,
  StepTelemetry,
  WorldAdapter,
} from '@rapier-avbd/viewer-shared';
import initRapier, { RapierWorld, StepMetrics, init_panic_hook } from '@validvector/rapier3d-avbd';
import type { AvbdParams } from '@rapier-avbd/viewer-shared';

let wasmReady = false;

async function ensureWasm() {
  if (!wasmReady) {
    await initRapier();
    init_panic_hook?.();
    wasmReady = true;
  }
}

class AvbdWorld implements WorldAdapter {
  private world: RapierWorld;
  private handles: SpawnedHandles = { cubes: [], spheres: [] };

  constructor(world: RapierWorld) {
    this.world = world;
  }

  step(): StepTelemetry {
    const metrics: StepMetrics = this.world.step_with_metrics();
    return {
      solverMs: metrics.solver_ms(),
      totalMs: metrics.total_ms(),
      iterations: metrics.iterations(),
    };
  }

  setGravityScale(scale: number): void {
    this.world.set_gravity_scale(scale);
  }

  spawnBodies(count: number, config: SimulationConfig): SpawnedHandles {
    this.clearBodies();
    const { cubeHalfExtent, sphereRadius } = config.objectSizes;
    const { spread, startHeight } = config.spawn;

    // Preserve ground if it already exists; otherwise create one.
    if (this.handles.cubes.length === 0 && this.handles.spheres.length === 0) {
      const ground = this.world.create_fixed_body(0, 0, 0);
      this.world.create_cuboid_collider(ground, spread, 0.5, spread);
    }

    const newCubes: number[] = [];
    const newSpheres: number[] = [];
    for (let i = 0; i < count; i++) {
      const phase = i % 2 === 0 ? 'cube' : 'sphere';
      const x = (Math.random() - 0.5) * spread;
      const z = (Math.random() - 0.5) * spread;
      const y = startHeight + (i / count) * startHeight * 0.25;
      const handle = this.world.create_dynamic_body(x, y, z);

      if (phase === 'cube') {
        this.world.create_cuboid_collider(handle, cubeHalfExtent, cubeHalfExtent, cubeHalfExtent);
        newCubes.push(handle);
      } else {
        this.world.create_ball_collider(handle, sphereRadius);
        newSpheres.push(handle);
      }
    }

    this.handles = { cubes: newCubes, spheres: newSpheres };
    return this.handles;
  }

  getBodyPositions(handles: SpawnedHandles, target: Float32Array): void {
    let cursor = 0;
    const writePosition = (handle: number) => {
      const [x, y, z] = this.world.get_body_translation(handle);
      target[cursor++] = x;
      target[cursor++] = y;
      target[cursor++] = z;
    };

    handles.cubes.forEach(writePosition);
    handles.spheres.forEach(writePosition);
  }

  clearBodies(): void {
    this.world.clear_dynamic_bodies();
    this.handles = { cubes: [], spheres: [] };
  }

  async runBenchmark(ctx: BenchmarkContext): Promise<BenchmarkResult> {
    const spawnCount = Math.floor(ctx.config.bodyCount * ctx.scenario.spawnMultiplier);
    this.spawnBodies(spawnCount, ctx.config);

    let solverMs = 0;
    let totalMs = 0;
    for (let i = 0; i < ctx.scenario.steps; i++) {
      const metrics = this.step();
      solverMs += metrics.solverMs;
      totalMs += metrics.totalMs;
    }

    return {
      scenarioId: ctx.scenario.id,
      scenarioLabel: ctx.scenario.label,
      solverId: 'avbd',
      solverLabel: 'AVBD',
      solverMs,
      totalMs,
      steps: ctx.scenario.steps,
      bodies: spawnCount,
      timestamp: new Date().toISOString(),
    };
  }

  applyAvbdParams(params: AvbdParams): void {
    this.world.set_avbd_params(
      params.iterations,
      params.alpha,
      params.beta,
      params.gamma,
      params.stiffnessMin,
      params.stiffnessMax,
      params.regularization
    );
  }

  dispose(): void {
    // wasm-bindgen will handle memory cleanup when references are dropped.
  }
}

const avbdAdapter: SimulationAdapter = {
  solverId: 'avbd',
  solverLabel: 'AVBD Solver',
  supportsAvbdParams: true,
  async init() {
    await ensureWasm();
  },
  async createWorld(initialConfig: SimulationConfig): Promise<WorldAdapter> {
    await ensureWasm();
    const world = new RapierWorld(0, -9.81, 0, true);
    world.set_gravity_scale(initialConfig.gravityScale);
    if (initialConfig.avbdParams) {
      world.set_avbd_params(
        initialConfig.avbdParams.iterations,
        initialConfig.avbdParams.alpha,
        initialConfig.avbdParams.beta,
        initialConfig.avbdParams.gamma,
        initialConfig.avbdParams.stiffnessMin,
        initialConfig.avbdParams.stiffnessMax,
        initialConfig.avbdParams.regularization
      );
    }
    return new AvbdWorld(world);
  },
};

export default avbdAdapter;
