import type {
  BenchmarkContext,
  BenchmarkResult,
  SimulationAdapter,
  SimulationConfig,
  SpawnedHandles,
  StepTelemetry,
  WorldAdapter,
} from '@rapier-avbd/viewer-shared';
import type * as RapierTypes from '@dimforge/rapier3d-compat';

let RAPIER: typeof RapierTypes | null = null;
let baseGravity = { x: 0, y: -9.81, z: 0 };

async function ensureRapier() {
  if (!RAPIER) {
    const module = await import('@dimforge/rapier3d-compat');
    await module.init();
    RAPIER = module;
  }
}

class ImpulseWorld implements WorldAdapter {
  private world: RapierTypes.World;
  private cubes: RapierTypes.RigidBody[] = [];
  private spheres: RapierTypes.RigidBody[] = [];
  private groundBody: RapierTypes.RigidBody | null = null;

  constructor(world: RapierTypes.World) {
    this.world = world;
  }

  step(): StepTelemetry {
    const start = performance.now();
    this.world.step();
    const total = performance.now() - start;
    return {
      solverMs: total,
      totalMs: total,
      iterations: undefined,
    };
  }

  setGravityScale(scale: number): void {
    if (!RAPIER) return;
    this.world.gravity = {
      x: baseGravity.x * scale,
      y: baseGravity.y * scale,
      z: baseGravity.z * scale,
    };
  }

  spawnBodies(count: number, config: SimulationConfig): SpawnedHandles {
    this.clearBodies();
    if (!RAPIER) throw new Error('Rapier not initialised');

    const { cubeHalfExtent, sphereRadius } = config.objectSizes;
    const { spread, startHeight } = config.spawn;

    if (!this.groundBody) {
      const groundDesc = RAPIER.RigidBodyDesc.fixed().setTranslation(0, 0, 0);
      this.groundBody = this.world.createRigidBody(groundDesc);
      const groundCollider = RAPIER.ColliderDesc.cuboid(spread, 0.5, spread);
      this.world.createCollider(groundCollider, this.groundBody);
    }

    const cubes: RapierTypes.RigidBody[] = [];
    const spheres: RapierTypes.RigidBody[] = [];

    for (let i = 0; i < count; i++) {
      const isCube = i % 2 === 0;
      const x = (Math.random() - 0.5) * spread;
      const z = (Math.random() - 0.5) * spread;
      const y = startHeight + (i / count) * startHeight * 0.25;
      const desc = RAPIER.RigidBodyDesc.dynamic().setTranslation(x, y, z);
      const body = this.world.createRigidBody(desc);

      if (isCube) {
        const collider = RAPIER.ColliderDesc.cuboid(cubeHalfExtent, cubeHalfExtent, cubeHalfExtent);
        this.world.createCollider(collider, body);
        cubes.push(body);
      } else {
        const collider = RAPIER.ColliderDesc.ball(sphereRadius);
        this.world.createCollider(collider, body);
        spheres.push(body);
      }
    }

    this.cubes = cubes;
    this.spheres = spheres;

    return {
      cubes,
      spheres,
    } as unknown as SpawnedHandles;
  }

  getBodyPositions(handles: SpawnedHandles, target: Float32Array): void {
    let cursor = 0;

    const writeFromBody = (body: RapierTypes.RigidBody | undefined) => {
      if (!body) return;
      const translation = body.translation();
      target[cursor++] = translation.x;
      target[cursor++] = translation.y;
      target[cursor++] = translation.z;
    };

    (handles.cubes as RapierTypes.RigidBody[]).forEach(writeFromBody);
    (handles.spheres as RapierTypes.RigidBody[]).forEach(writeFromBody);
  }

  clearBodies(): void {
    if (!RAPIER) return;
    const toRemove = [...this.cubes, ...this.spheres];
    for (const body of toRemove) {
      this.world.removeRigidBody(body);
    }
    this.cubes = [];
    this.spheres = [];
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
      solverId: 'impulse',
      solverLabel: 'Impulse',
      solverMs,
      totalMs,
      steps: ctx.scenario.steps,
      bodies: spawnCount,
      timestamp: new Date().toISOString(),
    };
  }

  applyAvbdParams(): void {
    // Impulse solver does not expose AVBD parameters.
  }

  dispose(): void {
    this.clearBodies();
  }
}

const impulseAdapter: SimulationAdapter = {
  solverId: 'impulse',
  solverLabel: 'Impulse Solver',
  supportsAvbdParams: false,
  async init() {
    await ensureRapier();
  },
  async createWorld(initialConfig: SimulationConfig): Promise<WorldAdapter> {
    await ensureRapier();
    if (!RAPIER) throw new Error('Rapier not available');
    baseGravity = { x: 0, y: -9.81, z: 0 };
    const world = new RAPIER.World(baseGravity);
    world.gravity = {
      x: baseGravity.x * initialConfig.gravityScale,
      y: baseGravity.y * initialConfig.gravityScale,
      z: baseGravity.z * initialConfig.gravityScale,
    };
    return new ImpulseWorld(world);
  },
};

export default impulseAdapter;
