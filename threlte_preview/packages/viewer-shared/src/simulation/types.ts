import type { Writable } from "svelte/store";

export interface StepTelemetry {
  solverMs: number;
  totalMs: number;
  iterations?: number;
}

export interface ObjectSizes {
  cubeHalfExtent: number;
  sphereRadius: number;
}

export interface SpawnSettings {
  spread: number;
  startHeight: number;
}

export interface AvbdParams {
  iterations: number;
  alpha: number;
  beta: number;
  gamma: number;
  stiffnessMin: number;
  stiffnessMax: number;
  regularization: number;
}

export interface SimulationConfig {
  bodyCount: number;
  gravityScale: number;
  objectSizes: ObjectSizes;
  spawn: SpawnSettings;
  avbdParams?: AvbdParams;
}

export interface BenchmarkScenario {
  id: string;
  label: string;
  description: string;
  steps: number;
  spawnMultiplier: number;
}

export interface BenchmarkResult {
  scenarioId: string;
  scenarioLabel: string;
  solverId: string;
  solverLabel: string;
  solverMs: number;
  totalMs: number;
  steps: number;
  bodies: number;
  timestamp: string;
}

export interface SpawnedHandles {
  cubes: unknown[];
  spheres: unknown[];
}

export interface BenchmarkContext {
  config: SimulationConfig;
  scenario: BenchmarkScenario;
}

export interface WorldAdapter {
  step(): StepTelemetry;
  setGravityScale(scale: number): void;
  spawnBodies(count: number, config: SimulationConfig): SpawnedHandles;
  getBodyPositions(handles: SpawnedHandles, target: Float32Array): void;
  clearBodies(): void;
  runBenchmark(ctx: BenchmarkContext): Promise<BenchmarkResult>;
  applyAvbdParams?(params: AvbdParams): void;
  dispose?(): void;
}

export interface SimulationAdapter {
  solverId: string;
  solverLabel: string;
  supportsAvbdParams: boolean;
  init(): Promise<void>;
  createWorld(initialConfig: SimulationConfig): Promise<WorldAdapter>;
}

export interface SimulationState {
  config: SimulationConfig;
  regenToken: number;
  liveMetrics: StepTelemetry;
  benchmarkHistory: BenchmarkResult[];
  benchmarkRunning: boolean;
  activeScenario?: string;
}

export interface SimulationStores {
  state: Writable<SimulationState>;
  requestRespawn: () => void;
  pushBenchmark: (result: BenchmarkResult) => void;
  setBenchmarkRunning: (running: boolean, scenarioId?: string) => void;
  updateConfig: (partial: Partial<SimulationConfig>) => void;
  updateAvbdParams: (partial: Partial<AvbdParams>) => void;
}
