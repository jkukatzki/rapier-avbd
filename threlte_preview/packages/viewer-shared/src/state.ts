import { writable } from "svelte/store";
import type {
  AvbdParams,
  SimulationConfig,
  SimulationState,
  SimulationStores,
  StepTelemetry,
} from "./simulation/types";

const DEFAULT_AVBD_PARAMS: AvbdParams = {
  iterations: 6,
  alpha: 0.95,
  beta: 12,
  gamma: 0.98,
  stiffnessMin: 1,
  stiffnessMax: 1_000_000,
  regularization: 0.000_001,
};

const DEFAULT_METRICS: StepTelemetry = {
  solverMs: 0,
  totalMs: 0,
  iterations: 0,
};

const DEFAULT_CONFIG: SimulationConfig = {
  bodyCount: 120,
  gravityScale: 1,
  objectSizes: {
    cubeHalfExtent: 0.5,
    sphereRadius: 0.5,
  },
  spawn: {
    spread: 10,
    startHeight: 6,
  },
  avbdParams: DEFAULT_AVBD_PARAMS,
};

export function createSimulationStores(initialConfig?: Partial<SimulationConfig>): SimulationStores {
  const state = writable<SimulationState>({
    config: {
      ...DEFAULT_CONFIG,
      ...initialConfig,
      avbdParams: {
        ...DEFAULT_AVBD_PARAMS,
        ...initialConfig?.avbdParams,
      },
    },
    regenToken: 0,
    liveMetrics: DEFAULT_METRICS,
    benchmarkHistory: [],
    benchmarkRunning: false,
  });

  const requestRespawn = () => {
    state.update((current) => ({
      ...current,
      regenToken: current.regenToken + 1,
    }));
  };

  const pushBenchmark = (result) => {
    state.update((current) => ({
      ...current,
      benchmarkHistory: [result, ...current.benchmarkHistory].slice(0, 20),
      benchmarkRunning: false,
      activeScenario: undefined,
    }));
  };

  const setBenchmarkRunning = (running: boolean, scenarioId?: string) => {
    state.update((current) => ({
      ...current,
      benchmarkRunning: running,
      activeScenario: running ? scenarioId : undefined,
    }));
  };

  const updateConfig = (partial: Partial<SimulationConfig>) => {
    state.update((current) => ({
      ...current,
      config: {
        ...current.config,
        ...partial,
        objectSizes: {
          ...current.config.objectSizes,
          ...partial.objectSizes,
        },
        spawn: {
          ...current.config.spawn,
          ...partial.spawn,
        },
        avbdParams: {
          ...current.config.avbdParams!,
          ...partial.avbdParams,
        },
      },
    }));
  };

  const updateAvbdParams = (partial: Partial<AvbdParams>) => {
    state.update((current) => ({
      ...current,
      config: {
        ...current.config,
        avbdParams: {
          ...current.config.avbdParams!,
          ...partial,
        },
      },
    }));
  };

  return {
    state,
    requestRespawn,
    pushBenchmark,
    setBenchmarkRunning,
    updateConfig,
    updateAvbdParams,
  };
}
