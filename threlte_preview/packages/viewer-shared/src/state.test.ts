import { get } from "svelte/store";
import { describe, expect, it } from "vitest";

import type { BenchmarkResult } from "./simulation/types";
import { createSimulationStores } from "./state";

const makeBenchmark = (overrides: Partial<BenchmarkResult> = {}): BenchmarkResult => ({
  scenarioId: "stack",
  scenarioLabel: "Stack Stress",
  solverId: "avbd",
  solverLabel: "AVBD",
  solverMs: 1.5,
  totalMs: 2.1,
  steps: 120,
  bodies: 256,
  timestamp: new Date().toISOString(),
  ...overrides,
});

describe("createSimulationStores", () => {
  it("initialises defaults when no overrides are provided", () => {
    const stores = createSimulationStores();
    const state = get(stores.state);

    expect(state.config.bodyCount).toBe(120);
    expect(state.config.avbdParams?.iterations).toBe(6);
    expect(state.liveMetrics.totalMs).toBe(0);
    expect(state.benchmarkHistory).toHaveLength(0);
  });

  it("merges nested configuration updates", () => {
    const stores = createSimulationStores();

    stores.updateConfig({
      gravityScale: 2,
      objectSizes: { cubeHalfExtent: 1 },
      spawn: { spread: 20 },
    });

    const state = get(stores.state);
    expect(state.config.gravityScale).toBe(2);
    expect(state.config.objectSizes.cubeHalfExtent).toBe(1);
    expect(state.config.objectSizes.sphereRadius).toBe(0.5);
    expect(state.config.spawn.spread).toBe(20);
    expect(state.config.spawn.startHeight).toBe(6);
  });

  it("updates AVBD params independently", () => {
    const stores = createSimulationStores();

    stores.updateAvbdParams({ iterations: 12, gamma: 0.9 });
    const state = get(stores.state);

    expect(state.config.avbdParams?.iterations).toBe(12);
    expect(state.config.avbdParams?.gamma).toBeCloseTo(0.9);
    expect(state.config.avbdParams?.alpha).toBeCloseTo(0.95);
  });

  it("records benchmark results and clears the active scenario", () => {
    const stores = createSimulationStores();
    stores.setBenchmarkRunning(true, "stack");

    const first = makeBenchmark({ scenarioId: "stack" });
    stores.pushBenchmark(first);

    const next = get(stores.state);
    expect(next.benchmarkHistory[0]).toEqual(first);
    expect(next.benchmarkRunning).toBe(false);
    expect(next.activeScenario).toBeUndefined();
  });

  it("retains only the most recent 20 benchmark results", () => {
    const stores = createSimulationStores();

    for (let i = 0; i < 25; i += 1) {
      stores.pushBenchmark(makeBenchmark({ timestamp: `${i}` }));
    }

    const history = get(stores.state).benchmarkHistory;
    expect(history).toHaveLength(20);
    expect(history[0].timestamp).toBe("24");
    expect(history.at(-1)?.timestamp).toBe("5");
  });

  it("increments the regen token on respawn requests", () => {
    const stores = createSimulationStores();
    const before = get(stores.state).regenToken;

    stores.requestRespawn();
    stores.requestRespawn();

    const after = get(stores.state).regenToken;
    expect(after - before).toBe(2);
  });
});
