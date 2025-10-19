import type { BenchmarkScenario } from "./types";

export const BENCHMARK_SCENARIOS: BenchmarkScenario[] = [
  {
    id: "stack-drop",
    label: "Stack Drop",
    description: "Spawn a tall stack of alternating cubes and spheres and simulate the collapse.",
    steps: 240,
    spawnMultiplier: 1,
  },
  {
    id: "pile-driver",
    label: "Pile Driver",
    description: "Drop a dense cluster of bodies into a confined pit to stress contact resolution.",
    steps: 360,
    spawnMultiplier: 1.5,
  },
  {
    id: "broad-sweep",
    label: "Broad Sweep",
    description: "Simulate a wide horizontal sweep of bodies to exercise broad-phase and sleeping logic.",
    steps: 180,
    spawnMultiplier: 2,
  },
];
