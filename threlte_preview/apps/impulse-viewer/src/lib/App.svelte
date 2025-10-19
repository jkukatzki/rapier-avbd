<script lang="ts">
  import { onMount } from 'svelte';
  import {
    ControlPanel,
    SceneCanvas,
    BENCHMARK_SCENARIOS,
    createSimulationStores,
    type BenchmarkScenario,
  } from '@rapier-avbd/viewer-shared';
  import adapter from './adapter';

  const stores = createSimulationStores({
    avbdParams: undefined,
  });
  let scene: { runBenchmark: (scenario: BenchmarkScenario) => Promise<void> } | null = null;

  onMount(async () => {
    await adapter.init();
  });

  async function handleBenchmark(event: CustomEvent<BenchmarkScenario>) {
    const scenario = event.detail;
    if (!scene) return;
    await scene.runBenchmark(scenario);
  }
</script>

<div class="viewer">
  <ControlPanel
    stores={stores}
    adapter={adapter}
    scenarios={BENCHMARK_SCENARIOS}
    on:runBenchmark={handleBenchmark}
  />
  <SceneCanvas bind:this={scene} stores={stores} adapter={adapter} scenarios={BENCHMARK_SCENARIOS} />
</div>

<style>
  .viewer {
    position: relative;
    width: 100vw;
    height: 100vh;
    overflow: hidden;
  }
</style>
