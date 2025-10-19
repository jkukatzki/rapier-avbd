<script lang="ts">
  import type {
    BenchmarkScenario,
    SimulationAdapter,
    SimulationStores,
  } from "@rapier-avbd/viewer-shared";
  import { createEventDispatcher } from "svelte";

  export let stores: SimulationStores;
  export let adapter: SimulationAdapter;
  export let scenarios: BenchmarkScenario[];

  const dispatch = createEventDispatcher<{ runBenchmark: BenchmarkScenario }>();

  const stateStore = stores.state;

  $: state = $stateStore;
  $: config = state.config;
  $: metrics = state.liveMetrics;
  $: benchmarkHistory = state.benchmarkHistory;
  $: benchmarkRunning = state.benchmarkRunning;
  $: activeScenario = state.activeScenario;

  let bodyCount = config.bodyCount;
  $: if (config.bodyCount !== bodyCount) {
    bodyCount = config.bodyCount;
  }

  let gravityScale = config.gravityScale;
  $: if (config.gravityScale !== gravityScale) {
    gravityScale = config.gravityScale;
  }

  let cubeSize = config.objectSizes.cubeHalfExtent;
  let sphereRadius = config.objectSizes.sphereRadius;
  $: if (config.objectSizes.cubeHalfExtent !== cubeSize) {
    cubeSize = config.objectSizes.cubeHalfExtent;
  }
  $: if (config.objectSizes.sphereRadius !== sphereRadius) {
    sphereRadius = config.objectSizes.sphereRadius;
  }

  let spread = config.spawn.spread;
  let startHeight = config.spawn.startHeight;
  $: if (config.spawn.spread !== spread) {
    spread = config.spawn.spread;
  }
  $: if (config.spawn.startHeight !== startHeight) {
    startHeight = config.spawn.startHeight;
  }

  let avbd = config.avbdParams;
  $: avbd = config.avbdParams;

  function changeBodyCount(event: Event) {
    const value = Number((event.currentTarget as HTMLInputElement).value);
    stores.updateConfig({ bodyCount: value });
  }

  function changeGravity(event: Event) {
    const value = Number((event.currentTarget as HTMLInputElement).value);
    stores.updateConfig({ gravityScale: value });
  }

  function changeCubeSize(event: Event) {
    const value = Number((event.currentTarget as HTMLInputElement).value);
    stores.updateConfig({ objectSizes: { cubeHalfExtent: value } });
  }

  function changeSphereRadius(event: Event) {
    const value = Number((event.currentTarget as HTMLInputElement).value);
    stores.updateConfig({ objectSizes: { sphereRadius: value } });
  }

  function changeSpread(event: Event) {
    const value = Number((event.currentTarget as HTMLInputElement).value);
    stores.updateConfig({ spawn: { spread: value } });
  }

  function changeStartHeight(event: Event) {
    const value = Number((event.currentTarget as HTMLInputElement).value);
    stores.updateConfig({ spawn: { startHeight: value } });
  }

  function handleBenchmark(scenario: BenchmarkScenario) {
    if (benchmarkRunning) return;
    dispatch("runBenchmark", scenario);
  }

  function updateAvbdParams(partial: Partial<typeof avbd>) {
    stores.updateAvbdParams(partial);
  }
</script>

<div class="control-panel">
  <section class="group">
    <header>
      <h2>Simulation Controls</h2>
      <p class="solver">{adapter.solverLabel}</p>
    </header>
    <button class="primary" on:click={stores.requestRespawn}>Respawn Bodies</button>
    <label>
      <span>Bodies</span>
      <input
        type="range"
        min="20"
        max="4000"
        step="20"
        bind:value={bodyCount}
        on:change={changeBodyCount}
      />
      <span class="value">{bodyCount}</span>
    </label>
    <label>
      <span>Gravity Scale</span>
      <input
        type="range"
        min="-4"
        max="4"
        step="0.1"
        bind:value={gravityScale}
        on:change={changeGravity}
      />
      <span class="value">{gravityScale.toFixed(2)}</span>
    </label>
  </section>

  <section class="group">
    <header><h3>Object Sizes</h3></header>
    <label>
      <span>Cube Half-Extent</span>
      <input
        type="range"
        min="0.1"
        max="2"
        step="0.05"
        bind:value={cubeSize}
        on:change={changeCubeSize}
      />
      <span class="value">{cubeSize.toFixed(2)}</span>
    </label>
    <label>
      <span>Sphere Radius</span>
      <input
        type="range"
        min="0.1"
        max="2"
        step="0.05"
        bind:value={sphereRadius}
        on:change={changeSphereRadius}
      />
      <span class="value">{sphereRadius.toFixed(2)}</span>
    </label>
  </section>

  <section class="group">
    <header><h3>Spawn Settings</h3></header>
    <label>
      <span>Spread</span>
      <input
        type="range"
        min="2"
        max="120"
        step="2"
        bind:value={spread}
        on:change={changeSpread}
      />
      <span class="value">{spread.toFixed(0)}</span>
    </label>
    <label>
      <span>Start Height</span>
      <input
        type="range"
        min="2"
        max="30"
        step="0.5"
        bind:value={startHeight}
        on:change={changeStartHeight}
      />
      <span class="value">{startHeight.toFixed(1)}</span>
    </label>
  </section>

  {#if adapter.supportsAvbdParams && avbd}
    <section class="group">
      <header><h3>AVBD Parameters</h3></header>
      <label>
        <span>Iterations</span>
        <input
          type="range"
          min="1"
          max="32"
          step="1"
          value={avbd.iterations}
          on:input={(event) => updateAvbdParams({ iterations: Number((event.currentTarget as HTMLInputElement).value) })}
        />
        <span class="value">{avbd.iterations}</span>
      </label>
      <label>
        <span>Alpha</span>
        <input
          type="range"
          min="0"
          max="1"
          step="0.01"
          value={avbd.alpha}
          on:input={(event) => updateAvbdParams({ alpha: Number((event.currentTarget as HTMLInputElement).value) })}
        />
        <span class="value">{avbd.alpha.toFixed(2)}</span>
      </label>
      <label>
        <span>Beta</span>
        <input
          type="range"
          min="1"
          max="25"
          step="0.5"
          value={avbd.beta}
          on:input={(event) => updateAvbdParams({ beta: Number((event.currentTarget as HTMLInputElement).value) })}
        />
        <span class="value">{avbd.beta.toFixed(1)}</span>
      </label>
      <label>
        <span>Gamma</span>
        <input
          type="range"
          min="0"
          max="1"
          step="0.01"
          value={avbd.gamma}
          on:input={(event) => updateAvbdParams({ gamma: Number((event.currentTarget as HTMLInputElement).value) })}
        />
        <span class="value">{avbd.gamma.toFixed(2)}</span>
      </label>
      <label>
        <span>Stiffness Min</span>
        <input
          type="range"
          min="0"
          max="200000"
          step="100"
          value={avbd.stiffnessMin}
          on:input={(event) => updateAvbdParams({ stiffnessMin: Number((event.currentTarget as HTMLInputElement).value) })}
        />
        <span class="value">{avbd.stiffnessMin.toFixed(0)}</span>
      </label>
      <label>
        <span>Stiffness Max</span>
        <input
          type="range"
          min="1000"
          max="2000000"
          step="1000"
          value={avbd.stiffnessMax}
          on:input={(event) => updateAvbdParams({ stiffnessMax: Number((event.currentTarget as HTMLInputElement).value) })}
        />
        <span class="value">{avbd.stiffnessMax.toFixed(0)}</span>
      </label>
      <label>
        <span>Regularization</span>
        <input
          type="range"
          min="0"
          max="0.01"
          step="0.0001"
          value={avbd.regularization}
          on:input={(event) => updateAvbdParams({ regularization: Number((event.currentTarget as HTMLInputElement).value) })}
        />
        <span class="value">{avbd.regularization.toExponential(2)}</span>
      </label>
    </section>
  {/if}

  <section class="group">
    <header><h3>Benchmarks</h3></header>
    <div class="metrics">
      <div>
        <span>Solver Step</span>
        <strong>{metrics.solverMs.toFixed(3)} ms</strong>
      </div>
      <div>
        <span>Total Step</span>
        <strong>{metrics.totalMs.toFixed(3)} ms</strong>
      </div>
      {#if metrics.iterations !== undefined}
        <div>
          <span>Iterations</span>
          <strong>{metrics.iterations}</strong>
        </div>
      {/if}
    </div>
    <div class="scenario-list">
      {#each scenarios as scenario}
        <button
          class:active={activeScenario === scenario.id}
          disabled={benchmarkRunning}
          on:click={() => handleBenchmark(scenario)}
        >
          Run {scenario.label}
        </button>
        <p class="scenario-desc">{scenario.description}</p>
      {/each}
    </div>
    <table>
      <thead>
        <tr>
          <th>Scenario</th>
          <th>Solver</th>
          <th>Solver ms</th>
          <th>Total ms</th>
          <th>Steps</th>
          <th>Bodies</th>
          <th>Recorded</th>
        </tr>
      </thead>
      <tbody>
        {#if benchmarkHistory.length === 0}
          <tr>
            <td colspan="7" class="empty">No benchmark results yet.</td>
          </tr>
        {:else}
          {#each benchmarkHistory as result}
            <tr>
              <td>{result.scenarioLabel}</td>
              <td>{result.solverLabel}</td>
              <td>{result.solverMs.toFixed(3)}</td>
              <td>{result.totalMs.toFixed(3)}</td>
              <td>{result.steps}</td>
              <td>{result.bodies}</td>
              <td>{new Date(result.timestamp).toLocaleTimeString()}</td>
            </tr>
          {/each}
        {/if}
      </tbody>
    </table>
  </section>
</div>

<style>
  .control-panel {
    position: absolute;
    top: 1rem;
    left: 1rem;
    width: 320px;
    max-height: calc(100vh - 2rem);
    overflow-y: auto;
    padding: 1rem;
    border-radius: 12px;
    background: rgba(18, 18, 18, 0.85);
    color: #f5f5f5;
    backdrop-filter: blur(10px);
    box-shadow: 0 12px 24px rgba(0, 0, 0, 0.3);
    display: flex;
    flex-direction: column;
    gap: 1rem;
    z-index: 10;
  }

  .group header h2,
  .group header h3 {
    margin: 0;
    font-size: 1.1rem;
  }

  .solver {
    margin: 0;
    font-size: 0.85rem;
    color: #9bd2ff;
  }

  .group {
    display: flex;
    flex-direction: column;
    gap: 0.5rem;
    background: rgba(255, 255, 255, 0.04);
    padding: 0.75rem;
    border-radius: 10px;
  }

  label {
    display: grid;
    grid-template-columns: auto 1fr auto;
    gap: 0.5rem;
    align-items: center;
    font-size: 0.9rem;
  }

  label span.value {
    font-feature-settings: "tnum";
    font-variant-numeric: tabular-nums;
  }

  input[type="range"] {
    width: 100%;
  }

  button {
    border: none;
    border-radius: 8px;
    padding: 0.5rem 0.75rem;
    cursor: pointer;
    background: #2c6ff3;
    color: white;
    font-weight: 600;
    transition: background 0.2s ease, transform 0.2s ease;
  }

  button.primary {
    background: linear-gradient(135deg, #2c6ff3, #59b6ff);
  }

  button:disabled {
    opacity: 0.5;
    cursor: not-allowed;
  }

  button:not(:disabled):hover {
    transform: translateY(-1px);
  }

  .metrics {
    display: grid;
    grid-template-columns: repeat(2, minmax(0, 1fr));
    gap: 0.5rem;
    font-size: 0.85rem;
  }

  .metrics div {
    background: rgba(0, 0, 0, 0.3);
    padding: 0.5rem;
    border-radius: 6px;
  }

  .metrics strong {
    display: block;
    font-size: 1rem;
    font-feature-settings: "tnum";
  }

  .scenario-list {
    display: flex;
    flex-direction: column;
    gap: 0.25rem;
  }

  .scenario-list button {
    display: block;
    width: 100%;
  }

  .scenario-list button.active {
    background: #ff8c42;
  }

  .scenario-desc {
    font-size: 0.75rem;
    margin: 0 0 0.5rem 0;
    color: rgba(245, 245, 245, 0.7);
  }

  table {
    width: 100%;
    border-collapse: collapse;
    font-size: 0.75rem;
    background: rgba(0, 0, 0, 0.3);
    border-radius: 8px;
    overflow: hidden;
  }

  th,
  td {
    padding: 0.4rem 0.5rem;
    text-align: left;
  }

  tbody tr:nth-child(every) {
    background: rgba(255, 255, 255, 0.02);
  }

  .empty {
    text-align: center;
    padding: 1rem;
    color: rgba(245, 245, 245, 0.7);
  }
</style>
