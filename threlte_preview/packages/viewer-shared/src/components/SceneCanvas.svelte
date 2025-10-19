<script lang="ts">
  import { onMount, onDestroy } from "svelte";
  import { Canvas, T, useTask } from "@threlte/core";
  import {
    AmbientLight,
    BoxGeometry,
    DirectionalLight,
    InstancedMesh,
    MeshStandardMaterial,
    Object3D,
    SphereGeometry,
  } from "three";
  import { OrbitControls, Gizmo } from "@threlte/extras";
  import { get } from "svelte/store";
  import type {
    BenchmarkScenario,
    SimulationAdapter,
    SimulationStores,
    SpawnedHandles,
    WorldAdapter,
  } from "@rapier-avbd/viewer-shared";
  import { BENCHMARK_SCENARIOS } from "@rapier-avbd/viewer-shared";

  export let adapter: SimulationAdapter;
  export let stores: SimulationStores;
  export let scenarios: BenchmarkScenario[] = BENCHMARK_SCENARIOS;

  const stateStore = stores.state;

  const dummy = new Object3D();
  const MAX_INSTANCES = 20000;
  const cubeGeometry = new BoxGeometry(1, 1, 1);
  const sphereGeometry = new SphereGeometry(0.5, 18, 18);
  const cubeMaterial = new MeshStandardMaterial({ color: 0x4f9cdb });
  const sphereMaterial = new MeshStandardMaterial({ color: 0xd86acb });

  let cubeMesh: InstancedMesh | null = null;
  let sphereMesh: InstancedMesh | null = null;
  let world: WorldAdapter | null = null;
  let handles: SpawnedHandles = { cubes: [], spheres: [] };
  let ready = false;

  let positionsBuffer = new Float32Array(MAX_INSTANCES * 3);
  let lastBodyCount = 0;
  let lastRegen = 0;
  let lastCubeSize = 0.5;
  let lastSphereRadius = 0.5;

  async function initialiseWorld() {
    await adapter.init();
    const initialConfig = get(stateStore).config;
    world = await adapter.createWorld(initialConfig);
    handles = world.spawnBodies(initialConfig.bodyCount, initialConfig);
    lastBodyCount = initialConfig.bodyCount;
    lastCubeSize = initialConfig.objectSizes.cubeHalfExtent;
    lastSphereRadius = initialConfig.objectSizes.sphereRadius;
    lastRegen = get(stateStore).regenToken;
    updateInstanceCounts();
    ready = true;
  }

  function updateInstanceCounts() {
    if (cubeMesh) {
      cubeMesh.count = handles.cubes.length;
      cubeMesh.instanceMatrix.needsUpdate = true;
    }
    if (sphereMesh) {
      sphereMesh.count = handles.spheres.length;
      sphereMesh.instanceMatrix.needsUpdate = true;
    }
  }

  function refreshSpawn() {
    if (!world) return;
    const current = get(stateStore).config;
    world.clearBodies();
    handles = world.spawnBodies(current.bodyCount, current);
    lastBodyCount = current.bodyCount;
    lastCubeSize = current.objectSizes.cubeHalfExtent;
    lastSphereRadius = current.objectSizes.sphereRadius;
    updateInstanceCounts();
  }

  function syncTransforms() {
    if (!world) return;
    if (!cubeMesh || !sphereMesh) return;
    const totalInstances = handles.cubes.length + handles.spheres.length;
    if (totalInstances === 0) return;
    world.getBodyPositions(handles, positionsBuffer);
    let cursor = 0;
    const current = get(stateStore).config;
    const cubeScale = current.objectSizes.cubeHalfExtent * 2;
    const sphereScale = current.objectSizes.sphereRadius * 2;

    for (let i = 0; i < handles.cubes.length; i++) {
      dummy.position.set(
        positionsBuffer[cursor],
        positionsBuffer[cursor + 1],
        positionsBuffer[cursor + 2]
      );
      dummy.scale.setScalar(cubeScale);
      dummy.updateMatrix();
      cubeMesh.setMatrixAt(i, dummy.matrix);
      cursor += 3;
    }

    for (let i = 0; i < handles.spheres.length; i++) {
      dummy.position.set(
        positionsBuffer[cursor],
        positionsBuffer[cursor + 1],
        positionsBuffer[cursor + 2]
      );
      dummy.scale.setScalar(sphereScale);
      dummy.updateMatrix();
      sphereMesh.setMatrixAt(i, dummy.matrix);
      cursor += 3;
    }

    cubeMesh.instanceMatrix.needsUpdate = true;
    sphereMesh.instanceMatrix.needsUpdate = true;
  }

  $: if (ready && world) {
    const current = $stateStore.config;
    const regen = $stateStore.regenToken;
    if (
      current.bodyCount !== lastBodyCount ||
      current.objectSizes.cubeHalfExtent !== lastCubeSize ||
      current.objectSizes.sphereRadius !== lastSphereRadius ||
      regen !== lastRegen
    ) {
      lastRegen = regen;
      refreshSpawn();
    }
    world.setGravityScale(current.gravityScale);
    if (world.applyAvbdParams && current.avbdParams) {
      world.applyAvbdParams(current.avbdParams);
    }
  }

  useTask(() => {
    if (!ready || !world) return;
    const metrics = world.step();
    stores.state.update((state) => ({
      ...state,
      liveMetrics: metrics,
    }));
    syncTransforms();
  });

  export async function runBenchmark(scenario: BenchmarkScenario) {
    if (!world) return;
    const config = get(stateStore).config;
    stores.setBenchmarkRunning(true, scenario.id);
    try {
      const result = await world.runBenchmark({ config, scenario });
      stores.pushBenchmark(result);
    } finally {
      stores.setBenchmarkRunning(false);
    }
  }

  onMount(() => {
    initialiseWorld();
  });

  onDestroy(() => {
    world?.dispose?.();
    cubeGeometry.dispose();
    sphereGeometry.dispose();
    cubeMaterial.dispose();
    sphereMaterial.dispose();
  });
</script>

<div class="scene">
  <Canvas>
    <T.PerspectiveCamera makeDefault position={[12, 14, 16]} fov={55}>
      <OrbitControls enableDamping />
      <Gizmo />
    </T.PerspectiveCamera>
    <T.AmbientLight args={[0xffffff, 0.5]} />
    <T.DirectionalLight args={[0xffffff, 1.2]} position={[14, 12, 8]} castShadow />

    <T.Mesh rotation={[-Math.PI / 2, 0, 0]} receiveShadow>
      <T.PlaneGeometry args={[120, 120]} />
      <T.MeshStandardMaterial color="#323232" />
    </T.Mesh>

    <T.Mesh
      bind:this={cubeMesh}
      args={[cubeGeometry, cubeMaterial, MAX_INSTANCES / 2]}
      frustumCulled={false}
      castShadow
      receiveShadow
    />
    <T.Mesh
      bind:this={sphereMesh}
      args={[sphereGeometry, sphereMaterial, MAX_INSTANCES / 2]}
      frustumCulled={false}
      castShadow
      receiveShadow
    />
  </Canvas>
</div>

<style>
  .scene {
    position: absolute;
    inset: 0;
  }

  canvas {
    width: 100%;
    height: 100%;
    display: block;
  }
</style>
