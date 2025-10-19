<script lang="ts">
	import { T, useTask } from "@threlte/core";
	import { simulationState } from "./sharedState.svelte";
	import {
		AmbientLight,
		BoxGeometry,
		DirectionalLight,
		InstancedBufferAttribute,
		InstancedMesh,
		MeshStandardMaterial,
		Object3D,
		SphereGeometry
	} from "three";
	import { Gizmo, interactivity, OrbitControls } from "@threlte/extras";

	interactivity();
	const MAX_INSTANCES = 20000;
	const worldRoot = new Object3D();

	const cubeMaterial = new MeshStandardMaterial({ color: 'green' });
	const sphereMaterial = new MeshStandardMaterial({ color: 'blue' });

	let cubeMesh: InstancedMesh;
	let sphereMesh: InstancedMesh;

	useTask(() => {
		const avbdMetrics = worldAvbd.step_with_metrics();
		simulationState.solverTimings.solverMs = avbdMetrics.solver_ms();
		simulationState.solverTimings.totalMs = avbdMetrics.total_ms();
		updateInstances(cubeMeshAvbd, cubeHandlesAvbd, worldAvbd);
		updateInstances(sphereMeshAvbd, sphereHandlesAvbd, worldAvbd);
	});

	// Initialize worlds on mount
	$effect(() => {
		if (import.meta.env.SSR) return;

		(async () => {
			const rapierModule = await import("@validvector/rapier3d-avbd");
			const { default: init, init_panic_hook, RapierWorld } = rapierModule;

			await init();
			init_panic_hook();

			world = new RapierWorld(0, -9.81, 0, true);

			setupGround(world);
			applyAvbdParams();

			worldsReady = true;
			spawnBodies(simulationState.numberOfObjects);
		})();

		return () => {
			worldAvbd = null;
			worldImpulse = null;
		};
	});

	// Watch for changes in numberOfObjects
	$effect(() => {
		if (worldsReady && simulationState.numberOfObjects !== lastSpawnCount) {
			spawnBodies(simulationState.numberOfObjects);
		}
	});



	// Watch for regen changes
	$effect(() => {
		// Access the regen value to track it
		const currentRegen = simulationState.regen;
		
		if (worldsReady && currentRegen !== lastRegenValue) {
			spawnBodies(simulationState.numberOfObjects);
			lastRegenValue = currentRegen;
		}
	});



	// Interactive sphere controlled by mouse
	let mousePosition = $state({ x: 0, y: 5, z: 0 });
	let interactiveBodyAvbd = $state<number | null>(null);
	let interactiveBodyImpulse = $state<number | null>(null);
	const interactiveRadius = 1.0;


	// Update kinematic body positions when mouse moves
	$effect(() => {
		if (interactiveBodyAvbd === null || interactiveBodyImpulse === null) return;
		if (!worldAvbd || !worldImpulse) return;
		
		const { x, y, z } = mousePosition;
		worldAvbd.set_body_translation(interactiveBodyAvbd, x, y, z);
	});

	// Handle pointer move with raycasting
	function handlePointerMove(event: any) {
		if (!event.intersections?.length) return;
		
		// Get the intersection with the ground plane
		const intersection = event.intersections[0];
		if (intersection?.point) {
			mousePosition = {
				x: intersection.point.x,
				y: 5, // Keep it at a fixed height above ground
				z: intersection.point.z
			};
		}
	}
</script>

<T.PerspectiveCamera
  makeDefault
  fov={100}
  position.z={-20}
  position.y={20}
>
  <OrbitControls enableDamping ><Gizmo /></OrbitControls>
</T.PerspectiveCamera>
<T.AmbientLight args={[0xffffff, 0.7]} />
<T.DirectionalLight args={[0xffffff, 1.1]} position={[6, 12, 6]} castShadow={true} />

<T.Mesh onpointermove={handlePointerMove} position={[0, 0, 0]}>
	<T.BoxGeometry args={[1000, 0.1, 1000]} />
	<T.MeshStandardMaterial color="#5f5f5f" />
</T.Mesh>

<!-- Interactive sphere that follows mouse -->
{#if interactiveBodyAvbd !== null && worldAvbd}
	<T.Mesh position={[mousePosition.x, mousePosition.y, mousePosition.z]}>
		<T.SphereGeometry args={[1.0, 32, 32]} />
		<T.MeshStandardMaterial color="yellow" />
	</T.Mesh>
{/if}

<T is={worldRoot} />
