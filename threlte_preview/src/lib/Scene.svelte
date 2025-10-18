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
	const avbdRoot = new Object3D();
	const impulseRoot = new Object3D();
	worldRoot.add(avbdRoot);
	worldRoot.add(impulseRoot);

	const cubeMaterialAvbd = new MeshStandardMaterial({ color: 'green' });
	const sphereMaterialAvbd = new MeshStandardMaterial({ color: 'blue' });
	const cubeMaterialImpulse = new MeshStandardMaterial({
		color: 'orange',
		transparent: true,
		opacity: 0.5
	});
	const sphereMaterialImpulse = new MeshStandardMaterial({
		color: 'red',
		transparent: true,
		opacity: 0.5
	});

	let cubeMeshAvbd: InstancedMesh;
	let sphereMeshAvbd: InstancedMesh;
	let cubeMeshImpulse: InstancedMesh;
	let sphereMeshImpulse: InstancedMesh;

	function createMeshes() {
		const cubeSize = simulationState.objectSizes.cubeSize;
		const sphereRadius = simulationState.objectSizes.sphereRadius;
		const offset = simulationState.objectSizes.impulseOffset;

		// Remove old meshes if they exist
		if (cubeMeshAvbd) avbdRoot.remove(cubeMeshAvbd);
		if (sphereMeshAvbd) avbdRoot.remove(sphereMeshAvbd);
		if (cubeMeshImpulse) impulseRoot.remove(cubeMeshImpulse);
		if (sphereMeshImpulse) impulseRoot.remove(sphereMeshImpulse);

		// Create new meshes with updated sizes
		cubeMeshAvbd = new InstancedMesh(
			new BoxGeometry(cubeSize * 2, cubeSize * 2, cubeSize * 2),
			cubeMaterialAvbd,
			MAX_INSTANCES
		);
		sphereMeshAvbd = new InstancedMesh(
			new SphereGeometry(sphereRadius, 24, 24),
			sphereMaterialAvbd,
			MAX_INSTANCES
		);
		cubeMeshImpulse = new InstancedMesh(
			new BoxGeometry((cubeSize + offset) * 2, (cubeSize + offset) * 2, (cubeSize + offset) * 2),
			cubeMaterialImpulse,
			MAX_INSTANCES
		);
		sphereMeshImpulse = new InstancedMesh(
			new SphereGeometry(sphereRadius + offset, 24, 24),
			sphereMaterialImpulse,
			MAX_INSTANCES
		);

		cubeMeshAvbd.frustumCulled = false;
		sphereMeshAvbd.frustumCulled = false;
		cubeMeshImpulse.frustumCulled = false;
		sphereMeshImpulse.frustumCulled = false;

		avbdRoot.add(cubeMeshAvbd);
		avbdRoot.add(sphereMeshAvbd);
		impulseRoot.add(cubeMeshImpulse);
		impulseRoot.add(sphereMeshImpulse);
	}

	// Initialize meshes
	createMeshes();

	const cubeHandlesAvbd: number[] = [];
	const sphereHandlesAvbd: number[] = [];
	const cubeHandlesImpulse: number[] = [];
	const sphereHandlesImpulse: number[] = [];

	const tempObject = new Object3D();

	let worldAvbd: any = $state(null);
	let worldImpulse: any = $state(null);
	let worldsReady = $state(false);

	let lastSpawnCount = simulationState.numberOfObjects;
	let lastRegenValue = simulationState.regen;

	function setupGround(world: any) {
		const groundHandle = world.create_fixed_body(0, 0, 0);
		world.create_cuboid_collider(groundHandle, 1000, 0.1, 1000);
	}

function clearHandles() {
	cubeHandlesAvbd.length = 0;
	sphereHandlesAvbd.length = 0;
	cubeHandlesImpulse.length = 0;
	sphereHandlesImpulse.length = 0;
		updateInstances(cubeMeshAvbd, cubeHandlesAvbd, worldAvbd);
		updateInstances(sphereMeshAvbd, sphereHandlesAvbd, worldAvbd);
		updateInstances(cubeMeshImpulse, cubeHandlesImpulse, worldImpulse);
	updateInstances(sphereMeshImpulse, sphereHandlesImpulse, worldImpulse);
}

function refreshGravityScales() {
	if (!worldAvbd || !worldImpulse) return;
	const avbdScale = simulationState.gravityScale.avbd;
	const impulseScale = simulationState.gravityScale.impulse;

	for (const handle of cubeHandlesAvbd) {
		worldAvbd.set_body_gravity_scale(handle, avbdScale);
	}
	for (const handle of sphereHandlesAvbd) {
		worldAvbd.set_body_gravity_scale(handle, avbdScale);
	}
	for (const handle of cubeHandlesImpulse) {
		worldImpulse.set_body_gravity_scale(handle, impulseScale);
	}
	for (const handle of sphereHandlesImpulse) {
		worldImpulse.set_body_gravity_scale(handle, impulseScale);
	}
}

function applyAvbdParams() {
	if (!worldAvbd) return;
	const params = simulationState.avbd;
	worldAvbd.set_avbd_params(
		params.iterations,
			params.alpha,
			params.beta,
			params.gamma,
			params.stiffnessMin,
			params.stiffnessMax,
			params.regularization
		);
	}

	function resetWorlds() {
		if (!worldAvbd || !worldImpulse) return;
		worldAvbd.reset();
		worldImpulse.reset();
		setupGround(worldAvbd);
		setupGround(worldImpulse);
		clearHandles();
	}

	function spawnBodies(count: number) {
		if (!worldAvbd || !worldImpulse) return;

		resetWorlds();

		const cubeCount = Math.ceil(count * 0.6);
		const sphereCount = Math.max(count - cubeCount, 0);
		const spread = simulationState.spawnSettings.spread;
		const startHeight = simulationState.spawnSettings.startHeight;
		const cubeSize = simulationState.objectSizes.cubeSize;
		const sphereRadius = simulationState.objectSizes.sphereRadius;

		for (let i = 0; i < cubeCount; i++) {
			const x = (Math.random() - 0.5) * spread;
			const y = startHeight + Math.random() * 6;
			const z = (Math.random() - 0.5) * spread;

			const handleAvbd = worldAvbd.create_dynamic_body(x, y, z);
			worldAvbd.create_cuboid_collider(handleAvbd, cubeSize, cubeSize, cubeSize);
			cubeHandlesAvbd.push(handleAvbd);

			const handleImpulse = worldImpulse.create_dynamic_body(x, y, z);
			worldImpulse.create_cuboid_collider(handleImpulse, cubeSize, cubeSize, cubeSize);
			cubeHandlesImpulse.push(handleImpulse);
		}

	for (let i = 0; i < sphereCount; i++) {
		const x = (Math.random() - 0.5) * spread;
		const y = startHeight + 2 + Math.random() * 6;
		const z = (Math.random() - 0.5) * spread;

		const handleAvbd = worldAvbd.create_dynamic_body(x, y, z);
		worldAvbd.create_ball_collider(handleAvbd, sphereRadius);
		sphereHandlesAvbd.push(handleAvbd);

		const handleImpulse = worldImpulse.create_dynamic_body(x, y, z);
		worldImpulse.create_ball_collider(handleImpulse, sphereRadius);
		sphereHandlesImpulse.push(handleImpulse);
	}

	refreshGravityScales();

	updateInstances(cubeMeshAvbd, cubeHandlesAvbd, worldAvbd);
	updateInstances(sphereMeshAvbd, sphereHandlesAvbd, worldAvbd);
	updateInstances(cubeMeshImpulse, cubeHandlesImpulse, worldImpulse);
	updateInstances(sphereMeshImpulse, sphereHandlesImpulse, worldImpulse);
	lastSpawnCount = count;
	}

	function updateInstances(mesh: InstancedMesh, handles: number[], world: any) {
		const capacity =
			(mesh.instanceMatrix as InstancedBufferAttribute | undefined)?.count ??
			mesh.count ??
			MAX_INSTANCES;
		const instanceCount = Math.min(handles.length, capacity);
		mesh.count = instanceCount;

		if (!world || instanceCount === 0) {
			mesh.instanceMatrix.needsUpdate = true;
			if (handles.length > capacity) {
				console.warn(
					`[Scene] Instanced mesh capacity exceeded; skipping ${
						handles.length - capacity
					} instances.`,
				);
			}
			return;
		}

		for (let i = 0; i < instanceCount; i++) {
			const translation = world.get_body_translation(handles[i]);
			const rotation = world.get_body_rotation(handles[i]);

			tempObject.position.set(translation[0], translation[1], translation[2]);

			if (rotation && rotation.length === 4) {
				tempObject.quaternion.set(rotation[0], rotation[1], rotation[2], rotation[3]);
			} else {
				tempObject.quaternion.identity();
			}

			tempObject.updateMatrix();
			mesh.setMatrixAt(i, tempObject.matrix);
		}

		mesh.instanceMatrix.needsUpdate = true;

		if (handles.length > capacity) {
			console.warn(
				`[Scene] Instanced mesh capacity exceeded; rendering capped at ${capacity} instances.`,
			);
		}
	}

	useTask(() => {
		if (!worldAvbd || !worldImpulse) return;
		
		// Step AVBD world if enabled
		if (simulationState.enabled.avbd) {
			const avbdMetrics = worldAvbd.step_with_metrics();
			simulationState.solverTimings.avbd.solverMs = avbdMetrics.solver_ms();
			simulationState.solverTimings.avbd.totalMs = avbdMetrics.total_ms();
			updateInstances(cubeMeshAvbd, cubeHandlesAvbd, worldAvbd);
			updateInstances(sphereMeshAvbd, sphereHandlesAvbd, worldAvbd);
		}
		
		// Step Impulse world if enabled
		if (simulationState.enabled.impulse) {
			const impulseMetrics = worldImpulse.step_with_metrics();
			simulationState.solverTimings.impulse.solverMs = impulseMetrics.solver_ms();
			simulationState.solverTimings.impulse.totalMs = impulseMetrics.total_ms();
			updateInstances(cubeMeshImpulse, cubeHandlesImpulse, worldImpulse);
			updateInstances(sphereMeshImpulse, sphereHandlesImpulse, worldImpulse);
		}
	});

	// Initialize worlds on mount
	$effect(() => {
		if (import.meta.env.SSR) return;

		(async () => {
			const rapierModule = await import("@validvector/rapier3d-avbd");
			const { default: init, init_panic_hook, RapierWorld } = rapierModule;

			await init();
			init_panic_hook();

			worldAvbd = new RapierWorld(0, -9.81, 0, true);
			worldImpulse = new RapierWorld(0, -9.81, 0, false);

			console.info("AVBD solver backend:", worldAvbd.get_solver_backend());
			console.info("Impulse solver backend:", worldImpulse.get_solver_backend());

			setupGround(worldAvbd);
			setupGround(worldImpulse);
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

	$effect(() => {
		if (!worldsReady) return;
		// Access AVBD parameters to register dependencies with runes.
		const {
			iterations,
			alpha,
			beta,
			gamma,
			stiffnessMin,
			stiffnessMax,
			regularization
		} = simulationState.avbd;

	if (!worldAvbd) return;
	worldAvbd.set_avbd_params(
		iterations,
		alpha,
		beta,
		gamma,
		stiffnessMin,
		stiffnessMax,
		regularization
	);

	const paramsReport =
		typeof worldAvbd.get_avbd_params === "function"
			? worldAvbd.get_avbd_params()
			: null;
	if (paramsReport && paramsReport.length === 7) {
		const [iter, alphaApplied, betaApplied, gammaApplied, stiffMin, stiffMax, reg] =
			paramsReport;
		console.log(
			`[AVBD params] iterations=${iter}, alpha=${alphaApplied.toFixed(
				3
			)}, beta=${betaApplied.toFixed(2)}, gamma=${gammaApplied.toFixed(
				3
			)}, stiffnessMin=${stiffMin.toFixed(2)}, stiffnessMax=${stiffMax.toFixed(
				2
			)}, regularization=${reg}`
		);
		simulationState.avbdReport = {
			iterations: iter,
			alpha: alphaApplied,
			beta: betaApplied,
			gamma: gammaApplied,
			stiffnessMin: stiffMin,
			stiffnessMax: stiffMax,
			regularization: reg
		};
	} else if (!paramsReport) {
		console.warn("[AVBD params] get_avbd_params() unavailable; rebuild WASM package?");
	}
});

$effect(() => {
	if (!worldsReady) return;
	const { avbd, impulse } = simulationState.gravityScale;
	// Touch the values to register reactivity, then refresh.
	void avbd;
	void impulse;
	refreshGravityScales();
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

	// Control visibility of solver meshes
	$effect(() => {
		avbdRoot.visible = simulationState.enabled.avbd;
		impulseRoot.visible = simulationState.enabled.impulse;
	});

	// Watch for size changes and recreate meshes + respawn
	$effect(() => {
		if (!worldsReady) return;
		const { cubeSize, sphereRadius, impulseOffset } = simulationState.objectSizes;
		// Touch values to register reactivity
		void cubeSize;
		void sphereRadius;
		void impulseOffset;
		
		createMeshes();
		spawnBodies(simulationState.numberOfObjects);
	});

	// Watch for spawn setting changes
	$effect(() => {
		if (!worldsReady) return;
		const { spread, startHeight } = simulationState.spawnSettings;
		// Touch values to register reactivity
		void spread;
		void startHeight;
		
		spawnBodies(simulationState.numberOfObjects);
	});

	// Interactive sphere controlled by mouse
	let mousePosition = $state({ x: 0, y: 5, z: 0 });
	let interactiveBodyAvbd = $state<number | null>(null);
	let interactiveBodyImpulse = $state<number | null>(null);
	const interactiveRadius = 1.0;

	// Create kinematic bodies once when worlds are ready
	$effect(() => {
		if (!worldsReady || !worldAvbd || !worldImpulse) return;

		// Create kinematic bodies (can be moved without physics simulation)
		// Start at origin, will be moved by mouse later
		const bodyAvbd = worldAvbd.create_kinematic_body(0, 5, 0);
		worldAvbd.create_ball_collider(bodyAvbd, interactiveRadius);
		interactiveBodyAvbd = bodyAvbd;

		const bodyImpulse = worldImpulse.create_kinematic_body(0, 5, 0);
		worldImpulse.create_ball_collider(bodyImpulse, interactiveRadius);
		interactiveBodyImpulse = bodyImpulse;

		return () => {
			if (bodyAvbd !== null && worldAvbd) {
				worldAvbd.remove_body(bodyAvbd);
			}
			if (bodyImpulse !== null && worldImpulse) {
				worldImpulse.remove_body(bodyImpulse);
			}
			interactiveBodyAvbd = null;
			interactiveBodyImpulse = null;
		};
	});

	// Update kinematic body positions when mouse moves
	$effect(() => {
		if (interactiveBodyAvbd === null || interactiveBodyImpulse === null) return;
		if (!worldAvbd || !worldImpulse) return;
		
		const { x, y, z } = mousePosition;
		worldAvbd.set_body_translation(interactiveBodyAvbd, x, y, z);
		worldImpulse.set_body_translation(interactiveBodyImpulse, x, y, z);
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
