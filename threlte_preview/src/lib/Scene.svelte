<script lang="ts">
	import { T, useTask } from "@threlte/core";
	import { simulationState } from "./sharedState.svelte";
	import {
		AmbientLight,
		BoxGeometry,
		DirectionalLight,
		InstancedMesh,
		MeshStandardMaterial,
		Object3D,
		SphereGeometry
	} from "three";

	const MAX_INSTANCES = 200;
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

	const cubeMeshAvbd = new InstancedMesh(new BoxGeometry(1, 1, 1), cubeMaterialAvbd, MAX_INSTANCES);
	const sphereMeshAvbd = new InstancedMesh(new SphereGeometry(0.6, 24, 24), sphereMaterialAvbd, MAX_INSTANCES);
	const cubeMeshImpulse = new InstancedMesh(
		new BoxGeometry(3, 3, 3),
		cubeMaterialImpulse,
		MAX_INSTANCES
	);
	const sphereMeshImpulse = new InstancedMesh(
		new SphereGeometry(0.6, 24, 24),
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

	const cubeHandlesAvbd: number[] = [];
	const sphereHandlesAvbd: number[] = [];
	const cubeHandlesImpulse: number[] = [];
	const sphereHandlesImpulse: number[] = [];

	const tempObject = new Object3D();

	let worldAvbd: any = null;
	let worldImpulse: any = null;
	let worldsReady = $state(false);

	let lastSpawnCount = simulationState.numberOfObjects;
	let lastRegenValue = simulationState.regen;

	function setupGround(world: any) {
		const groundHandle = world.create_fixed_body(0, -6, 0);
		world.create_cuboid_collider(groundHandle, 50, 2, 50);
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
		const spread = 12;

		for (let i = 0; i < cubeCount; i++) {
			const x = (Math.random() - 0.5) * spread;
			const y = 6 + Math.random() * 6;
			const z = (Math.random() - 0.5) * spread;

			const handleAvbd = worldAvbd.create_dynamic_body(x, y, z);
			worldAvbd.create_cuboid_collider(handleAvbd, 0.5, 0.5, 0.5);
			cubeHandlesAvbd.push(handleAvbd);

			const handleImpulse = worldImpulse.create_dynamic_body(x, y, z);
			worldImpulse.create_cuboid_collider(handleImpulse, 0.5, 0.5, 0.5);
			cubeHandlesImpulse.push(handleImpulse);
		}

		for (let i = 0; i < sphereCount; i++) {
			const x = (Math.random() - 0.5) * spread;
			const y = 8 + Math.random() * 6;
			const z = (Math.random() - 0.5) * spread;

			const handleAvbd = worldAvbd.create_dynamic_body(x, y, z);
			worldAvbd.create_ball_collider(handleAvbd, 0.6);
			sphereHandlesAvbd.push(handleAvbd);

			const handleImpulse = worldImpulse.create_dynamic_body(x, y, z);
			worldImpulse.create_ball_collider(handleImpulse, 0.6);
			sphereHandlesImpulse.push(handleImpulse);
		}

		updateInstances(cubeMeshAvbd, cubeHandlesAvbd, worldAvbd);
		updateInstances(sphereMeshAvbd, sphereHandlesAvbd, worldAvbd);
		updateInstances(cubeMeshImpulse, cubeHandlesImpulse, worldImpulse);
		updateInstances(sphereMeshImpulse, sphereHandlesImpulse, worldImpulse);
		lastSpawnCount = count;
	}

	function updateInstances(mesh: InstancedMesh, handles: number[], world: any) {
		mesh.count = handles.length;

		if (!world || handles.length === 0) {
			mesh.instanceMatrix.needsUpdate = true;
			return;
		}

		for (let i = 0; i < handles.length; i++) {
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
	}

	useTask(() => {
		if (!worldAvbd || !worldImpulse) return;
		worldAvbd.step();
		worldImpulse.step();
		updateInstances(cubeMeshAvbd, cubeHandlesAvbd, worldAvbd);
		updateInstances(sphereMeshAvbd, sphereHandlesAvbd, worldAvbd);
		updateInstances(cubeMeshImpulse, cubeHandlesImpulse, worldImpulse);
		updateInstances(sphereMeshImpulse, sphereHandlesImpulse, worldImpulse);
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
</script>

<T.PerspectiveCamera position={[12, 12, 12]} lookAt={[0, 0, 0]} fov={45} near={0.1} far={200} />
<T.AmbientLight args={[0xffffff, 0.7]} />
<T.DirectionalLight args={[0xffffff, 1.1]} position={[6, 12, 6]} castShadow={true} />

<T.Mesh position={[0, -7, 0]}>
	<T.BoxGeometry args={[80, 2, 80]} />
	<T.MeshStandardMaterial color="#5f5f5f" />
</T.Mesh>

<T is={worldRoot} />
