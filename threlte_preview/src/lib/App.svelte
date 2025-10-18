<script lang="ts">
    	import App from "$lib/App.svelte";
	import { Canvas, T, useTask } from "@threlte/core";
	import { onDestroy, onMount } from "svelte";
	import {
		AmbientLight,
		BoxGeometry,
		DirectionalLight,
		InstancedMesh,
		MeshStandardMaterial,
		Object3D,
		SphereGeometry
	} from "three";

	const worldRoot = new Object3D();
	const instancedRoot = new Object3D();
	worldRoot.add(instancedRoot);

	const ambient = new AmbientLight(0xffffff, 0.75);
	const directional = new DirectionalLight(0xffffff, 1.0);
	directional.position.set(6, 12, 6);
	worldRoot.add(ambient);
	worldRoot.add(directional);

	const cubeCount = 60;
	const sphereCount = 40;
	const cubeMesh = new InstancedMesh(
		new BoxGeometry(1, 1, 1),
		new MeshStandardMaterial({ color: 0x4caf50 }),
		cubeCount
	);
	const sphereMesh = new InstancedMesh(
		new SphereGeometry(0.6, 24, 24),
		new MeshStandardMaterial({ color: 0x2196f3 }),
		sphereCount
	);

	cubeMesh.frustumCulled = false;
	sphereMesh.frustumCulled = false;
	instancedRoot.add(cubeMesh);
	instancedRoot.add(sphereMesh);

	const cubeHandles: number[] = [];
	const sphereHandles: number[] = [];

	const tempObject = new Object3D();
	let rapierWorld: any = null;
	const pixelRatio = typeof window !== "undefined" ? Math.min(window.devicePixelRatio, 2) : 1;

	function updateInstances(mesh: InstancedMesh, handles: number[]) {
		if (!rapierWorld) return;

		for (let i = 0; i < handles.length; i++) {
			const translation = rapierWorld.get_body_translation(handles[i]);
			tempObject.position.set(translation[0], translation[1], translation[2]);
			tempObject.rotation.set(0, 0, 0);
			tempObject.updateMatrix();
			mesh.setMatrixAt(i, tempObject.matrix);
		}
		mesh.instanceMatrix.needsUpdate = true;
	}

	useTask(() => {
		if (!rapierWorld) return;
		rapierWorld.step();
		updateInstances(cubeMesh, cubeHandles);
		updateInstances(sphereMesh, sphereHandles);
	});

	onMount(async () => {
		if (import.meta.env.SSR) return;

		const rapierModule = await import("@dimforge/rapier3d-compat");
		const { default: init, init_panic_hook, RapierWorld } = rapierModule;

		await init();
		init_panic_hook();

		rapierWorld = new RapierWorld(0, -9.81, 0, true);

		const groundHandle = rapierWorld.create_fixed_body(0, -6, 0);
		rapierWorld.create_cuboid_collider(groundHandle, 50, 2, 50);

		const spread = 12;
		for (let i = 0; i < cubeCount; i++) {
			const bodyHandle = rapierWorld.create_dynamic_body(
				(Math.random() - 0.5) * spread,
				6 + Math.random() * 6,
				(Math.random() - 0.5) * spread
			);
			rapierWorld.create_cuboid_collider(bodyHandle, 0.5, 0.5, 0.5);
			cubeHandles.push(bodyHandle);
		}

		for (let i = 0; i < sphereCount; i++) {
			const bodyHandle = rapierWorld.create_dynamic_body(
				(Math.random() - 0.5) * spread,
				8 + Math.random() * 6,
				(Math.random() - 0.5) * spread
			);
			rapierWorld.create_ball_collider(bodyHandle, 0.6);
			sphereHandles.push(bodyHandle);
		}

		updateInstances(cubeMesh, cubeHandles);
		updateInstances(sphereMesh, sphereHandles);
	});

	onDestroy(() => {
		rapierWorld = null;
	});
</script>

<T.PerspectiveCamera
    position={[0, 10, 20]}
    lookAt={[0, 0, 0]}
    fov={45}
    near={0.1}
    far={1000}
    pixelRatio={pixelRatio}></T.PerspectiveCamera>
<T.Mesh>
    <T.BoxGeometry args={[50, 2, 50]} />
    <T.MeshStandardMaterial color="#9e9e9e" />
</T.Mesh>
<T is={worldRoot} />