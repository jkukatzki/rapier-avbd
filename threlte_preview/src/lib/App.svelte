<script lang="ts">
	import Scene from "./Scene.svelte";
	import { Pane, Button, Slider, Folder, Separator, Stepper } from "svelte-tweakpane-ui";
	import { Canvas } from "@threlte/core";
	import { simulationState, triggerRegen } from "./sharedState.svelte";

	const isDev = import.meta.env.DEV;
</script>

<Pane title="Simulation Controls" position="fixed" expanded={true}>
	<Button title="Respawn Bodies" on:click={triggerRegen} />
	<Slider
		label="Bodies per Solver"
		min={20}
		max={1000}
		step={10}
		bind:value={simulationState.numberOfObjects}
	/>
	{#if isDev}
		<Separator />
		<Folder title="AVBD Solver (dev)" expanded={false}>
			<Stepper
				label="Iterations"
				min={1}
				max={32}
				step={1}
				bind:value={simulationState.avbd.iterations}
			/>
			<Slider
				label="Alpha"
				min={0}
				max={1}
				step={0.01}
				bind:value={simulationState.avbd.alpha}
			/>
			<Slider
				label="Beta"
				min={1}
				max={25}
				step={0.5}
				bind:value={simulationState.avbd.beta}
			/>
			<Slider
				label="Gamma"
				min={0}
				max={1}
				step={0.005}
				bind:value={simulationState.avbd.gamma}
			/>
			<Stepper
				label="Stiffness Min"
				min={0}
				max={100000}
				step={10}
				bind:value={simulationState.avbd.stiffnessMin}
			/>
			<Stepper
				label="Stiffness Max"
				min={1000}
				max={2000000}
				step={1000}
				bind:value={simulationState.avbd.stiffnessMax}
			/>
			<Stepper
				label="Regularization"
				min={0}
				max={0.01}
				step={0.000001}
				bind:value={simulationState.avbd.regularization}
			/>
		</Folder>
	{/if}
</Pane>

<div class="viewport">
	<Canvas dpr={Math.min(typeof window !== "undefined" ? window.devicePixelRatio : 1, 2)}>
		<Scene />
	</Canvas>
</div>

<style>
	.viewport {
		width: 100vw;
		height: 100vh;
		z-index: 100;
	}
</style>
