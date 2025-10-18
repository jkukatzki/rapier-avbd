<script lang="ts">
	import Scene from "./Scene.svelte";
	import { Pane, Button, Slider } from "svelte-tweakpane-ui";
	import { Canvas } from "@threlte/core";
	import { simulationState, triggerRegen } from "./sharedState.svelte";
</script>

<Pane title="Simulation Controls" position="fixed" expanded={true}>
	<Button title="Respawn Bodies" on:click={triggerRegen} />
	<Slider
		label="Bodies per Solver"
		min={20}
		max={100}
		step={10}
		bind:value={simulationState.numberOfObjects}
	/>
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
	}
</style>
