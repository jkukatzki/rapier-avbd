<script lang="ts">
import Scene from "./Scene.svelte";
import { Pane, Button, Slider, Folder, Separator, Stepper, Element, Checkbox } from "svelte-tweakpane-ui";
	import { Canvas } from "@threlte/core";
	import { simulationState, triggerRegen } from "./sharedState.svelte";

	const isDev = import.meta.env.DEV;
</script>

<Pane title="Simulation Controls" position="fixed" expanded={true}>
	<Button title="Respawn Bodies" on:click={triggerRegen} />
	<Slider
		label="Bodies per Solver"
		min={20}
    max={2000}
		step={10}
		bind:value={simulationState.numberOfObjects}
	/>
	<Separator />
	<Folder title="Enable/Disable Solvers" expanded={true}>
		<Checkbox
			label="AVBD Solver"
			bind:value={simulationState.enabled.avbd}
		/>
		<Checkbox
			label="Impulse Solver"
			bind:value={simulationState.enabled.impulse}
		/>
	</Folder>
	<Separator />
	<Folder title="Solver Performance" expanded={true}>
		<Element>
			<div class="timing-display">
				<div class="timing-row">
					<strong>AVBD Solver:</strong>
					<span>{simulationState.solverTimings.avbd.solverMs.toFixed(3)} ms</span>
				</div>
				<div class="timing-row">
					<strong>Impulse Solver:</strong>
					<span>{simulationState.solverTimings.impulse.solverMs.toFixed(3)} ms</span>
				</div>
				<div class="timing-separator"></div>
				<div class="timing-row timing-secondary">
					<span>AVBD Total:</span>
					<span>{simulationState.solverTimings.avbd.totalMs.toFixed(3)} ms</span>
				</div>
				<div class="timing-row timing-secondary">
					<span>Impulse Total:</span>
					<span>{simulationState.solverTimings.impulse.totalMs.toFixed(3)} ms</span>
				</div>
			</div>
		</Element>
	</Folder>
	<Folder title="Gravity" expanded={false}>
		<Slider
			label="AVBD Gravity Scale"
			min={-10}
			max={10}
			step={0.1}
			bind:value={simulationState.gravityScale.avbd}
		/>
		<Slider
			label="Impulse Gravity Scale"
			min={-10}
			max={10}
			step={0.1}
			bind:value={simulationState.gravityScale.impulse}
		/>
	</Folder>
    {#if isDev}
        <Separator />
        <Folder title="AVBD Solver (dev)" expanded={false}>
            <Stepper
                label="Iterations"
                min={0}
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
            <Separator />
            <Element>
                <div class="avbd-report">
                    <strong>Applied Parameters</strong>
                    <span>iterations: {simulationState.avbdReport.iterations}</span>
                    <span>alpha: {simulationState.avbdReport.alpha.toFixed(3)}</span>
                    <span>beta: {simulationState.avbdReport.beta.toFixed(2)}</span>
                    <span>gamma: {simulationState.avbdReport.gamma.toFixed(3)}</span>
                    <span>stiffness min: {simulationState.avbdReport.stiffnessMin.toFixed(2)}</span>
                    <span>stiffness max: {simulationState.avbdReport.stiffnessMax.toFixed(2)}</span>
                    <span>regularization: {simulationState.avbdReport.regularization}</span>
                </div>
            </Element>
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

    .avbd-report {
        display: flex;
        flex-direction: column;
        gap: 4px;
        font-size: 12px;
    }

    .avbd-report strong {
        font-size: 13px;
    }

    .timing-display {
        display: flex;
        flex-direction: column;
        gap: 6px;
        font-size: 12px;
        padding: 4px 0;
    }

    .timing-row {
        display: flex;
        justify-content: space-between;
        align-items: center;
        gap: 12px;
    }

    .timing-row strong {
        font-size: 13px;
        font-weight: 600;
    }

    .timing-secondary {
        font-size: 11px;
        opacity: 0.8;
    }

    .timing-separator {
        height: 1px;
        background: rgba(255, 255, 255, 0.1);
        margin: 2px 0;
    }
</style>
