// Shared reactive state using Svelte 5 runes
export const simulationState = $state({
	numberOfObjects: 60,
	regen: false
});

// Helper function to trigger regeneration
export function triggerRegen() {
	simulationState.regen = !simulationState.regen;
}
