// Shared reactive state using Svelte 5 runes
export const simulationState = $state({
	numberOfObjects: 60,
	regen: false,
	avbd: {
		iterations: 4,
		alpha: 0.95,
		beta: 10,
		gamma: 0.99,
		stiffnessMin: 1,
		stiffnessMax: 1_000_000,
		regularization: 0.000_001
	}
});

// Helper function to trigger regeneration
export function triggerRegen() {
	simulationState.regen = !simulationState.regen;
}
