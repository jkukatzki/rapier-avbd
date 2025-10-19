// Shared reactive state using Svelte 5 runes
export const simulationState = $state({
	numberOfObjects: 60,
	regen: false,
	enabled: {
		avbd: true,
		impulse: true
	},
	objectSizes: {
		cubeSize: 0.5,
		sphereRadius: 0.5,
		impulseOffset: 0.05
	},
	spawnSettings: {
		spread: 8,
		startHeight: 6
	},
	gravityScale: {
		avbd: 1,
		impulse: 1
	},
	avbd: {
		iterations: 4,
		alpha: 0.95,
		beta: 10,
		gamma: 0.99,
		stiffnessMin: 1,
		stiffnessMax: 1_000_000,
		regularization: 0.000_001
	},
	solverTimings: {
			solverMs: 0,
			totalMs: 0
		},
	
});

// Helper function to trigger regeneration
export function triggerRegen() {
	simulationState.regen = !simulationState.regen;
}
