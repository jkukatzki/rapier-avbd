// Shared reactive state using Svelte 5 runes
export const simulationState = $state({
	numberOfObjects: 60,
	regen: false,
	enabled: {
		avbd: true,
		impulse: true
	},
	gravityScale: {
		avbd: 1,
		impulse: 1
	},
	avbdReport: {
		iterations: 0,
		alpha: 0,
		beta: 0,
		gamma: 0,
		stiffnessMin: 0,
		stiffnessMax: 0,
		regularization: 0
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
		avbd: {
			solverMs: 0,
			totalMs: 0
		},
		impulse: {
			solverMs: 0,
			totalMs: 0
		}
	}
});

// Helper function to trigger regeneration
export function triggerRegen() {
	simulationState.regen = !simulationState.regen;
}
