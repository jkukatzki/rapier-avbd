import { sveltekit } from '@sveltejs/kit/vite';
import { defineConfig } from 'vite';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const projectRoot = dirname(fileURLToPath(import.meta.url));
const workspaceRoot = resolve(projectRoot, '..');
const wasmPackageDir = resolve(workspaceRoot, 'rapier-wasm-avbd/pkg');

export default defineConfig({
	plugins: [sveltekit()],
	server: {
		fs: {
			allow: [projectRoot, workspaceRoot, wasmPackageDir]
		}
	},
	optimizeDeps: {
		exclude: ['@dimforge/rapier3d-compat']
	}
});
