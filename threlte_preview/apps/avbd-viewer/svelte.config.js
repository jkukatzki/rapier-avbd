import adapter from '@sveltejs/adapter-static';
import { vitePreprocess } from '@sveltejs/vite-plugin-svelte';

const config = {
  kit: {
    adapter: adapter(),
    alias: {
      '@rapier-avbd/viewer-shared': '../../packages/viewer-shared/src'
    }
  },
  preprocess: vitePreprocess()
};

export default config;
