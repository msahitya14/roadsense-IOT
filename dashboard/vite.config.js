import { defineConfig } from 'vite';
import { svelte } from '@sveltejs/vite-plugin-svelte';

// base './' so dist/ works from any path (broker/run.sh serves it on :8000, or open it via any static server)
export default defineConfig({
  plugins: [svelte()],
  base: './',
  build: { outDir: 'dist', chunkSizeWarningLimit: 1500 },
  server: { port: 5173 },
});
