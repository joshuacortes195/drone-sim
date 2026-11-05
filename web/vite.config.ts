import { defineConfig } from 'vite';

export default defineConfig({
  // relative paths, so the same build works on github pages under /drone-sim/ and locally
  base: './',
});
