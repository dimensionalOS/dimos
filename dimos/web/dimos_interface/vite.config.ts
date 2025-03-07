import { defineConfig } from 'vite';
import { svelte } from '@sveltejs/vite-plugin-svelte';

// https://vitejs.dev/config/
export default defineConfig({
  plugins: [svelte()],
  server: {
    port: 3000,
    watch: {
      // Exclude node_modules, .git and other large directories
      ignored: ['**/node_modules/**', '**/.git/**', '**/dist/**', 'lambda/**'],
      // Use polling instead of filesystem events (less efficient but uses fewer watchers)
      usePolling: true,
    },
    proxy: {
      '/api': {
        target: 'https://0rqz7w5rvf.execute-api.us-east-2.amazonaws.com',
        changeOrigin: true,
        rewrite: (path) => path.replace(/^\/api/, '/default/getGenesis'),
        configure: (proxy, _options) => {
          proxy.on('error', (err, _req, _res) => {
            console.log('proxy error', err);
          });
          proxy.on('proxyReq', (proxyReq, req, _res) => {
            console.log('Sending Request to the Target:', req.method, req.url);
          });
          proxy.on('proxyRes', (proxyRes, req, _res) => {
            console.log('Received Response from the Target:', proxyRes.statusCode, req.url);
          });
        },
      },
      '/unitree': {
        target: 'http://localhost:5555',
        changeOrigin: true,
        configure: (proxy, _options) => {
          proxy.on('error', (err, _req, _res) => {
            console.log('unitree proxy error', err);
          });
          proxy.on('proxyReq', (proxyReq, req, _res) => {
            console.log('Sending Unitree Request:', req.method, req.url);
          });
          proxy.on('proxyRes', (proxyRes, req, _res) => {
            console.log('Received Unitree Response:', proxyRes.statusCode, req.url);
          });
        },
      },
      '/simulation': {
        target: '',  // Will be set dynamically
        changeOrigin: true,
        configure: (proxy, _options) => {
          proxy.on('error', (err, _req, _res) => {
            console.log('proxy error', err);
          });
          proxy.on('proxyReq', (proxyReq, req, _res) => {
            console.log('Sending Simulation Request:', req.method, req.url);
          });
        },
      }
    },
    cors: true
  },
  define: {
    'process.env': process.env
  }
});
