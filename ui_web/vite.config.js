import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';

// Carga variables prefijadas con VITE_
export default defineConfig({
  plugins: [react()],
  server: {
    port: 5173,
    open: true
  },
  define: {
    __ROSBRIDGE_URL__: JSON.stringify(
      process.env.VITE_ROSBRIDGE_URL || 'ws://localhost:9090'
    ),
    __API_URL__: JSON.stringify(process.env.VITE_API_URL || 'http://localhost:5000')
  }
});
