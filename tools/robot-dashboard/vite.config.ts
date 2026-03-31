import path from 'node:path'
import { fileURLToPath } from 'node:url'
import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'
import electron from 'vite-plugin-electron'

const __dirname = path.dirname(fileURLToPath(import.meta.url))
const monorepoRoot = path.resolve(__dirname, '../..')

// https://vite.dev/config/
export default defineConfig({
  plugins: [
    react(),
    electron({
      entry: 'electron/main.ts',
    }),
  ],
  build: {
    outDir: 'dist',
  },
  server: {
    fs: {
      allow: [monorepoRoot],
    },
  },
})
