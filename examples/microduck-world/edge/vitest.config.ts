import { cloudflareTest } from "@cloudflare/vitest-plugin";
import { defineConfig } from "vitest/config";
export default defineConfig({
  plugins: [cloudflareTest({ wrangler: { configPath: "./wrangler.jsonc" }, miniflare: { bindings: { GITHUB_CLIENT_SECRET: "test-only-github-secret", HOST_SECRET: "a".repeat(64) } } })],
  test: { include: ["test/**/*.test.ts"] },
});
