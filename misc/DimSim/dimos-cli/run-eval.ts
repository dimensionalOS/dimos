#!/usr/bin/env -S deno run --allow-all --unstable-net
/**
 * Run eval against an already-running bridge server.
 * Usage: deno run --allow-all --unstable-net dimos-cli/run-eval.ts [--workflow reach-vase] [--port 8090]
 */
import { resolve, dirname, fromFileUrl } from "@std/path";
import { runEvals } from "./eval/runner.ts";

const CLI_DIR = dirname(fromFileUrl(import.meta.url));
const EVALS_DIR = resolve(CLI_DIR, "../evals");

const args = Deno.args;
let port = 8090;
let workflow: string | undefined;
let env: string | undefined;

for (let i = 0; i < args.length; i++) {
  if (args[i] === "--port" && args[i + 1]) port = parseInt(args[++i]);
  if (args[i] === "--workflow" && args[i + 1]) workflow = args[++i];
  if (args[i] === "--env" && args[i + 1]) env = args[++i];
}

const wsUrl = `ws://localhost:${port}`;
const manifestPath = resolve(EVALS_DIR, "manifest.json");

console.log(`[eval] Connecting to bridge at ${wsUrl}`);
console.log(`[eval] Workflow: ${workflow || "all"}, Env: ${env || "all"}`);

const results = await runEvals({
  wsUrl,
  manifestPath,
  filterEnv: env,
  filterWorkflow: workflow,
  outputFormat: "json",
});

const passed = results.filter((r) => r.pass).length;
const failed = results.length - passed;
console.log(`\n[eval] Done: ${passed} passed, ${failed} failed`);
Deno.exit(failed > 0 ? 1 : 0);
