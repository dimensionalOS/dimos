#!/usr/bin/env -S deno run --allow-all --unstable-net

/**
 * DimSim CLI — 3D simulation + eval runner + dev server.
 *
 * Usage:
 *   dimsim dev   [--scene <name>] [--port <n>]              Dev server + browser
 *   dimsim eval  [--headless] [--parallel N] [--render gpu] Headless CI evals
 *   dimsim eval list                                        List eval workflows
 *   dimsim agent [--nav-only]                               dimos Python agent
 */

import { resolve, dirname, fromFileUrl } from "@std/path";
import { startBridgeServer } from "./bridge/server.ts";
import { launchHeadless, launchMultiPage, type RenderMode } from "./headless/launcher.ts";
import { runEvals, runEvalsMultiPage, collectWorkflows, toJunitXml, type EvalResult } from "./eval/runner.ts";

const CLI_DIR = dirname(fromFileUrl(import.meta.url));
const PROJECT_DIR = resolve(CLI_DIR, "..");
const LOCAL_DIST_DIR = resolve(PROJECT_DIR, "dist");
const EVALS_DIR = resolve(PROJECT_DIR, "evals");
const DIMOS_VENV = resolve(PROJECT_DIR, "../../.venv/bin/python");
const AGENT_PY = resolve(CLI_DIR, "agent.py");

/**
 * Build dist/ from the repo's own sources using Deno's npm compat.
 * Everything needed is already in-tree: src/ (engine), public/ (scenes,
 * agent-model, logo). Vite bundles src/ and copies public/ verbatim, so
 * no assets need to be downloaded from GitHub releases, and npm/Node are
 * not required — Deno runs Vite directly.
 *
 * Important for the vendored layout (misc/DimSim/ inside dimos): dist/ is
 * gitignored and never committed, so on first run we have to materialize
 * it ourselves rather than asking the user to `npm run build`.
 *
 * --no-lock keeps the repo's deno.lock (which tracks only JSR deps for
 * the CLI) from being polluted with the frontend's npm dep graph.
 */
async function tryBuildFromSource(
  projectDir: string,
  distDir: string,
): Promise<boolean> {
  let viteSpec = "npm:vite@^5";
  try {
    const pkg = JSON.parse(await Deno.readTextFile(`${projectDir}/package.json`));
    const v = pkg.devDependencies?.vite ?? pkg.dependencies?.vite;
    if (!v) return false;
    viteSpec = `npm:vite@${v}`;
  } catch {
    return false;
  }

  // node_modules/ — install from package.json if missing. Vite resolves
  // bare imports (three, rapier, etc.) via node_modules at build time.
  try {
    await Deno.stat(`${projectDir}/node_modules`);
  } catch {
    console.log(`[dimsim] node_modules/ not found — running 'deno install' (one-time)...`);
    const install = new Deno.Command(Deno.execPath(), {
      args: ["install", "--no-lock"], cwd: projectDir,
      stdout: "inherit", stderr: "inherit",
    }).spawn();
    const s = await install.status;
    if (!s.success) {
      console.error(`[dimsim] deno install failed (exit ${s.code}).`);
      return false;
    }
  }

  console.log(`[dimsim] Building frontend with Vite...`);
  const build = new Deno.Command(Deno.execPath(), {
    args: ["run", "-A", "--no-lock", viteSpec, "build"],
    cwd: projectDir,
    stdout: "inherit", stderr: "inherit",
  }).spawn();
  const bs = await build.status;
  if (!bs.success) {
    console.error(`[dimsim] vite build failed (exit ${bs.code}).`);
    return false;
  }

  try {
    await Deno.stat(`${distDir}/index.html`);
    return true;
  } catch {
    console.error(`[dimsim] Build completed but ${distDir}/index.html is missing.`);
    return false;
  }
}

/** Resolve distDir: use local dist/ if present, otherwise build it from source. */
async function resolveDistDir(): Promise<string> {
  try {
    await Deno.stat(`${LOCAL_DIST_DIR}/index.html`);
    return LOCAL_DIST_DIR;
  } catch { /* not found — fall through to build */ }

  if (await tryBuildFromSource(PROJECT_DIR, LOCAL_DIST_DIR)) {
    return LOCAL_DIST_DIR;
  }

  console.error(`[dimsim] No dist/ found and tryBuildFromSource() failed.`);
  console.error(`[dimsim] Build manually:  cd ${PROJECT_DIR} && npm run build`);
  Deno.exit(1);
}

function printUsage() {
  console.log(`
DimSim CLI — 3D simulation + eval harness for dimos

Commands:
  dimsim dev   [options]         Dev server (open browser, optional eval)
  dimsim eval list               List installed eval workflows
  dimsim eval  [options]         Run eval workflows (headless CI)
  dimsim agent [options]         Launch dimos Python agent

Dev:
  --scene <name>                 Scene to load (default: apartment)
  --port <n>                     Server port (default: 8090)
  --headless                     Launch headless browser (no GUI)
  --render gpu|cpu               Render mode for headless (default: gpu)
  --channels <n>                 Number of parallel browser pages (multi-instance)
  --eval <workflow>              Run eval after browser connects
  --env <name>                   Environment filter
  --image-rate <ms>              Image publish interval in ms (default: 500 = 2 Hz)
  --lidar-rate <ms>              LiDAR publish interval in ms (default: 200 = 5 Hz)
  --odom-rate <ms>               Odom publish interval in ms (default: 20 = 50 Hz)
  --no-depth                     Disable depth image publishing
  --camera-fov <deg>             Camera FOV in degrees (default: 80)

Eval:
  --connect                      Connect to existing bridge (use with dimos)
  --headless                     Headless Chromium (required for CI)
  --parallel <n>                 N parallel browser pages (default: 1)
  --render gpu|cpu               gpu = Metal/ANGLE, cpu = SwiftShader (default: cpu)
  --env <name>                   Filter to environment
  --workflow <name>              Filter to workflow
  --output json|junit            Output format (default: json)
  --port <n>                     Bridge port (default: 8090)
  --timeout <ms>                 Engine init timeout (default: auto)

Agent:
  --nav-only                     Nav stack only (no LLM agent)
  --venv <path>                  Python venv path (default: ../../.venv/bin/python relative to misc/DimSim/)
`);
}

function parseArgs(args: string[]) {
  const opts: Record<string, string | boolean> = {};
  for (let i = 0; i < args.length; i++) {
    const arg = args[i];
    if (arg.startsWith("--")) {
      const key = arg.slice(2);
      const next = args[i + 1];
      if (next && !next.startsWith("--")) {
        opts[key] = next;
        i++;
      } else {
        opts[key] = true;
      }
    }
  }
  return opts;
}

async function main() {
  const subcommand = Deno.args[0];
  const opts = parseArgs(Deno.args.slice(1));

  if (!subcommand || subcommand === "help" || subcommand === "--help") {
    printUsage();
    Deno.exit(0);
  }

  if (subcommand === "--version" || subcommand === "version") {
    try {
      const text = await Deno.readTextFile(new URL("./deno.json", import.meta.url));
      console.log(JSON.parse(text).version);
    } catch {
      console.log("unknown");
    }
    Deno.exit(0);
  }

  const port = parseInt(opts.port as string) || 8090;

  // ── Dev ─────────────────────────────────────────────────────────────
  if (subcommand === "dev") {
    const distDir = await resolveDistDir();
    const scene = (opts.scene as string) || "apt";
    const headless = opts.headless === true;
    const render = ((opts.render as string) === "cpu" ? "cpu" : "gpu") as RenderMode;
    const numChannels = Math.max(1, parseInt(opts.channels as string) || 1);
    const evalWorkflow = opts.eval as string | undefined;

    // Sensor publish rates (ms) — overrides browser defaults
    const sensorRates: Record<string, number> = {};
    if (opts["image-rate"]) sensorRates.images = parseInt(opts["image-rate"] as string);
    if (opts["lidar-rate"]) sensorRates.lidar = parseInt(opts["lidar-rate"] as string);
    if (opts["odom-rate"]) sensorRates.odom = parseInt(opts["odom-rate"] as string);

    // Sensor enable/disable (depth only — color and lidar are essential)
    const sensorEnable: Record<string, boolean> = {};
    if (opts["no-depth"] === true) sensorEnable.depth = false;

    // Camera FOV
    const cameraFov = opts["camera-fov"] ? parseInt(opts["camera-fov"] as string) : undefined;

    // Build channel list for multi-instance mode
    const channels = numChannels > 1
      ? Array.from({ length: numChannels }, (_, i) => `page-${i}`)
      : undefined;

    console.log(`[dimsim] Dev mode — scene: ${scene}, port: ${port}${headless ? " (headless)" : ""}${channels ? ` (${numChannels} channels)` : ""}`);
    console.log(`[dimsim] Serving from: ${distDir}`);

    // LCM bridge is always active in dev mode (unlike eval --headless which disables it)
    startBridgeServer({
      port, distDir, scene, headless, channels,
      sensorRates: Object.keys(sensorRates).length > 0 ? sensorRates : undefined,
      sensorEnable: Object.keys(sensorEnable).length > 0 ? sensorEnable : undefined,
      cameraFov,
    });

    if (headless) {
      if (channels) {
        // Multi-page mode: open N browser pages in one Chromium instance
        console.log(`[dimsim] Launching headless browser with ${numChannels} pages...`);
        const url = `http://localhost:${port}`;
        await launchMultiPage({ url, numPages: numChannels, render, timeout: 120_000 });
        await new Promise((r) => setTimeout(r, 3000));
        console.log(`[dimsim] ${numChannels} headless pages ready. LCM bridge active.`);
      } else {
        console.log("[dimsim] Launching headless browser...");
        const url = `http://localhost:${port}`;
        // CPU rendering with SwiftShader is slow — scene + agent init takes
        // ~27s on CI Macs. Allow 90s by default; override via env var.
        const headlessTimeout = parseInt(
          Deno.env.get("DIMSIM_HEADLESS_TIMEOUT") || "90000",
        );
        await launchHeadless({ url, timeout: headlessTimeout, render });
        await new Promise((r) => setTimeout(r, 3000));
        console.log("[dimsim] Headless browser ready. LCM bridge active.");
      }
    } else {
      console.log(`[dimsim] Open http://localhost:${port} in your browser`);
    }

    if (evalWorkflow) {
      console.log(`[dimsim] Eval workflow: ${evalWorkflow}`);
      console.log("[dimsim] Waiting for browser to connect and load scene...\n");

      const wsUrl = `ws://localhost:${port}`;
      const manifestPath = resolve(EVALS_DIR, "manifest.json");

      const results = await runEvals({
        wsUrl,
        manifestPath,
        filterEnv: opts.env as string,
        filterWorkflow: evalWorkflow,
        outputFormat: "json",
      });

      const passed = results.filter((r) => r.pass).length;
      const failed = results.length - passed;
      console.log(`\n[dimsim] Eval done: ${passed} passed, ${failed} failed`);

      // Stay alive in dev mode (don't exit like headless eval does)
      console.log("[dimsim] Eval complete. Server still running. Press Ctrl+C to stop.");
    } else {
      console.log("[dimsim] Press Ctrl+C to stop.");
    }

    // Keep alive
    await new Promise(() => {});
  }

  // ── Agent ───────────────────────────────────────────────────────────
  if (subcommand === "agent") {
    if (IS_REMOTE && !opts.venv) {
      console.error(`[dimsim] Agent mode requires a local dimos install.`);
      console.error(`[dimsim] Pass --venv /path/to/python`);
      Deno.exit(1);
    }
    const pythonBin = (opts.venv as string) || DIMOS_VENV!;
    const navOnly = opts["nav-only"] === true;

    if (IS_REMOTE && !AGENT_PY) {
      console.error(`[dimsim] Agent mode is only available when running from source.`);
      Deno.exit(1);
    }

    // Verify python exists
    try {
      await Deno.stat(pythonBin);
    } catch {
      console.error(`[dimsim] dimos venv not found at: ${pythonBin}`);
      console.error(`[dimsim] Install dimos first, or pass --venv /path/to/python`);
      Deno.exit(1);
    }

    const cmd = [pythonBin, AGENT_PY!];
    if (navOnly) cmd.push("--nav-only");

    console.log(`[dimsim] Starting dimos agent${navOnly ? " (nav-only)" : ""}...`);
    console.log(`[dimsim] Python: ${pythonBin}`);

    const proc = new Deno.Command(cmd[0], {
      args: cmd.slice(1),
      stdin: "inherit",
      stdout: "inherit",
      stderr: "inherit",
      env: { ...Deno.env.toObject() },
    }).spawn();

    const status = await proc.status;
    Deno.exit(status.code);
  }

  // ── Eval list ───────────────────────────────────────────────────────
  if (subcommand === "eval" && Deno.args[1] === "list") {
    const evalsDir = EVALS_DIR;
    const envs: Map<string, string[]> = new Map();

    try {
      for await (const entry of Deno.readDir(evalsDir)) {
        if (!entry.isDirectory) continue;
        const workflows: string[] = [];
        for await (const file of Deno.readDir(`${evalsDir}/${entry.name}`)) {
          if (file.name.endsWith(".json") && file.name !== "manifest.json") {
            workflows.push(file.name.replace(".json", ""));
          }
        }
        if (workflows.length > 0) {
          workflows.sort();
          envs.set(entry.name, workflows);
        }
      }
    } catch {
      console.log("\nNo evals installed under ${EVALS_DIR}.\n");
      Deno.exit(0);
    }

    if (envs.size === 0) {
      console.log("\nNo eval workflows found.\n");
      Deno.exit(0);
    }

    const sorted = [...envs.entries()].sort((a, b) => a[0].localeCompare(b[0]));
    let total = 0;
    console.log("");
    for (const [env, workflows] of sorted) {
      console.log(`  \x1b[1m${env}\x1b[0m \x1b[2m(${workflows.length})\x1b[0m`);
      for (const w of workflows) {
        console.log(`    ${w}`);
        total++;
      }
    }
    console.log(`\n  \x1b[2m${total} workflow(s) across ${envs.size} environment(s)\x1b[0m\n`);
    Deno.exit(0);
  }


  // ── Eval ────────────────────────────────────────────────────────────
  if (subcommand === "eval") {
    const connectMode = opts.connect === true;
    const outputFormat = (opts.output as string) === "junit" ? "junit" : "json";
    const manifestPath = resolve(EVALS_DIR, "manifest.json");

    // --connect mode: just run the eval runner against an existing bridge
    if (connectMode) {
      const wsUrl = `ws://localhost:${port}`;
      console.log(`[dimsim] Connecting to existing bridge at ${wsUrl}...`);

      const results = await runEvals({
        wsUrl,
        manifestPath,
        filterEnv: opts.env as string,
        filterWorkflow: opts.workflow as string,
        outputFormat: outputFormat as "json" | "junit",
      });

      const passed = results.filter((r) => r.pass).length;
      const failed = results.length - passed;
      console.log(`\n[dimsim] Done: ${passed} passed, ${failed} failed, ${results.length} total`);
      Deno.exit(failed > 0 ? 1 : 0);
    }

    const distDir = await resolveDistDir();
    const headless = opts.headless === true;
    const scene = (opts.scene as string) || (opts.env as string) || "apt";
    const parallel = Math.max(1, parseInt(opts.parallel as string) || 1);
    const render = ((opts.render as string) === "gpu" ? "gpu" : "cpu") as RenderMode;
    const defaultTimeout = render === "cpu" ? 120000 : 30000;
    const timeout = parseInt(opts.timeout as string) || defaultTimeout;

    if (headless && parallel > 1) {
      const allWorkflows = collectWorkflows(
        manifestPath,
        opts.env as string,
        opts.workflow as string,
      );

      if (allWorkflows.length === 0) {
        console.log("[dimsim] No workflows match filter criteria.");
        Deno.exit(0);
      }

      const numPages = Math.min(parallel, allWorkflows.length);
      console.log(`[dimsim] Multi-page eval — ${allWorkflows.length} workflows across ${numPages} page(s)`);

      startBridgeServer({ port, distDir, scene, evalOnly: true });
      await new Promise((r) => setTimeout(r, 500));

      const url = `http://localhost:${port}`;
      const instance = await launchMultiPage({ url, numPages, timeout, render });
      await new Promise((r) => setTimeout(r, 2000));

      const allResults = await runEvalsMultiPage({
        wsUrl: `ws://localhost:${port}`,
        manifestPath,
        channels: instance.channels,
        filterEnv: opts.env as string,
        filterWorkflow: opts.workflow as string,
      });

      await instance.close();

      if (outputFormat === "junit") {
        console.log(toJunitXml(allResults));
      } else {
        console.log(JSON.stringify(allResults, null, 2));
      }

      const passed = allResults.filter((r) => r.pass).length;
      const failed = allResults.length - passed;
      console.log(`\n[dimsim] Done: ${passed} passed, ${failed} failed, ${allResults.length} total`);
      Deno.exit(failed > 0 ? 1 : 0);
    }

    // -- Single worker eval (sequential) -----------------------------------
    console.log(`[dimsim] Eval mode — headless: ${headless}, port: ${port}`);

    startBridgeServer({ port, distDir, scene, evalOnly: headless });
    await new Promise((r) => setTimeout(r, 500));

    const url = `http://localhost:${port}`;

    if (headless) {
      console.log("[dimsim] Launching headless browser...");
      const instance = await launchHeadless({ url, timeout, render });
      await new Promise((r) => setTimeout(r, 3000));

      const results = await runEvals({
        wsUrl: `ws://localhost:${port}`,
        manifestPath,
        filterEnv: opts.env as string,
        filterWorkflow: opts.workflow as string,
        outputFormat: outputFormat as "json" | "junit",
      });

      await instance.close();

      const failed = results.filter((r) => !r.pass).length;
      Deno.exit(failed > 0 ? 1 : 0);
    } else {
      console.log(`[dimsim] Open ${url} in your browser to start evals`);
      console.log("[dimsim] Press Ctrl+C to stop.");
      await new Promise(() => {});
    }
  }

  printUsage();
  Deno.exit(1);
}

main();
