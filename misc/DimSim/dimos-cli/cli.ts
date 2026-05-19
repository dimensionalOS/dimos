#!/usr/bin/env -S deno run --allow-all --unstable-net

/**
 * DimSim CLI — 3D simulation, eval runner, dev server, and scene manager.
 *
 * Usage:
 *   dimsim setup                                            Download core assets
 *   dimsim scene install <name>                             Download a scene
 *   dimsim scene list                                       List scenes
 *   dimsim scene remove <name>                              Remove a scene
 *   dimsim dev   [--scene <name>] [--port <n>]              Dev server + browser
 *   dimsim eval create                                      Interactive eval wizard
 *   dimsim eval  [--headless] [--parallel N] [--render gpu] Headless CI evals
 *   dimsim agent [--nav-only]                               dimos Python agent
 */

import { resolve, dirname, fromFileUrl } from "@std/path";
import { startBridgeServer } from "./bridge/server.ts";
import { launchHeadless, launchMultiPage, type RenderMode } from "./headless/launcher.ts";
import { runEvals, runEvalsMultiPage, collectWorkflows, toJunitXml, type EvalResult } from "./eval/runner.ts";
import { getDimsimHome, getDistDir, setup, sceneInstall, sceneList, sceneRemove } from "./setup.ts";
import { loadSceneIndex, findObject, suggestObjects } from "./eval/scene-index.ts";
import { buildEval } from "./eval/builder.ts";

// Detect compiled binary: Deno.execPath() won't contain "deno" when compiled.
// When compiled or installed from JSR, local source paths don't exist.
const IS_COMPILED = !Deno.execPath().toLowerCase().includes("deno");
const IS_REMOTE = IS_COMPILED || !import.meta.url.startsWith("file:");

const CLI_DIR = IS_REMOTE ? null : dirname(fromFileUrl(import.meta.url));
const PROJECT_DIR = CLI_DIR ? resolve(CLI_DIR, "..") : null;
const LOCAL_DIST_DIR = PROJECT_DIR ? resolve(PROJECT_DIR, "dist") : null;
const EVALS_DIR = PROJECT_DIR ? resolve(PROJECT_DIR, "evals") : `${getDimsimHome()}/evals`;
const DIMOS_VENV = PROJECT_DIR ? resolve(PROJECT_DIR, "../dimos/.venv/bin/python") : null;
const AGENT_PY = CLI_DIR ? resolve(CLI_DIR, "agent.py") : null;

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

/** Resolve distDir: use local dist/ if it exists (dev / vendored), build it from sources if not, else fall back to ~/.dimsim/dist/. */
async function resolveDistDir(): Promise<string> {
  if (LOCAL_DIST_DIR && PROJECT_DIR) {
    try {
      await Deno.stat(`${LOCAL_DIST_DIR}/index.html`);
      return LOCAL_DIST_DIR;
    } catch { /* not found — try to build from repo sources */ }

    if (await tryBuildFromSource(PROJECT_DIR, LOCAL_DIST_DIR)) {
      return LOCAL_DIST_DIR;
    }
  }

  const installed = getDistDir();
  try {
    await Deno.stat(`${installed}/index.html`);
    return installed;
  } catch { /* not found */ }

  console.error(`[dimsim] No dist/ found.`);
  console.error(`[dimsim] Run 'dimsim setup' to download core assets.`);
  if (!IS_REMOTE) {
    console.error(`[dimsim] Or build locally with 'npm run build'.`);
  }
  Deno.exit(1);
}

function printUsage() {
  console.log(`
DimSim CLI — 3D simulation + eval harness for dimos

Commands:
  dimsim setup                   Download core assets (~40MB)
  dimsim scene install <name>    Download a scene
  dimsim scene list              List available + installed scenes
  dimsim scene remove <name>     Remove a local scene
  dimsim dev   [options]         Dev server (open browser, optional eval)
  dimsim eval list               List installed eval workflows
  dimsim eval create             Interactive eval builder wizard
  dimsim eval  [options]         Run eval workflows (headless CI)
  dimsim list objects [options]   List scene objects (eval targets)
  dimsim build eval [options]    Generate eval from validated target
  dimsim agent [options]         Launch dimos Python agent

Setup:
  --local <path>                 Use local archive instead of downloading

Dev:
  --scene <name>                 Scene to load (default: apt)
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

List Objects:
  --scene <name>                   Scene to inspect (required)
  --search <term>                  Filter objects by name

Build Eval:
  --scene <name>                   Scene name (required)
  --target <object>                Target object name (required, validated)
  --threshold <m>                  Distance threshold (default: 2.0)
  --timeout <s>                    Timeout in seconds (default: 60)
  --task <prompt>                  Agent prompt (default: auto from target)
  --name <id>                      Eval name (default: slugified target)
  --env <name>                     Manifest environment (default: scene name)

Agent:
  --nav-only                     Nav stack only (no LLM agent)
  --venv <path>                  Python venv path (default: ../dimos/.venv/bin/python)

Environment:
  DIMSIM_HOME                    Override data dir (default: ~/.dimsim)
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
    if (IS_COMPILED) {
      // Version is read from the embedded deno.json at compile time
      try {
        const text = await Deno.readTextFile(new URL("./deno.json", import.meta.url));
        console.log(JSON.parse(text).version);
      } catch {
        console.log("0.1.31");  // fallback — updated at release time
      }
    } else {
      const metaUrl = new URL("./deno.json", import.meta.url);
      try {
        const resp = await fetch(metaUrl);
        const meta = await resp.json();
        console.log(meta.version);
      } catch {
        console.log("unknown");
      }
    }
    Deno.exit(0);
  }

  const port = parseInt(opts.port as string) || 8090;

  // ── Setup ───────────────────────────────────────────────────────────
  if (subcommand === "setup") {
    const local = opts.local;
    if (local === true) {
      console.error("[dimsim] --local requires a path: dimsim setup --local ./dimsim-core-v0.1.0.tar.gz");
      Deno.exit(1);
    }
    await setup(local as string | undefined);
    Deno.exit(0);
  }

  // ── Scene management ────────────────────────────────────────────────
  if (subcommand === "scene") {
    const action = Deno.args[1];
    const name = Deno.args[2];
    const sceneOpts = parseArgs(Deno.args.slice(2));

    if (action === "install" && name) {
      const local = sceneOpts.local;
      if (local === true) {
        console.error("[dimsim] --local requires a path: dimsim scene install apt --local ./scene-apt-v0.1.0.tar.gz");
        Deno.exit(1);
      }
      await sceneInstall(name, local as string | undefined);
    } else if (action === "list") {
      await sceneList();
    } else if (action === "remove" && name) {
      await sceneRemove(name);
    } else {
      console.log("Usage:");
      console.log("  dimsim scene install <name> [--local <path>]");
      console.log("  dimsim scene list");
      console.log("  dimsim scene remove <name>");
    }
    Deno.exit(0);
  }

  // ── List objects ────────────────────────────────────────────────────
  if (subcommand === "list") {
    const what = Deno.args[1];
    if (what === "objects") {
      const listOpts = parseArgs(Deno.args.slice(2));
      const sceneName = listOpts.scene as string;
      if (!sceneName) {
        console.error("[dimsim] --scene is required. Example: dimsim list objects --scene apt");
        Deno.exit(1);
      }

      const distDir = await resolveDistDir();
      const scenePath = `${distDir}/sims/${sceneName}.json`;
      try {
        await Deno.stat(scenePath);
      } catch {
        console.error(`[dimsim] Scene "${sceneName}" not found at ${scenePath}`);
        console.error(`[dimsim] Run 'dimsim scene install ${sceneName}' first.`);
        Deno.exit(1);
      }

      const index = loadSceneIndex(scenePath, sceneName);
      const search = listOpts.search as string | undefined;

      let filtered = index.objects;
      if (search) {
        const lower = search.toLowerCase();
        filtered = index.objects.filter(
          (o) => o.title.toLowerCase().includes(lower) || o.id.toLowerCase().includes(lower),
        );
        console.log(`\nObjects matching "${search}" in scene "${sceneName}" (${filtered.length}):\n`);
      } else {
        console.log(`\nObjects in scene "${sceneName}" (${filtered.length} titled assets):\n`);
      }

      if (filtered.length === 0) {
        console.log("  (none)");
      } else {
        const maxTitle = Math.min(45, Math.max(...filtered.map((o) => o.title.length)));
        for (const obj of filtered) {
          const t = obj.title.padEnd(maxTitle);
          console.log(`  ${t}  (${obj.position.x}, ${obj.position.y}, ${obj.position.z})`);
        }
      }
      console.log();
      Deno.exit(0);
    }
    console.log("Usage: dimsim list objects --scene <name> [--search <term>]");
    Deno.exit(1);
  }

  // ── Build eval ─────────────────────────────────────────────────────
  if (subcommand === "build") {
    const what = Deno.args[1];
    if (what === "eval") {
      const buildOpts = parseArgs(Deno.args.slice(2));
      const sceneName = buildOpts.scene as string;
      const target = buildOpts.target as string;

      if (!sceneName || !target) {
        console.error("[dimsim] --scene and --target are required.");
        console.error("Example: dimsim build eval --scene apt --target television");
        Deno.exit(1);
      }

      const distDir = await resolveDistDir();
      const scenePath = `${distDir}/sims/${sceneName}.json`;
      try {
        await Deno.stat(scenePath);
      } catch {
        console.error(`[dimsim] Scene "${sceneName}" not found at ${scenePath}`);
        console.error(`[dimsim] Run 'dimsim scene install ${sceneName}' first.`);
        Deno.exit(1);
      }

      try {
        const result = buildEval({
          scenePath,
          sceneName,
          target,
          threshold: buildOpts.threshold ? parseFloat(buildOpts.threshold as string) : undefined,
          timeout: buildOpts.timeout ? parseInt(buildOpts.timeout as string) : undefined,
          task: buildOpts.task as string | undefined,
          name: buildOpts.name as string | undefined,
          env: buildOpts.env as string | undefined,
          evalsDir: EVALS_DIR,
        });

        console.log(`\nCreated eval: ${result.filePath}`);
        console.log(`  Task:      "${result.task}"`);
        console.log(`  Target:    ${result.targetTitle} (${result.targetPosition.x}, ${result.targetPosition.y}, ${result.targetPosition.z})`);
        console.log(`  Threshold: ${result.threshold}m`);
        console.log(`  Timeout:   ${result.timeout}s`);
        console.log(`\nRun: dimsim eval --connect --env ${result.env} --workflow ${result.workflowName}\n`);
      } catch (err: any) {
        console.error(`[dimsim] ${err.message}`);
        Deno.exit(1);
      }
      Deno.exit(0);
    }
    console.log("Usage: dimsim build eval --scene <name> --target <object> [options]");
    Deno.exit(1);
  }

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
      console.log("\nNo evals installed. Run 'dimsim setup' or 'dimsim eval create' first.\n");
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

  // ── Eval create (interactive wizard) ─────────────────────────────────
  if (subcommand === "eval" && Deno.args[1] === "create") {
    // ANSI helpers
    const c = {
      bold: (s: string) => `\x1b[1m${s}\x1b[0m`,
      cyan: (s: string) => `\x1b[36m${s}\x1b[0m`,
      green: (s: string) => `\x1b[32m${s}\x1b[0m`,
      yellow: (s: string) => `\x1b[33m${s}\x1b[0m`,
      red: (s: string) => `\x1b[31m${s}\x1b[0m`,
      dim: (s: string) => `\x1b[2m${s}\x1b[0m`,
    };

    const distDir = await resolveDistDir();
    const simsDir = `${distDir}/sims`;

    // ── 1. Pick scene ──────────────────────────────────────────────────
    const installed: string[] = [];
    try {
      for await (const entry of Deno.readDir(simsDir)) {
        if (entry.name.endsWith(".json") && entry.name !== "manifest.json") {
          installed.push(entry.name.replace(".json", ""));
        }
      }
    } catch { /* no sims */ }
    installed.sort();

    if (installed.length === 0) {
      console.error(c.red("No scenes installed. Run 'dimsim scene install <name>' first."));
      Deno.exit(1);
    }

    console.log(`\n${c.bold("  Create Eval Workflow")}\n`);
    console.log(c.cyan("  Installed scenes:"));
    installed.forEach((s, i) => console.log(`    ${c.dim(`${i + 1}.`)} ${s}`));

    let sceneName = "";
    let scenePath = "";
    while (true) {
      const input = prompt(`\n  ${c.cyan("Scene")} ${c.dim(`[${installed[0]}]`)}:`) || installed[0];
      const resolved = installed.includes(input)
        ? input
        : installed[parseInt(input) - 1];
      if (resolved) {
        sceneName = resolved;
        scenePath = `${simsDir}/${sceneName}.json`;
        try {
          await Deno.stat(scenePath);
          console.log(`  ${c.green("→")} ${sceneName}`);
          break;
        } catch { /* fall through */ }
      }
      console.log(c.yellow(`  "${input}" not found. Pick a number or name from the list above.`));
    }

    // ── 2. Pick rubric ─────────────────────────────────────────────────
    const rubricChoices = [
      { key: "objectDistance", label: "objectDistance", desc: "agent must reach a target object" },
      { key: "llmJudge", label: "llmJudge", desc: "VLM judges success from screenshots" },
      { key: "groundTruth", label: "groundTruth", desc: "check spatial ground truth conditions" },
    ];

    console.log(`\n${c.cyan("  Rubric types:")}`);
    rubricChoices.forEach((r, i) =>
      console.log(`    ${c.dim(`${i + 1}.`)} ${c.bold(r.label)} ${c.dim(`— ${r.desc}`)}`)
    );

    let rubric = "";
    while (true) {
      const input = prompt(`\n  ${c.cyan("Rubric")} ${c.dim("[1]")}:`) || "1";
      const byNum = rubricChoices[parseInt(input) - 1];
      const byName = rubricChoices.find((r) => r.key === input);
      const match = byNum || byName;
      if (match) {
        rubric = match.key;
        console.log(`  ${c.green("→")} ${match.label}`);
        break;
      }
      console.log(c.yellow(`  Invalid choice. Enter 1-3 or a rubric name.`));
    }

    // ── 3. Pick target object (objectDistance needs it) ─────────────────
    const needsTarget = rubric === "objectDistance";
    const index = loadSceneIndex(scenePath, sceneName);
    let target = "";
    let matchedObj: ReturnType<typeof findObject> = null;

    if (needsTarget) {
      console.log(`\n${c.cyan(`  Objects in "${sceneName}"`)} ${c.dim(`(${index.objects.length})`)}:`);
      const sample = index.objects.slice(0, 20);
      for (const obj of sample) {
        console.log(`    ${obj.title}`);
      }
      if (index.objects.length > 20) {
        console.log(c.dim(`    ... and ${index.objects.length - 20} more (dimsim list objects --scene ${sceneName})`));
      }

      while (true) {
        const input = prompt(`\n  ${c.cyan("Target object")}:`);
        if (!input) {
          console.log(c.yellow("  Target is required for objectDistance rubric."));
          continue;
        }
        matchedObj = findObject(input, index);
        if (matchedObj) {
          target = input;
          console.log(`  ${c.green("→")} "${matchedObj.title}" at (${matchedObj.position.x}, ${matchedObj.position.y}, ${matchedObj.position.z})`);
          break;
        }
        const suggestions = suggestObjects(input, index);
        if (suggestions.length > 0) {
          console.log(c.yellow(`  No match for "${input}". Similar: ${suggestions.join(", ")}`));
        } else {
          console.log(c.yellow(`  No match for "${input}". Try 'dimsim list objects --scene ${sceneName}'.`));
        }
      }
    }

    // ── 4. Task prompt ─────────────────────────────────────────────────
    const defaultTask = needsTarget && matchedObj
      ? `Go to the ${matchedObj.title}`
      : "";
    let task = "";
    while (true) {
      const suffix = defaultTask ? ` ${c.dim(`[${defaultTask}]`)}` : "";
      const input = prompt(`\n  ${c.cyan("Task prompt")}${suffix}:`) || defaultTask;
      if (input) {
        task = input;
        break;
      }
      console.log(c.yellow("  Task prompt is required."));
    }

    // ── 5. Rubric-specific config ──────────────────────────────────────
    let threshold = 2.0;
    let llmPrompt = "";

    if (rubric === "objectDistance") {
      while (true) {
        const input = prompt(`  ${c.cyan("Distance threshold")} ${c.dim("[2.0m]")}:`) || "2.0";
        const val = parseFloat(input);
        if (!isNaN(val) && val > 0) {
          threshold = val;
          break;
        }
        console.log(c.yellow("  Enter a positive number (meters)."));
      }
    } else if (rubric === "llmJudge") {
      const defaultJudge = `Did the agent successfully complete: ${task}?`;
      llmPrompt = prompt(`  ${c.cyan("LLM judge prompt")} ${c.dim(`[${defaultJudge}]`)}:`) || defaultJudge;
    }

    // ── 6. Eval name ───────────────────────────────────────────────────
    const slug = (s: string) => s.toLowerCase().replace(/[^a-z0-9]+/g, "-").replace(/^-|-$/g, "");
    const defaultName = target ? slug(target) : slug(task.slice(0, 40));
    const name = prompt(`  ${c.cyan("Eval name")} ${c.dim(`[${defaultName}]`)}:`) || defaultName;

    // ── 7. Timeout ─────────────────────────────────────────────────────
    let timeout = 60;
    while (true) {
      const input = prompt(`  ${c.cyan("Timeout")} ${c.dim("[60s]")}:`) || "60";
      const val = parseInt(input);
      if (!isNaN(val) && val > 0) {
        timeout = val;
        break;
      }
      console.log(c.yellow("  Enter a positive number (seconds)."));
    }

    // ── Build & write ──────────────────────────────────────────────────
    const successCriteria: Record<string, unknown> = {};
    if (rubric === "objectDistance") {
      successCriteria.objectDistance = { object: "agent", target, thresholdM: threshold };
    } else if (rubric === "llmJudge") {
      successCriteria.llmJudge = { prompt: llmPrompt };
    } else if (rubric === "groundTruth") {
      successCriteria.groundTruth = {};
    }

    const env = sceneName;
    const workflow = {
      name,
      environment: env,
      task,
      startPose: { x: 0, y: 0.5, z: 3, yaw: 0 },
      timeoutSec: timeout,
      successCriteria,
    };

    const envDir = `${EVALS_DIR}/${env}`;
    try { Deno.mkdirSync(envDir, { recursive: true }); } catch { /* exists */ }
    const filePath = `${envDir}/${name}.json`;
    Deno.writeTextFileSync(filePath, JSON.stringify(workflow, null, 2) + "\n");

    console.log(`\n  ${c.green("Created:")} ${filePath}`);
    console.log(`\n  ${c.cyan("Run it:")}`);
    console.log(`    dimsim eval --connect --env ${env} --workflow ${name}`);
    console.log(`    dimsim eval --headless --env ${env} --workflow ${name}\n`);
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
