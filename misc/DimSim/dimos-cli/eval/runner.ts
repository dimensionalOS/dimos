/**
 * Eval Runner — Deno-side orchestrator that drives eval workflows.
 *
 * Connects to the bridge server via WebSocket and sends commands to the
 * browser's EvalHarness: load environment, start workflow, collect results.
 *
 * Two modes:
 *   runEvals()          — Sequential: one page, one workflow at a time
 *   runEvalsMultiPage() — Parallel: N pages in one browser, channel-routed
 */

export interface EvalResult {
  name: string;
  environment: string;
  reason: string;
  durationMs: number;
  rubricScores: Record<string, unknown>;
  pass: boolean;
}

export interface RunEvalOptions {
  wsUrl: string;
  manifestPath: string;
  filterEnv?: string;
  filterWorkflow?: string;
  /** Run only these specific workflow names (overrides filterWorkflow). */
  filterWorkflows?: string[];
  outputFormat?: "json" | "junit";
}

export interface WorkflowEntry {
  env: string;
  scene: string;
  workflowPath: string;
  workflowName: string;
}

/** Collect workflows from manifest, applying filters. */
export function collectWorkflows(manifestPath: string, filterEnv?: string, filterWorkflow?: string, filterWorkflows?: string[]): WorkflowEntry[] {
  const manifestText = Deno.readTextFileSync(manifestPath);
  const manifest = JSON.parse(manifestText);
  const result: WorkflowEntry[] = [];

  for (const env of manifest.environments) {
    if (filterEnv && env.name !== filterEnv) continue;
    for (const wfName of env.workflows) {
      if (filterWorkflows) {
        if (!filterWorkflows.includes(wfName)) continue;
      } else if (filterWorkflow && wfName !== filterWorkflow) {
        continue;
      }
      const dir = new URL(`../../evals/${env.name}/`, import.meta.url).pathname;
      result.push({
        env: env.name,
        scene: env.scene,
        workflowPath: `${dir}${wfName}.json`,
        workflowName: wfName,
      });
    }
  }
  return result;
}

export async function runEvals(options: RunEvalOptions): Promise<EvalResult[]> {
  const { wsUrl, manifestPath, filterEnv, filterWorkflow, filterWorkflows, outputFormat } = options;

  const workflowsToRun = collectWorkflows(manifestPath, filterEnv, filterWorkflow, filterWorkflows);

  if (workflowsToRun.length === 0) {
    console.log("[runner] No workflows match filter criteria.");
    return [];
  }

  console.log(`[runner] Running ${workflowsToRun.length} workflow(s)...`);

  // Connect to bridge WebSocket
  const ws = new WebSocket(wsUrl);
  ws.binaryType = "arraybuffer";

  await new Promise<void>((resolve, reject) => {
    const timeout = setTimeout(() => reject(new Error("WebSocket connect timeout")), 30000);
    ws.onopen = () => { clearTimeout(timeout); resolve(); };
    ws.onerror = (e) => { clearTimeout(timeout); reject(new Error(`WebSocket connection failed: ${e}`)); };
  });

  console.log("[runner] Connected to bridge");

  // Helper: send command and wait for response
  function sendAndWait(cmd: Record<string, unknown>, responseType: string, timeoutMs = 60000): Promise<Record<string, unknown>> {
    return new Promise((resolve, reject) => {
      const timeout = setTimeout(() => reject(new Error(`Timeout waiting for ${responseType}`)), timeoutMs);

      const handler = (event: MessageEvent) => {
        if (typeof event.data !== "string") return;
        try {
          const msg = JSON.parse(event.data);
          if (msg.type === responseType) {
            clearTimeout(timeout);
            ws.removeEventListener("message", handler);
            resolve(msg);
          }
        } catch { /* not JSON */ }
      };

      ws.addEventListener("message", handler);
      ws.send(JSON.stringify(cmd));
    });
  }

  // Wait for the browser eval harness to be alive (ping/pong handshake)
  console.log("[runner] Waiting for browser eval harness...");
  const harnessTimeout = 60000;
  const harnessStart = Date.now();
  let harnessReady = false;
  while (Date.now() - harnessStart < harnessTimeout) {
    try {
      await sendAndWait({ type: "ping" }, "pong", 3000);
      harnessReady = true;
      break;
    } catch {
      // No response yet — browser not connected or harness not initialized
      await new Promise((r) => setTimeout(r, 1000));
    }
  }
  if (!harnessReady) {
    console.error("[runner] Timeout waiting for browser eval harness. Is the browser open?");
    ws.close();
    return [];
  }
  console.log("[runner] Browser eval harness connected!");

  const results: EvalResult[] = [];
  let currentScene = "";

  for (const wf of workflowsToRun) {
    // Load environment if different from current
    if (wf.scene !== currentScene) {
      console.log(`[runner] Loading environment: ${wf.env} (scene: ${wf.scene})`);
      await sendAndWait({ type: "loadEnv", scene: wf.scene }, "envReady", 120000);
      currentScene = wf.scene;
      // Wait for physics to settle
      await new Promise((r) => setTimeout(r, 2000));
    }

    // Load workflow definition
    const wfText = await Deno.readTextFile(wf.workflowPath);
    const workflow = JSON.parse(wfText);

    console.log(`[runner] Starting workflow: ${wf.workflowName} — "${workflow.task}"`);

    // Start workflow and wait for completion
    const timeoutMs = (workflow.timeoutSec || 120) * 1000 + 30000; // +30s buffer for slow renderers
    const result = await sendAndWait(
      { type: "startWorkflow", workflow },
      "workflowComplete",
      timeoutMs,
    ) as Record<string, unknown>;

    const scores = result.rubricScores as Record<string, { pass?: boolean }> || {};
    const allPass = Object.values(scores).every((s) => s.pass !== false);

    const evalResult: EvalResult = {
      name: wf.workflowName,
      environment: wf.env,
      reason: result.reason as string,
      durationMs: result.durationMs as number,
      rubricScores: scores,
      pass: allPass,
    };

    results.push(evalResult);

    const status = allPass ? "PASS" : "FAIL";
    console.log(`[runner] ${status}: ${wf.workflowName} (${evalResult.durationMs}ms)`);
  }

  ws.close();

  // Output results
  if (outputFormat === "junit") {
    const xml = toJunitXml(results);
    console.log(xml);
  } else {
    console.log(JSON.stringify(results, null, 2));
  }

  // Summary
  const passed = results.filter((r) => r.pass).length;
  const failed = results.length - passed;
  console.log(`\n[runner] Done: ${passed} passed, ${failed} failed, ${results.length} total`);

  return results;
}

// ── Multi-page parallel runner ────────────────────────────────────────────

export interface RunEvalsMultiPageOptions {
  wsUrl: string;
  manifestPath: string;
  /** Channel IDs matching the browser pages (e.g. ["page-0", "page-1"]) */
  channels: string[];
  filterEnv?: string;
  filterWorkflow?: string;
}

/**
 * Run eval workflows in parallel across multiple browser pages within a
 * single browser instance.  One WebSocket connection to the bridge; commands
 * are routed to pages via a `channel` field that each page's EvalHarness
 * filters on.
 */
export async function runEvalsMultiPage(options: RunEvalsMultiPageOptions): Promise<EvalResult[]> {
  const { wsUrl, manifestPath, channels, filterEnv, filterWorkflow } = options;
  const numPages = channels.length;

  const allWorkflows = collectWorkflows(manifestPath, filterEnv, filterWorkflow);
  if (allWorkflows.length === 0) {
    console.log("[runner] No workflows match filter criteria.");
    return [];
  }

  console.log(`[runner] Multi-page: ${allWorkflows.length} workflow(s) across ${numPages} page(s)`);

  // Connect to bridge
  const ws = new WebSocket(wsUrl);
  ws.binaryType = "arraybuffer";

  await new Promise<void>((resolve, reject) => {
    const timeout = setTimeout(() => reject(new Error("WebSocket connect timeout")), 30000);
    ws.onopen = () => { clearTimeout(timeout); resolve(); };
    ws.onerror = (e) => { clearTimeout(timeout); reject(new Error(`WebSocket connection failed: ${e}`)); };
  });

  console.log("[runner] Connected to bridge");

  // Channel-aware sendAndWait: matches on BOTH message type AND channel
  function sendAndWait(
    cmd: Record<string, unknown>,
    responseType: string,
    channel: string,
    timeoutMs = 60000,
  ): Promise<Record<string, unknown>> {
    return new Promise((resolve, reject) => {
      const timer = setTimeout(
        () => reject(new Error(`Timeout waiting for ${responseType} on ${channel}`)),
        timeoutMs,
      );

      const handler = (event: MessageEvent) => {
        if (typeof event.data !== "string") return;
        try {
          const msg = JSON.parse(event.data);
          if (msg.type === responseType && msg.channel === channel) {
            clearTimeout(timer);
            ws.removeEventListener("message", handler);
            resolve(msg);
          }
        } catch { /* not JSON */ }
      };

      ws.addEventListener("message", handler);
      ws.send(JSON.stringify({ ...cmd, channel }));
    });
  }

  // Wait for all pages to be ready (ping/pong per channel)
  console.log("[runner] Waiting for all pages to be ready...");
  for (const ch of channels) {
    const start = Date.now();
    let ready = false;
    while (Date.now() - start < 60000) {
      try {
        await sendAndWait({ type: "ping" }, "pong", ch, 3000);
        ready = true;
        break;
      } catch {
        await new Promise((r) => setTimeout(r, 1000));
      }
    }
    if (!ready) {
      console.error(`[runner] Timeout waiting for page ${ch}`);
      ws.close();
      return [];
    }
    console.log(`[runner] Page ${ch} ready`);
  }

  // Distribute workflows round-robin across pages
  const batches: WorkflowEntry[][] = Array.from({ length: numPages }, () => []);
  allWorkflows.forEach((wf, i) => batches[i % numPages].push(wf));

  // Run each page's batch concurrently
  const pagePromises = batches.map(async (batch, i) => {
    const ch = channels[i];
    const tag = `[${ch}]`;
    const pageResults: EvalResult[] = [];
    let currentScene = "";

    for (const wf of batch) {
      try {
        // Load scene if needed
        if (wf.scene !== currentScene) {
          console.log(`${tag} Loading environment: ${wf.env} (scene: ${wf.scene})`);
          await sendAndWait({ type: "loadEnv", scene: wf.scene }, "envReady", ch, 120000);
          currentScene = wf.scene;
          await new Promise((r) => setTimeout(r, 2000));
        }

        const wfText = await Deno.readTextFile(wf.workflowPath);
        const workflow = JSON.parse(wfText);

        console.log(`${tag} Starting: ${wf.workflowName} — "${workflow.task}"`);

        const timeoutMs = (workflow.timeoutSec || 120) * 1000 + 30000;
        const result = await sendAndWait(
          { type: "startWorkflow", workflow },
          "workflowComplete",
          ch,
          timeoutMs,
        ) as Record<string, unknown>;

        const scores = result.rubricScores as Record<string, { pass?: boolean }> || {};
        const allPass = Object.values(scores).every((s) => s.pass !== false);

        const evalResult: EvalResult = {
          name: wf.workflowName,
          environment: wf.env,
          reason: result.reason as string,
          durationMs: result.durationMs as number,
          rubricScores: scores,
          pass: allPass,
        };
        pageResults.push(evalResult);

        const status = allPass ? "PASS" : "FAIL";
        console.log(`${tag} ${status}: ${wf.workflowName} (${evalResult.durationMs}ms)`);
      } catch (err) {
        console.error(`${tag} Error on ${wf.workflowName}: ${err}`);
        pageResults.push({
          name: wf.workflowName,
          environment: wf.env,
          reason: `page error: ${err}`,
          durationMs: 0,
          rubricScores: {},
          pass: false,
        });
      }
    }

    return pageResults;
  });

  const allResults = (await Promise.all(pagePromises)).flat();
  ws.close();

  // Summary
  const passed = allResults.filter((r) => r.pass).length;
  const failed = allResults.length - passed;
  console.log(`\n[runner] Done: ${passed} passed, ${failed} failed, ${allResults.length} total`);

  return allResults;
}

// ── JUnit XML output ──────────────────────────────────────────────────────

export function toJunitXml(results: EvalResult[]): string {
  const totalTime = results.reduce((s, r) => s + r.durationMs, 0) / 1000;
  const failures = results.filter((r) => !r.pass).length;

  let xml = `<?xml version="1.0" encoding="UTF-8"?>\n`;
  xml += `<testsuites tests="${results.length}" failures="${failures}" time="${totalTime.toFixed(1)}">\n`;
  xml += `  <testsuite name="dimsim-evals" tests="${results.length}" failures="${failures}">\n`;

  for (const r of results) {
    const time = (r.durationMs / 1000).toFixed(1);
    xml += `    <testcase name="${r.name}" classname="${r.environment}" time="${time}"`;
    if (r.pass) {
      xml += ` />\n`;
    } else {
      xml += `>\n`;
      xml += `      <failure message="${r.reason}">${JSON.stringify(r.rubricScores)}</failure>\n`;
      xml += `    </testcase>\n`;
    }
  }

  xml += `  </testsuite>\n`;
  xml += `</testsuites>\n`;
  return xml;
}
