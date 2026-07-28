/**
 * Headless Launcher — Playwright-based headless Chromium for CI/CD evals.
 *
 * Rendering modes:
 *   gpu  — Metal/ANGLE (macOS, fast, max ~3 parallel pages)
 *   cpu  — SwiftShader (Linux CI, no GPU needed, sequential only on <16 cores)
 */

import { chromium, type Browser, type Page } from "playwright";

export type RenderMode = "gpu" | "cpu";

export interface LaunchOptions {
  url: string;
  timeout?: number;
  render?: RenderMode;
}

export interface HeadlessInstance {
  browser: Browser;
  page: Page;
  close: () => Promise<void>;
}

export interface MultiPageInstance {
  browser: Browser;
  pages: Page[];
  channels: string[];
  close: () => Promise<void>;
}

export interface MultiPageOptions {
  url: string;
  numPages: number;
  timeout?: number;
  render?: RenderMode;
}

// ── Chrome flags per render mode ──────────────────────────────────────────

const GPU_ARGS = [
  "--headless=new",
  "--no-sandbox",
  "--disable-setuid-sandbox",
  "--disable-features=SkiaGraphite",
  "--enable-webgl",
  "--enable-webgl2",
  "--ignore-gpu-blocklist",
  "--enable-gpu",
  "--use-gl=angle",
  "--use-angle=metal",
  "--in-process-gpu",
  "--disable-gpu-sandbox",
  // Prevent Chrome from throttling timers in headless/background mode
  "--disable-background-timer-throttling",
  "--disable-backgrounding-occluded-windows",
  "--disable-renderer-backgrounding",
];

const CPU_ARGS = [
  "--headless=new",
  "--no-sandbox",
  "--disable-setuid-sandbox",
  "--disable-features=SkiaGraphite",
  "--enable-webgl",
  "--enable-webgl2",
  "--use-gl=angle",
  "--use-angle=swiftshader",
  "--enable-unsafe-swiftshader",
  "--disable-gpu",
  // Prevent Chrome from throttling timers in headless/background mode
  "--disable-background-timer-throttling",
  "--disable-backgrounding-occluded-windows",
  "--disable-renderer-backgrounding",
];

// Default: bundled Chromium (works on Linux + macOS in CPU mode with SwiftShader).
// Set DIMSIM_CHROME_CHANNEL=chrome to use system Google Chrome (needed for hardware
// WebGL on macOS — bundled Chromium ships without the full Metal/ANGLE GPU stack).
const LAUNCH_CHANNEL = Deno.env.get("DIMSIM_CHROME_CHANNEL") || undefined;

// ── Helpers ───────────────────────────────────────────────────────────────

/** Filter noisy browser console output — only forward errors, warnings, and eval/bridge logs. */
function hookPageConsole(page: Page, tag: string): void {
  const verbose = Deno.env.get("DIMSIM_VERBOSE") === "1";
  page.on("console", (msg) => {
    const type = msg.type();
    const text = msg.text();
    if (!verbose) {
      if (text.includes("Texture marked for update") || text.includes("Failed to load resource") ||
          text.includes("GPU stall due to ReadPixels") || text.includes("Automatic fallback to software WebGL") ||
          text.includes("GroupMarkerNotSet")) return;
    }
    if (type === "error") console.error(`${tag} ${text}`);
    else if (type === "warning") console.warn(`${tag} ${text}`);
    else if (verbose || text.startsWith("[eval]") || text.startsWith("[DimosBridge]")) {
      console.log(`${tag} ${text}`);
    }
  });
}

function getViewport(render: RenderMode) {
  // CPU mode: tiny viewport — SwiftShader renders every pixel on CPU
  return render === "cpu"
    ? { width: 320, height: 240 }
    : { width: 1280, height: 720 };
}

// ── Single-page launcher ─────────────────────────────────────────────────

export async function launchHeadless(options: LaunchOptions): Promise<HeadlessInstance> {
  const { url, timeout = 30000, render = "cpu" } = options;
  const args = render === "gpu" ? GPU_ARGS : CPU_ARGS;

  console.log(`[headless] Launching: render=${render}`);

  const browser = await chromium.launch({
    headless: false,  // --headless=new passed via args (Playwright's built-in headless uses old mode)
    channel: LAUNCH_CHANNEL,
    args,
  });

  const context = await browser.newContext({ viewport: getViewport(render), deviceScaleFactor: 1 });
  const page = await context.newPage();
  hookPageConsole(page, "[browser]");

  // Set default timeout so waitForFunction picks it up (its third arg is
  // options, not the second — passing {timeout} as second silently uses
  // Playwright's 30s default).
  context.setDefaultTimeout(timeout);
  page.setDefaultTimeout(timeout);

  await page.goto(url, { waitUntil: "load", timeout });
  await page.waitForFunction(
    () => typeof (window as unknown as Record<string, unknown>).__dimosBridge !== "undefined",
    undefined,
    { timeout },
  );

  console.log("[headless] Engine ready.");

  return {
    browser,
    page,
    close: async () => {
      await browser.close();
      console.log("[headless] Browser closed.");
    },
  };
}

// ── Multi-page launcher (single browser, N tabs) ────────────────────────

export async function launchMultiPage(options: MultiPageOptions): Promise<MultiPageInstance> {
  const { url, numPages, timeout = 120_000, render = "cpu" } = options;
  const args = render === "gpu" ? GPU_ARGS : CPU_ARGS;
  const viewport = getViewport(render);

  console.log(`[headless] Multi-page: ${numPages} pages, render=${render}, timeout=${timeout}ms`);

  const browser = await chromium.launch({ headless: false, channel: LAUNCH_CHANNEL, args });

  const pages: Page[] = [];
  const channels: string[] = [];

  for (let i = 0; i < numPages; i++) {
    const channel = `page-${i}`;
    channels.push(channel);

    const context = await browser.newContext({ viewport, deviceScaleFactor: 1 });
    context.setDefaultTimeout(timeout);
    const page = await context.newPage();
    page.setDefaultTimeout(timeout);
    hookPageConsole(page, `[page-${i}]`);

    const pageUrl = `${url}?channel=${channel}`;
    console.log(`[headless] Page ${i}: loading...`);
    await page.goto(pageUrl, { waitUntil: "load", timeout });
    await page.waitForFunction(
      () => typeof (window as unknown as Record<string, unknown>).__dimosBridge !== "undefined",
      undefined,
      { timeout },
    );
    console.log(`[headless] Page ${i}: ready`);

    pages.push(page);

    // Stagger launches to avoid GPU/CPU contention during scene load
    if (i < numPages - 1) await new Promise((r) => setTimeout(r, 5000));
  }

  console.log(`[headless] All ${numPages} pages ready.`);

  return {
    browser,
    pages,
    channels,
    close: async () => {
      await browser.close();
      console.log("[headless] Browser closed.");
    },
  };
}
