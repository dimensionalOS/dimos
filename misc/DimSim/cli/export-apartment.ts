/**
 * Drives `window.dimsim.exportApartmentAssets()` against a freshly-booted
 * apartment scene in headless Chromium. Writes GLBs + manifest.json into
 * scenes/apartment/objects/ via the bridge's /export-asset endpoint.
 *
 * Run:
 *   # Full decompose: data-driven apartment → GLBs + manifest
 *   ~/.deno/bin/deno run --allow-all --unstable-net misc/DimSim/cli/export-apartment.ts
 *
 *   # Verify-only: just load the apartment and confirm assets instantiate
 *   ~/.deno/bin/deno run --allow-all --unstable-net misc/DimSim/cli/export-apartment.ts --verify
 *
 * Requires system Chrome on macOS (bundled Chromium WebGL is broken):
 *   DIMSIM_CHROME_CHANNEL=chrome (set automatically below)
 */
import { launchHeadless } from "./headless/launcher.ts";

const PORT = 8099;
const DIST_DIR = new URL("../dist", import.meta.url).pathname;
const URL_BASE = `http://localhost:${PORT}`;
const VERIFY_ONLY = Deno.args.includes("--verify");

// Force system Chrome for WebGL on macOS — bundled Chromium can't render.
if (!Deno.env.get("DIMSIM_CHROME_CHANNEL")) {
  Deno.env.set("DIMSIM_CHROME_CHANNEL", "chrome");
}

async function waitForUrl(url: string, timeoutMs = 15_000): Promise<void> {
  const start = Date.now();
  while (Date.now() - start < timeoutMs) {
    try {
      const res = await fetch(url);
      if (res.ok) return;
    } catch { /* not up yet */ }
    await new Promise((r) => setTimeout(r, 200));
  }
  throw new Error(`bridge not ready within ${timeoutMs}ms`);
}

async function main() {
  // 1. Start bridge as subprocess
  const bridgeCmd = new Deno.Command("deno", {
    args: [
      "run", "--allow-all", "--unstable-net",
      new URL("./bridge/server.ts", import.meta.url).pathname,
      "--scene", "apartment",
      "--port", String(PORT),
    ],
    stdout: "piped", stderr: "piped",
  });
  const bridge = bridgeCmd.spawn();
  // Forward bridge logs so we can see /export-asset writes happen
  (async () => {
    const dec = new TextDecoder();
    for await (const c of bridge.stdout.values()) Deno.stdout.write(new TextEncoder().encode(`[bridge] ${dec.decode(c)}`));
  })();
  (async () => {
    const dec = new TextDecoder();
    for await (const c of bridge.stderr.values()) Deno.stdout.write(new TextEncoder().encode(`[bridge!] ${dec.decode(c)}`));
  })();

  console.log(`[driver] waiting for bridge on :${PORT}`);
  await waitForUrl(`${URL_BASE}/`);
  console.log(`[driver] bridge up`);

  // 2. Launch headless chrome + go to apartment
  const inst = await launchHeadless({ url: `${URL_BASE}/`, render: "gpu", timeout: 60_000 });

  try {
    // 3. Wait until the apartment finishes instantiating all assets.
    //    The engine's dimos boot flips a "scene loaded" log after `loadLevel`
    //    completes; we detect that via the assets[] count growing past 80.
    console.log(`[driver] waiting for apartment assets to instantiate`);
    await inst.page.waitForFunction(
      () => {
        const w = window as unknown as { dimsim?: { exportApartmentAssets?: unknown } };
        // assets[] is closed over inside engine.js — exposed indirectly via the
        // export helper, which only resolves once `assets` is populated.
        if (!w.dimsim || typeof w.dimsim.exportApartmentAssets !== "function") return false;
        // Probe: peek at how many asset groups exist in the scene
        const scene = (window as unknown as { __dimsim?: { scene?: { traverse: (cb: (o: { name?: string }) => void) => void } } }).__dimsim?.scene;
        if (!scene) return false;
        let n = 0;
        scene.traverse((o) => { if (o.name && o.name.startsWith("asset:")) n += 1; });
        return n >= 80;
      },
      undefined,
      { timeout: 60_000, polling: 500 },
    );

    // Give the engine a bit more time to finish material+texture loads.
    await new Promise((r) => setTimeout(r, 3000));

    if (VERIFY_ONLY) {
      const stats = await inst.page.evaluate(() => {
        const scene = (window as unknown as { __dimsim?: { scene?: { traverse: (cb: (o: { name?: string }) => void) => void } } }).__dimsim?.scene;
        let n = 0;
        scene?.traverse((o) => { if (o.name && o.name.startsWith("asset:")) n += 1; });
        return { assetGroups: n };
      });
      console.log(`[driver] verify: ${stats.assetGroups} asset groups instantiated`);
    } else {
      console.log(`[driver] running exportApartmentAssets()`);
      const result = await inst.page.evaluate(async () => {
        const w = window as unknown as { dimsim: { exportApartmentAssets: () => Promise<unknown[]> } };
        const m = await w.dimsim.exportApartmentAssets();
        return { count: m.length };
      });
      console.log(`[driver] export returned: ${result.count} asset entries`);
      const manifestPath = new URL("../scenes/apartment/objects/manifest.json", import.meta.url).pathname;
      const txt = await Deno.readTextFile(manifestPath);
      const parsed = JSON.parse(txt);
      console.log(`[driver] manifest.json: ${parsed.length} entries`);
      const totalStates = parsed.reduce((acc: number, e: { states: unknown[] }) => acc + e.states.length, 0);
      console.log(`[driver] total state-glbs: ${totalStates}`);
    }
  } finally {
    await inst.close();
    try { bridge.kill("SIGTERM"); } catch { /* ignore */ }
    await bridge.status;
  }
}

if (import.meta.main) {
  await main();
  Deno.exit(0);
}
