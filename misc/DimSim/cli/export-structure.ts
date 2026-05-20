/**
 * One-shot conversion: bakes the apartment's static-structure primitives
 * (data/structure.js) into scenes/apartment/structure.glb via the in-browser
 * GLTFExporter. Mirrors cli/export-apartment.ts but boots a temporary
 * data-driven index.js (so the primitives[] array is populated in the engine),
 * runs window.dimsim.exportStructure(), then restores the GLB-driven index.js.
 *
 * Run:
 *   ~/.deno/bin/deno run --allow-all --unstable-net misc/DimSim/cli/export-structure.ts
 *
 * Requires system Chrome on macOS (bundled Chromium WebGL is broken):
 *   DIMSIM_CHROME_CHANNEL=chrome (set automatically below)
 */
import { launchHeadless } from "./headless/launcher.ts";

const PORT = 8099;
const URL_BASE = `http://localhost:${PORT}`;

const APARTMENT_INDEX = new URL("../scenes/apartment/index.js", import.meta.url).pathname;
const APARTMENT_INDEX_BACKUP = "/tmp/apartment.index.js.glb-driven";
const STRUCTURE_GLB = new URL("../scenes/apartment/structure.glb", import.meta.url).pathname;

// Minimal data-driven build() that populates engine.primitives[] from the
// existing data/structure.js. This is only used transiently during export —
// we restore the real (GLB-driven) index.js at the end.
const DATA_DRIVEN_INDEX = `// TEMP — used by cli/export-structure.ts to bake structure.glb.
// Restored to GLB-driven version automatically after export.
import { SKY }        from './data/sky.js';
import { TAGS }       from './data/tags.js';
import { GROUPS }     from './data/groups.js';
import { LIGHTS }     from './data/lights.js';
import { PRIMITIVES } from './data/structure.js';
import { ASSETS }     from './data/objects.js';

export default async function build({ loadLevel }) {
  await loadLevel({
    version: '2.0',
    worldKey: 'default',
    tags: TAGS,
    primitives: PRIMITIVES,
    assets: ASSETS,
    lights: LIGHTS,
    groups: GROUPS,
    sceneSettings: { sky: SKY },
  });

  return {
    embodiment: null,
    spawnPoint: { x: 2, y: 0.5, z: 3 },
  };
}
`;

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

async function fileExists(path: string): Promise<boolean> {
  try {
    await Deno.stat(path);
    return true;
  } catch {
    return false;
  }
}

async function main() {
  // 1. Snapshot the current GLB-driven index.js and swap in a data-driven one.
  const originalIndex = await Deno.readTextFile(APARTMENT_INDEX);
  await Deno.writeTextFile(APARTMENT_INDEX_BACKUP, originalIndex);
  await Deno.writeTextFile(APARTMENT_INDEX, DATA_DRIVEN_INDEX);
  console.log(`[driver] swapped apartment/index.js to data-driven mode (backup at ${APARTMENT_INDEX_BACKUP})`);

  let bridge: Deno.ChildProcess | null = null;
  try {
    // 2. Start bridge as subprocess
    const bridgeCmd = new Deno.Command("deno", {
      args: [
        "run", "--allow-all", "--unstable-net",
        new URL("./bridge/server.ts", import.meta.url).pathname,
        "--scene", "apartment",
        "--port", String(PORT),
      ],
      stdout: "piped", stderr: "piped",
    });
    bridge = bridgeCmd.spawn();
    (async () => {
      const dec = new TextDecoder();
      for await (const c of bridge!.stdout.values()) Deno.stdout.write(new TextEncoder().encode(`[bridge] ${dec.decode(c)}`));
    })();
    (async () => {
      const dec = new TextDecoder();
      for await (const c of bridge!.stderr.values()) Deno.stdout.write(new TextEncoder().encode(`[bridge!] ${dec.decode(c)}`));
    })();

    console.log(`[driver] waiting for bridge on :${PORT}`);
    await waitForUrl(`${URL_BASE}/`);
    console.log(`[driver] bridge up`);

    // 3. Launch headless chrome + go to apartment
    const inst = await launchHeadless({ url: `${URL_BASE}/`, render: "gpu", timeout: 60_000 });

    try {
      // 4. Wait until the apartment finishes instantiating all assets — same
      //    gate as export-apartment.ts. Once asset groups are in the scene,
      //    the engine's primitives[] is also populated.
      console.log(`[driver] waiting for apartment to finish loading`);
      await inst.page.waitForFunction(
        () => {
          const w = window as unknown as { dimsim?: { exportStructure?: unknown } };
          if (!w.dimsim || typeof w.dimsim.exportStructure !== "function") return false;
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

      console.log(`[driver] running exportStructure()`);
      const result = await inst.page.evaluate(async () => {
        const w = window as unknown as { dimsim: { exportStructure: () => Promise<{ bytes: number; count: number }> } };
        return await w.dimsim.exportStructure();
      });
      console.log(`[driver] export returned: ${result.count} primitives, ${(result.bytes/1024).toFixed(1)} KB`);
    } finally {
      await inst.close();
    }
  } finally {
    // 5. Always restore the GLB-driven index.js, even on error.
    try {
      const backup = await Deno.readTextFile(APARTMENT_INDEX_BACKUP);
      await Deno.writeTextFile(APARTMENT_INDEX, backup);
      console.log(`[driver] restored apartment/index.js from backup`);
    } catch (e) {
      console.error(`[driver] FAILED to restore index.js (backup still at ${APARTMENT_INDEX_BACKUP}):`, e);
    }
    if (bridge) {
      try { bridge.kill("SIGTERM"); } catch { /* ignore */ }
      await bridge.status;
    }
  }

  // 6. Verify the GLB landed on disk.
  if (!(await fileExists(STRUCTURE_GLB))) {
    throw new Error(`structure.glb was not written to ${STRUCTURE_GLB}`);
  }
  const stat = await Deno.stat(STRUCTURE_GLB);
  console.log(`[driver] OK: ${STRUCTURE_GLB} (${(stat.size/1024).toFixed(1)} KB)`);
}

if (import.meta.main) {
  await main();
  Deno.exit(0);
}
