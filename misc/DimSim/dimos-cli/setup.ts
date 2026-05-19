/**
 * DimSim Setup — Downloads core assets and scenes from GitHub Releases.
 *
 * Local data stored at ~/.dimsim/ (override with DIMSIM_HOME env var).
 *
 *   ~/.dimsim/
 *   ├── dist/           (core frontend: index.html, assets/, agent-model/)
 *   │   └── sims/       (downloaded scene JSON files)
 *   │       └── apt.json
 *   └── evals/          (eval workflows)
 */

const GITHUB_REPO = "Antim-Labs/DimSim";
const RELEASES_API = `https://api.github.com/repos/${GITHUB_REPO}/releases/latest`;

export function getDimsimHome(): string {
  return (
    Deno.env.get("DIMSIM_HOME") ||
    `${Deno.env.get("HOME")}/.dimsim`
  );
}

export function getDistDir(): string {
  return `${getDimsimHome()}/dist`;
}

// ── Registry ────────────────────────────────────────────────────────────

interface SceneEntry {
  url: string;
  description: string;
  size: number;
}

interface Registry {
  version: string;
  coreUrl: string;
  evalsUrl?: string;
  scenes: Record<string, SceneEntry>;
}

async function fetchRegistry(localPath?: string): Promise<Registry> {
  if (localPath) {
    return JSON.parse(await Deno.readTextFile(localPath));
  }

  // Fetch latest release from GitHub API, find registry.json asset
  const resp = await fetch(RELEASES_API, {
    headers: { Accept: "application/vnd.github.v3+json" },
  });
  if (!resp.ok) throw new Error(`Failed to fetch latest release: ${resp.status}`);
  const release = await resp.json();

  const asset = release.assets?.find(
    (a: { name: string }) => a.name === "registry.json",
  );
  if (!asset) {
    throw new Error("registry.json not found in latest GitHub release");
  }

  const regResp = await fetch(asset.browser_download_url);
  if (!regResp.ok) throw new Error(`Failed to download registry: ${regResp.status}`);
  return regResp.json();
}

// ── Download with progress ──────────────────────────────────────────────

async function download(url: string, dest: string): Promise<void> {
  console.log(`  Downloading ${url}`);
  const resp = await fetch(url);
  if (!resp.ok) throw new Error(`Download failed: ${resp.status} ${url}`);

  const total = parseInt(resp.headers.get("content-length") || "0");
  const reader = resp.body!.getReader();
  const chunks: Uint8Array[] = [];
  let received = 0;

  while (true) {
    const { done, value } = await reader.read();
    if (done) break;
    chunks.push(value);
    received += value.length;
    if (total > 0) {
      const pct = ((received / total) * 100).toFixed(0);
      const mb = (received / 1e6).toFixed(1);
      Deno.stderr.writeSync(
        new TextEncoder().encode(`\r  ${mb} MB / ${(total / 1e6).toFixed(1)} MB (${pct}%)`)
      );
    }
  }
  Deno.stderr.writeSync(new TextEncoder().encode("\n"));

  // Concatenate chunks into a single Uint8Array
  const totalLength = chunks.reduce((sum, c) => sum + c.length, 0);
  const result = new Uint8Array(totalLength);
  let offset = 0;
  for (const chunk of chunks) {
    result.set(chunk, offset);
    offset += chunk.length;
  }
  await Deno.writeFile(dest, result);
}

// ── Extract tar.gz ──────────────────────────────────────────────────────

async function extractTarGz(archive: string, destDir: string): Promise<void> {
  await Deno.mkdir(destDir, { recursive: true });
  const proc = new Deno.Command("tar", {
    args: ["-xzf", archive, "-C", destDir],
    stdout: "inherit",
    stderr: "inherit",
  }).spawn();
  const status = await proc.status;
  if (!status.success) throw new Error(`tar extract failed (exit ${status.code})`);
}

/** Extract a single gzipped file (not a tar, just gzip). */
async function extractGz(archive: string, destFile: string): Promise<void> {
  const parentDir = destFile.substring(0, destFile.lastIndexOf("/"));
  await Deno.mkdir(parentDir, { recursive: true });
  const proc = new Deno.Command("sh", {
    args: ["-c", `gunzip -c "${archive}" > "${destFile}"`],
    stdout: "inherit",
    stderr: "inherit",
  }).spawn();
  const status = await proc.status;
  if (!status.success) throw new Error(`gunzip failed (exit ${status.code})`);
}

// ── Version tracking ────────────────────────────────────────────────────

interface VersionInfo {
  core?: string;
  scenes?: Record<string, string>;
}

function versionPath(): string {
  return `${getDimsimHome()}/version.json`;
}

async function readVersionInfo(): Promise<VersionInfo> {
  try {
    return JSON.parse(await Deno.readTextFile(versionPath()));
  } catch {
    return {};
  }
}

async function writeVersionInfo(info: VersionInfo): Promise<void> {
  await Deno.writeTextFile(versionPath(), JSON.stringify(info, null, 2));
}

// ── Public API ──────────────────────────────────────────────────────────

export async function setup(localArchive?: string): Promise<void> {
  const home = getDimsimHome();
  const distDir = getDistDir();

  await Deno.mkdir(home, { recursive: true });

  let registry: Registry | null = null;
  let didWork = false;

  if (localArchive) {
    console.log(`[dimsim] Extracting core from local archive: ${localArchive}`);
    await extractTarGz(localArchive, distDir);
    didWork = true;
  } else {
    registry = await fetchRegistry();
    const local = await readVersionInfo();

    if (local.core !== registry.version) {
      if (local.core) {
        console.log(`[dimsim] Updating core: v${local.core} → v${registry.version}`);
      } else {
        console.log(`[dimsim] Installing core v${registry.version}`);
      }
      const tmpFile = `${home}/core-download.tar.gz`;
      await download(registry.coreUrl, tmpFile);
      console.log(`[dimsim] Extracting core assets...`);
      await extractTarGz(tmpFile, distDir);
      await Deno.remove(tmpFile);
      local.core = registry.version;
      await writeVersionInfo(local);
      didWork = true;
    }
  }

  // Ensure sims directory exists
  await Deno.mkdir(`${distDir}/sims`, { recursive: true });

  // Create empty manifest if none exists
  const manifestPath = `${distDir}/sims/manifest.json`;
  try {
    await Deno.stat(manifestPath);
  } catch {
    await Deno.writeTextFile(manifestPath, JSON.stringify([], null, 2));
  }

  // Install evals to ~/.dimsim/evals/
  if (registry?.evalsUrl) {
    const evalsDir = `${home}/evals`;
    const evalsVerFile = `${home}/evals-version`;
    let installedEvalsVer: string | null = null;
    try {
      installedEvalsVer = (await Deno.readTextFile(evalsVerFile)).trim();
    } catch { /* not installed */ }

    if (installedEvalsVer !== registry.version) {
      const tmpFile = `${home}/evals-download.tar.gz`;
      console.log(`[dimsim] Updating evals → v${registry.version}`);
      await download(registry.evalsUrl, tmpFile);
      await extractTarGz(tmpFile, evalsDir);
      await Deno.remove(tmpFile);
      await Deno.writeTextFile(evalsVerFile, registry.version);
      didWork = true;
    }
  }

  const verLabel = registry?.version ? `v${registry.version}` : "local";
  if (didWork) {
    console.log(`[dimsim] core + evals ready (${verLabel})`);
  } else {
    console.log(`[dimsim] core + evals up-to-date (${verLabel})`);
  }
}

export async function sceneInstall(
  name: string,
  localArchive?: string,
): Promise<void> {
  const home = getDimsimHome();
  const distDir = getDistDir();
  const simsDir = `${distDir}/sims`;
  const destFile = `${simsDir}/${name}.json`;

  // Check core is set up
  try {
    await Deno.stat(distDir);
  } catch {
    console.error(`[dimsim] Core not installed. Run 'dimsim setup' first.`);
    Deno.exit(1);
  }

  await Deno.mkdir(simsDir, { recursive: true });

  if (localArchive) {
    console.log(`[dimsim] Installing scene '${name}' from local: ${localArchive}`);
    if (localArchive.endsWith(".json")) {
      await Deno.copyFile(localArchive, destFile);
    } else {
      await extractGz(localArchive, destFile);
    }
  } else {
    const registry = await fetchRegistry();
    const entry = registry.scenes[name];
    if (!entry) {
      console.error(`[dimsim] Scene '${name}' not found. Available:`);
      for (const [k, v] of Object.entries(registry.scenes)) {
        console.error(`  ${k} — ${v.description}`);
      }
      Deno.exit(1);
    }

    const local = await readVersionInfo();
    const localSceneVer = local.scenes?.[name];

    if (localSceneVer === registry.version) {
      console.log(`[dimsim] Scene '${name}' already up-to-date (v${registry.version})`);
      return;
    }

    if (localSceneVer) {
      console.log(`[dimsim] Updating scene '${name}': v${localSceneVer} → v${registry.version}`);
    }
    const tmpFile = `${home}/${name}-download.gz`;
    console.log(`[dimsim] Downloading scene '${name}' (${(entry.size / 1e6).toFixed(1)} MB)...`);
    await download(entry.url, tmpFile);
    console.log(`[dimsim] Extracting...`);
    await extractGz(tmpFile, destFile);
    await Deno.remove(tmpFile);

    // Write updated version
    if (!local.scenes) local.scenes = {};
    local.scenes[name] = registry.version;
    await writeVersionInfo(local);
  }

  // Update local manifest
  const manifestPath = `${simsDir}/manifest.json`;
  let manifest: string[] = [];
  try {
    manifest = JSON.parse(await Deno.readTextFile(manifestPath));
  } catch { /* empty */ }
  if (!manifest.includes(name)) {
    manifest.push(name);
    await Deno.writeTextFile(manifestPath, JSON.stringify(manifest, null, 2));
  }

  console.log(`[dimsim] Scene '${name}' installed.`);
}

export async function sceneList(): Promise<void> {
  const simsDir = `${getDistDir()}/sims`;

  // Local scenes
  const installed: string[] = [];
  try {
    for await (const entry of Deno.readDir(simsDir)) {
      if (entry.name.endsWith(".json") && entry.name !== "manifest.json") {
        installed.push(entry.name.replace(".json", ""));
      }
    }
  } catch { /* no sims dir */ }

  // Remote scenes
  let registry: Registry | null = null;
  try {
    registry = await fetchRegistry();
  } catch {
    console.log("[dimsim] Could not fetch remote registry.");
  }

  console.log("\nInstalled scenes:");
  if (installed.length === 0) {
    console.log("  (none)");
  } else {
    for (const s of installed) console.log(`  * ${s}`);
  }

  if (registry) {
    console.log("\nAvailable scenes:");
    for (const [name, entry] of Object.entries(registry.scenes)) {
      const status = installed.includes(name) ? " (installed)" : "";
      console.log(`  ${name} — ${entry.description} (${(entry.size / 1e6).toFixed(1)} MB)${status}`);
    }
  }
  console.log();
}

export async function sceneRemove(name: string): Promise<void> {
  const simsDir = `${getDistDir()}/sims`;
  const scenePath = `${simsDir}/${name}.json`;
  try {
    await Deno.remove(scenePath);
    console.log(`[dimsim] Scene '${name}' removed.`);
  } catch {
    console.error(`[dimsim] Scene '${name}' not found locally.`);
    Deno.exit(1);
  }

  // Update manifest
  const manifestPath = `${simsDir}/manifest.json`;
  try {
    const manifest: string[] = JSON.parse(await Deno.readTextFile(manifestPath));
    const filtered = manifest.filter((s) => s !== name);
    await Deno.writeTextFile(manifestPath, JSON.stringify(filtered, null, 2));
  } catch { /* ok */ }
}
