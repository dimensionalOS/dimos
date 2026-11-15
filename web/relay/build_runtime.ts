// Export the relay and SDK sources that Desktop's offline Nix build compiles
// into its managed runtime: the pinned SDK bundle, the relay and shared
// TypeScript, a production-only lockfile and the vendored npm graph. Build-time
// only: the installed runtime neither finds a checkout nor downloads JS
// dependencies.
import { parseArgs } from "@std/cli";

const web = new URL("../", import.meta.url);

interface LockedPackage {
  dependencies?: string[];
  optionalDependencies?: string[];
}

/** The workspace lock lists every development package; vendor only the relay's graph. */
function productionNpm(
  packages: Record<string, LockedPackage>,
  roots: string[],
): Record<string, LockedPackage> {
  const selected: Record<string, LockedPackage> = {};
  const visit = (dependency: string) => {
    const matches = dependency in packages
      ? [dependency]
      : Object.keys(packages).filter((key) => key.startsWith(`${dependency}@`));
    if (matches.length !== 1) throw new Error(`ambiguous locked dependency: ${dependency}`);
    const key = matches[0];
    if (key in selected) return;
    const entry = packages[key];
    selected[key] = entry;
    for (const child of [...entry.dependencies ?? [], ...entry.optionalDependencies ?? []]) {
      visit(child);
    }
  };
  roots.forEach(visit);
  return Object.fromEntries(Object.entries(selected).sort(([a], [b]) => a.localeCompare(b)));
}

async function copySources(from: URL, to: string): Promise<void> {
  await Deno.mkdir(to, { recursive: true });
  for await (const entry of Deno.readDir(from)) {
    if (
      entry.isFile && entry.name.endsWith(".ts") && !entry.name.endsWith("_test.ts") &&
      entry.name !== "build_runtime.ts"
    ) {
      await Deno.copyFile(new URL(entry.name, from), `${to}/${entry.name}`);
    }
  }
}

async function copyTree(from: string, to: string): Promise<void> {
  await Deno.mkdir(to, { recursive: true });
  for await (const entry of Deno.readDir(from)) {
    const source = `${from}/${entry.name}`, target = `${to}/${entry.name}`;
    if (entry.isDirectory) await copyTree(source, target);
    else if (entry.isSymlink) await Deno.symlink(await Deno.readLink(source), target);
    else await Deno.copyFile(source, target);
  }
}

async function main(): Promise<void> {
  const args = parseArgs(Deno.args, { string: ["output"] });
  if (!args.output) throw new Error("--output must name the source artifact directory");
  const versionSource = await Deno.readTextFile(new URL("../dimos/utils/deno.py", web));
  const pinned = versionSource.match(/^DENO_VERSION = "v([^"]+)"$/m)?.[1];
  if (!pinned || Deno.version.deno !== pinned) {
    throw new Error(`runtime builds require Deno ${pinned}, got ${Deno.version.deno}`);
  }
  await Deno.mkdir(args.output, { recursive: true });
  const output = await Deno.realPath(args.output);
  const scratch = await Deno.makeTempDir({ prefix: "dimos-relay-build-" });
  try {
    // Build from the selected source with its frozen lock. Never copy the
    // ignored sdk/dist output left by a previous development build.
    const sdk = await new Deno.Command(Deno.execPath(), {
      args: [
        "run",
        "--frozen",
        "-A",
        "npm:vite",
        "build",
        "--outDir",
        `${scratch}/sdk`,
        "--emptyOutDir",
      ],
      cwd: new URL("sdk/", web),
      stdout: "inherit",
      stderr: "inherit",
    }).output();
    if (!sdk.success) throw new Error("cannot build the pinned SDK source");
    await copySources(new URL("relay/", web), `${scratch}/relay`);
    await copySources(new URL("shared/", web), `${scratch}/shared`);
    const workspace = JSON.parse(await Deno.readTextFile(new URL("deno.json", web)));
    const relay = JSON.parse(await Deno.readTextFile(new URL("relay/deno.json", web)));
    // Vendoring inside the development workspace would pull in ALL its npm
    // packages (including Vite, React, Three.js and Vitest). Isolate the
    // production graph while retaining the very same dependency pins and
    // integrity hashes.
    await Deno.writeTextFile(
      `${scratch}/deno.json`,
      JSON.stringify({
        nodeModulesDir: "auto",
        vendor: true,
        unstable: ["net"],
        imports: {
          ...workspace.imports,
          ...relay.imports,
          "@dimos/shared": "./shared/protocol.ts",
          "@dimos/shared/manifest": "./shared/manifest.ts",
        },
      }),
    );
    const lock = JSON.parse(await Deno.readTextFile(new URL("deno.lock", web)));
    const npmRoots: string[] = lock.workspace.members.relay.dependencies;
    lock.npm = productionNpm(
      lock.npm,
      npmRoots.map((specifier) => {
        const name = specifier.slice(4, specifier.lastIndexOf("@"));
        return `${name}@${lock.specifiers[specifier]}`;
      }),
    );
    lock.specifiers = Object.fromEntries(
      Object.entries(lock.specifiers).filter(([key]) =>
        !key.startsWith("npm:") || npmRoots.includes(key)
      ),
    );
    lock.workspace = {
      dependencies: [
        ...lock.workspace.dependencies,
        ...npmRoots,
      ].sort(),
    };
    await Deno.writeTextFile(`${scratch}/deno.lock`, JSON.stringify(lock, null, 2) + "\n");
    const cached = await new Deno.Command(Deno.execPath(), {
      args: ["cache", "--frozen", "--vendor", "--node-modules-dir=auto", "relay/main.ts"],
      cwd: scratch,
      stdout: "inherit",
      stderr: "inherit",
    }).output();
    if (!cached.success) throw new Error("cannot vendor the pinned relay graph");
    await copyTree(scratch, output);
    // Deno's setup cache includes the temporary source path. It is rebuilt
    // from the vendored packages and lockfile, and is not a source input.
    await Deno.remove(`${output}/node_modules/.deno/.setup-cache.bin`);
  } finally {
    await Deno.remove(scratch, { recursive: true });
  }
}

if (import.meta.main) await main();
