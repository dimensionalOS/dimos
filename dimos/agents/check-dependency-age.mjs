import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";

// npm ci trusts locked versions, so check their ages before running install scripts.
const npmrc = await readFile(new URL(".npmrc", import.meta.url), "utf8");
const days = Number(npmrc.match(/^min-release-age\s*=\s*(\d+)\s*$/m)?.[1]);
assert(Number.isSafeInteger(days) && days > 0, "Set min-release-age to a positive number of days in .npmrc");
const cutoff = Date.now() - days * 24 * 60 * 60 * 1000;
const lock = JSON.parse(await readFile(new URL("package-lock.json", import.meta.url), "utf8"));
assert.equal(lock.lockfileVersion, 3, "Expected package-lock.json version 3");

const packages = new Map();
for (const [path, entry] of Object.entries(lock.packages)) {
  if (path === "") continue;
  assert(
    typeof entry.version === "string" && entry.resolved?.startsWith("https://registry.npmjs.org/"),
    `${path}: cannot verify a publication date for a non-registry dependency`,
  );
  const name = entry.name ?? path.split("node_modules/").at(-1);
  if (!packages.has(name)) packages.set(name, new Set());
  packages.get(name).add(entry.version);
}

const entries = [...packages];
for (let offset = 0; offset < entries.length; offset += 16) {
  await Promise.all(
    entries.slice(offset, offset + 16).map(async ([name, versions]) => {
      const response = await fetch(`https://registry.npmjs.org/${encodeURIComponent(name)}`, {
        signal: AbortSignal.timeout(30_000),
      });
      assert(response.ok, `${name}: registry returned HTTP ${response.status}`);
      const metadata = await response.json();
      for (const version of versions) {
        const published = Date.parse(metadata.time?.[version]);
        assert(Number.isFinite(published), `${name}@${version}: registry has no valid publication date`);
        assert(
          published <= cutoff,
          `${name}@${version}: published ${metadata.time[version]}, less than ${days} days ago`,
        );
      }
    }),
  );
}
console.log(
  `All ${entries.reduce((count, [, versions]) => count + versions.size, 0)} locked versions are at least ${days} days old.`,
);
