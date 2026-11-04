import assert from "node:assert/strict";
import { spawnSync } from "node:child_process";
import { copyFile, mkdtemp, rm, writeFile } from "node:fs/promises";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { test } from "node:test";

async function check(
  t,
  { published, status = 200, resolved = "https://registry.npmjs.org/example/-/example-1.0.0.tgz" },
) {
  const directory = await mkdtemp(join(tmpdir(), "dependency-age-"));
  t.after(() => rm(directory, { recursive: true, force: true }));
  await copyFile(new URL("check-dependency-age.mjs", import.meta.url), join(directory, "check.mjs"));
  await writeFile(join(directory, ".npmrc"), "min-release-age=7\n");
  await writeFile(
    join(directory, "package-lock.json"),
    JSON.stringify({
      lockfileVersion: 3,
      packages: {
        "": { name: "project" },
        "node_modules/parent/node_modules/@scope/example": {
          version: "1.0.0",
          resolved,
          dev: true,
          optional: true,
        },
      },
    }),
  );
  const mock = `
    Date.now = () => Date.parse("2026-10-28T12:00:00Z");
    globalThis.fetch = async (url) => {
      if (url !== "https://registry.npmjs.org/%40scope%2Fexample") throw new Error("Unexpected registry URL: " + url);
      return new Response(${JSON.stringify(JSON.stringify({ time: { "1.0.0": published } }))}, { status: ${status} });
    };
  `;
  return spawnSync(
    process.execPath,
    ["--import", `data:text/javascript,${encodeURIComponent(mock)}`, join(directory, "check.mjs")],
    {
      encoding: "utf8",
      timeout: 10_000,
    },
  );
}

test("accepts a scoped, nested locked version exactly seven days old", async (t) => {
  const result = await check(t, { published: "2026-10-21T12:00:00Z" });
  assert.equal(result.status, 0, result.stderr);
  assert.match(result.stdout, /All 1 locked versions are at least 7 days old/);
});

test("rejects a recent transitive version even when it is optional and development-only", async (t) => {
  const result = await check(t, { published: "2026-10-21T12:00:01Z" });
  assert.equal(result.status, 1);
  assert.match(result.stderr, /@scope\/example@1\.0\.0: published .* less than 7 days ago/);
});

for (const published of [undefined, "invalid date"]) {
  test(`fails closed on a ${published === undefined ? "missing" : "malformed"} publication date`, async (t) => {
    const result = await check(t, { published });
    assert.equal(result.status, 1);
    assert.match(result.stderr, /registry has no valid publication date/);
  });
}

test("fails closed when the registry is unavailable", async (t) => {
  const result = await check(t, { status: 503 });
  assert.equal(result.status, 1);
  assert.match(result.stderr, /registry returned HTTP 503/);
});

test("rejects dependencies without a verifiable registry publication date", async (t) => {
  const result = await check(t, { resolved: "git+https://github.com/example/package.git#abc123" });
  assert.equal(result.status, 1);
  assert.match(result.stderr, /cannot verify a publication date for a non-registry dependency/);
});
