import assert from "node:assert/strict";
import { agentPaths } from "#dimos/agents/gateway/config/paths.ts";
import { join } from "node:path";
import { migrateDimcode } from "#dimos/agents/gateway/config/migrate.ts";
import { mkdir, mkdtemp, readFile, rm, stat, writeFile } from "node:fs/promises";
import { test } from "node:test";
import { tmpdir } from "node:os";

async function home(t: { after(fn: () => Promise<void>): void }) {
  const root = await mkdtemp(join(tmpdir(), "dimos-agent-migrate-"));
  t.after(() => rm(root, { recursive: true, force: true }));
  const paths = agentPaths({ HOME: root, XDG_CONFIG_HOME: join(root, "config"), XDG_STATE_HOME: join(root, "state") });
  await mkdir(paths.legacyDir, { recursive: true });
  return paths;
}

const AUTH =
  '{"openai-codex":{"type":"oauth","access":"fixture-access","refresh":"fixture-refresh","expires":1,"accountId":"a"}}';

test("dimcode's credentials and settings are copied once and the originals are only read", async (t) => {
  const paths = await home(t);
  const settings =
    '{"theme":"light","defaultProvider":"openai-codex","defaultModel":"gpt-5.6-luna","defaultThinkingLevel":"high"}';
  const config = '{"gatewayStart":"login","workspace":"/srv/robot","python":"/old/python","mcp":[]}';
  await writeFile(join(paths.legacyDir, "auth.json"), AUTH, { mode: 0o600 });
  await writeFile(join(paths.legacyDir, "settings.json"), settings);
  await writeFile(join(paths.legacyDir, "config.json"), config);

  assert.deepEqual(await migrateDimcode(paths), {
    kind: "copied",
    files: [
      join(paths.legacyDir, "auth.json"),
      join(paths.legacyDir, "config.json"),
      join(paths.legacyDir, "settings.json"),
    ],
  });
  assert.deepEqual(JSON.parse(await readFile(paths.configFile, "utf8")), {
    workspace: "/srv/robot",
    model: { provider: "openai-codex", modelId: "gpt-5.6-luna" },
    thinkingLevel: "high",
    theme: "light",
  });
  assert.equal(await readFile(paths.authFile, "utf8"), AUTH);
  assert.equal((await stat(paths.authFile)).mode & 0o777, 0o600);
  assert.equal(await readFile(join(paths.legacyDir, "auth.json"), "utf8"), AUTH);
  assert.equal(await readFile(join(paths.legacyDir, "settings.json"), "utf8"), settings);
  assert.equal(await readFile(join(paths.legacyDir, "config.json"), "utf8"), config);

  await writeFile(join(paths.legacyDir, "settings.json"), '{"theme":"dark"}');
  assert.deepEqual(await migrateDimcode(paths), { kind: "already" });
  assert.equal(JSON.parse(await readFile(paths.configFile, "utf8")).theme, "light", "a second run copies nothing");
});

test("migration never overwrites an existing configuration and writes nothing without dimcode files", async (t) => {
  const paths = await home(t);
  assert.deepEqual(await migrateDimcode(paths), { kind: "nothing" });
  await assert.rejects(stat(paths.configFile), { code: "ENOENT" });

  await writeFile(join(paths.legacyDir, "auth.json"), AUTH, { mode: 0o600 });
  await mkdir(paths.configDir, { recursive: true });
  await writeFile(paths.configFile, '{"theme":"mine"}');
  assert.deepEqual(await migrateDimcode(paths), { kind: "already" });
  assert.equal(await readFile(paths.configFile, "utf8"), '{"theme":"mine"}');
  await assert.rejects(stat(paths.authFile), { code: "ENOENT" });
});
