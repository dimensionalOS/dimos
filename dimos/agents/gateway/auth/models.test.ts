import assert from "node:assert/strict";
import { LoginFlows } from "#dimos/agents/gateway/auth/dialog.ts";
import { ModelCatalog, openModels } from "#dimos/agents/gateway/auth/models.ts";
import { getBuiltinModels } from "@earendil-works/pi-ai/providers/all";
import { join } from "node:path";
import { mkdtemp, rm, writeFile } from "node:fs/promises";
import { test } from "node:test";
import { tmpdir } from "node:os";
import { until } from "#dimos/agents/sdk/until.ts";

test("startup and login discover newly published models", async (t) => {
  const dir = await mkdtemp(join(tmpdir(), "dimos-model-catalog-"));
  const env = process.env;
  process.env = { HOME: dir, PATH: env.PATH };
  t.after(async () => {
    process.env = env;
    await rm(dir, { recursive: true, force: true });
  });
  const provider = "openai";
  const model = { ...getBuiltinModels(provider)[0], id: "first-catalog-model" };
  t.mock.method(globalThis, "fetch", async () =>
    Response.json([model], { headers: { "last-modified": new Date().toUTCString() } }),
  );
  const config = join(dir, "models.json");
  await writeFile(config, JSON.stringify({ providers: { openai: { apiKey: "!exit 1" } } }));
  const report = (error: unknown) => assert.fail(String(error));
  const models = await openModels(join(dir, "auth.json"), config, [], report);
  assert.ok(models.getModel(provider, model.id));
  model.id = "newer-catalog-model";
  const principal = { kind: "gateway" } as const;
  const logins = new LoginFlows(models, new ModelCatalog(models), report, 5_000);
  const state = logins.of(principal);
  await logins.start(principal, provider, "api_key");
  const prompt = await until(state, (s) => (s.phase === "running" ? (s.prompt ?? undefined) : undefined));
  await logins.answer(principal, prompt.dialog, "fixture-key");
  await until(state, (s) => (s.phase === "done" ? true : undefined));
  assert.ok(models.getModel(provider, model.id));
});
