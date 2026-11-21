import assert from "node:assert/strict";
import type { ConversationId } from "@earendil-works/pi-durable";
import type { ProfileDraft } from "#dimos/agents/shared/protocol/profiles.ts";
import { BACKGROUND_CONTEXT } from "@earendil-works/chord/context";
import { Conversations } from "#dimos/agents/harness/conversations.ts";
import { ContextBudget } from "#dimos/agents/harness/limit/budget.ts";
import { ConversationHistory } from "#dimos/agents/harness/history/forks.ts";
import { DimosConnection } from "#dimos/agents/harness/dimos/connection.ts";
import { InputGate } from "#dimos/agents/harness/inputs/gate.ts";
import { OutputLimiter } from "#dimos/agents/harness/limit/limit.ts";
import { RuntimeObserver } from "#dimos/agents/harness/context/observer.ts";
import { GENERAL_PROFILE, ProfileSchema } from "#dimos/agents/shared/protocol/profiles.ts";
import { Value } from "typebox/value";
import { Harness, createRegistry, defineExtension } from "@earendil-works/pi-durable";
import { Profiles, boundProfile, profileCatalog, profileExtension } from "#dimos/agents/harness/profiles.ts";
import { RuntimeClient } from "#dimos/agents/harness/dimos/client.ts";
import { SqliteStorage } from "@earendil-works/pi-durable/storage/sqlite";
import { fauxAssistantMessage, fauxToolCall } from "@earendil-works/pi-ai";
import { dirname, join } from "node:path";
import { mkdir, mkdtemp, rm, writeFile } from "node:fs/promises";
import { openNodeSqliteDatabase } from "@earendil-works/pi-durable/storage/sqlite/node";
import { profileTool } from "#dimos/agents/harness/tools/profile.ts";
import { scriptedModel } from "#dimos/agents/test/scripted-model.ts";
import { test } from "node:test";
import { tmpdir } from "node:os";

const context = BACKGROUND_CONTEXT;
const SNACKS: ProfileDraft = {
  alwaysLoadedSkills: [],
  availableSkills: [],
  name: "Piper snack sorter",
  description: "Sort snacks at the bench.",
  role: "Group snacks by type and arrange them neatly.",
  blueprints: ["piper-teleop"],
};

async function open(path: string, skillRoots: string[]) {
  const model = scriptedModel();
  const registry = createRegistry();
  registry.install(await profileExtension(skillRoots, () => {}));
  const harness = await Harness.open(
    await SqliteStorage.open(await openNodeSqliteDatabase(path)),
    { models: model.models, registry },
    context,
  );
  const profiles = new Profiles({
    harness,
    skillRoots,
    report: (error) => {
      throw error;
    },
    install: { python: "/bin/false", root: dirname(path) },
    env: {},
  });
  registry.install(defineExtension({ name: "profile-tool", tools: [profileTool(profiles).registration] }));
  const connection = new DimosConnection({
    url: new URL("http://127.0.0.1:9/mcp"),
    connectTimeoutMs: 100,
    maxBackoffMs: 100,
    report: () => {},
  });
  await connection.close();
  const conversations = new Conversations({
    harness,
    profiles,
    runtime: new RuntimeClient(connection),
    gatewayFacts: async () => ({}),
    defaults: () => ({ model: model.model, thinkingLevel: null, workspace: "/tmp" }),
    snapshotTimeoutMs: 100,
  });
  const observer = new RuntimeObserver({
    harness,
    runtime: new RuntimeClient(connection),
    jobFacts: async () => ({}),
    runsDir: join(dirname(path), "runs"),
    reuseForMs: 0,
    timeoutMs: 100,
    report: () => {},
  });
  const inputs = new InputGate({
    harness,
    observer,
    limiter: new OutputLimiter({
      dir: join(dirname(path), "results"),
      budget: new ContextBudget(model.models),
      defaultMaxChars: 2_000,
    }),
  });
  const history = new ConversationHistory(harness, inputs, model.models);
  harness.resume();
  const send = async (id: ConversationId, content: string) => {
    const conversation = await harness.conversation(id, context);
    assert.ok(conversation);
    const sent = await conversation.submit({ type: "input", content }, context);
    const settled = await sent.wait(context);
    assert.equal(settled.status, "done");
    await conversation.waitForIdle(context);
    return conversation;
  };
  return { model, harness, profiles, conversations, history, send };
}

test("defaults persist, explicit overrides do not change them, and old conversations retain their definition", async (t) => {
  const dir = await mkdtemp(join(tmpdir(), "dimos-profiles-"));
  t.after(() => rm(dir, { recursive: true, force: true }));
  const path = join(dir, "agent.sqlite");
  const first = await open(path, []);
  const general = await first.conversations.start(null, "general", context);
  assert.equal((await boundProfile(first.harness, general, context)).profile.id, "general");
  const snacks = await first.profiles.save(SNACKS, context);
  const created = await first.profiles.save({ ...SNACKS, name: "Another sorter" }, context);
  assert.ok(created.id);
  const named = await first.conversations.start(null, "by-name", context, "Another sorter");
  assert.equal((await boundProfile(first.harness, named, context)).profile.id, created.id);
  await first.profiles.save({ ...created, name: "Renamed sorter" }, context);
  assert.equal(
    (await profileCatalog(first.harness, context)).profiles.find((entry) => entry.name === "Renamed sorter")?.id,
    created.id,
  );
  await assert.rejects(first.profiles.save({ ...SNACKS, name: "Piper snack sorter" }, context), /already exists/);
  await assert.rejects(
    first.profiles.save({ ...SNACKS, availableSkills: ["sorting"], alwaysLoadedSkills: ["sorting"] }, context),
    /not both/,
  );
  await first.profiles.setDefault(snacks.id, context);
  const original = await first.conversations.start(null, "original", context);
  const override = await first.conversations.start(null, "override", context, "general");
  assert.equal((await boundProfile(first.harness, override, context)).profile.id, "general");
  assert.equal((await profileCatalog(first.harness, context)).defaultProfile, snacks.id);
  assert.equal(await first.conversations.start(null, "original", context, "general"), original);
  await first.profiles.save({ ...snacks, name: "Edited profile", role: "A new role." }, context);
  assert.deepEqual((await boundProfile(first.harness, original, context)).profile, snacks);
  await first.harness.close(context);
  const resumed = await open(path, []);
  t.after(() => resumed.harness.close(context));
  assert.equal((await profileCatalog(resumed.harness, context)).defaultProfile, snacks.id);
  assert.deepEqual((await boundProfile(resumed.harness, original, context)).profile, snacks);
  const next = await resumed.conversations.start(null, "next", context);
  assert.equal((await boundProfile(resumed.harness, next, context)).profile.name, "Edited profile");
  await resumed.profiles.setDefault("general", context);
  assert.deepEqual((await boundProfile(resumed.harness, original, context)).profile, snacks);
  await assert.rejects(resumed.conversations.start(null, "missing", context, "missing-profile"), /Unknown profile/);
  await assert.rejects(resumed.profiles.setDefault("missing-profile", context), /Unknown profile/);
});

test("always-loaded and available skill bodies are frozen across file edits, forks and a context summary", async (t) => {
  const dir = await mkdtemp(join(tmpdir(), "dimos-profile-skills-"));
  t.after(() => rm(dir, { recursive: true, force: true }));
  const bundled = join(dir, "bundled");
  const userRoot = join(dir, "user");
  const skill = async (name: string, text: string, root = userRoot, disabled = false) => {
    await mkdir(join(root, name), { recursive: true });
    await writeFile(
      join(root, name, "SKILL.md"),
      `---\nname: ${name}\ndescription: ${name} description\ndisable-model-invocation: ${disabled}\n---\n${text}\n`,
    );
  };
  await skill("sorting", "ALWAYS_ORIGINAL_INSTRUCTIONS");
  await skill("presentation", "AVAILABLE_ORIGINAL_INSTRUCTIONS");
  await skill("unselected", "UNSELECTED_INSTRUCTIONS");
  await skill("overridden", "Bundled instructions", bundled);
  await skill("disabled", "Bundled instructions", bundled);
  await skill("overridden", "User instructions");
  await skill("disabled", "Disabled by user", userRoot, true);
  const h = await open(join(dir, "agent.sqlite"), [bundled, userRoot]);
  t.after(() => h.harness.close(context));
  const options = await h.profiles.choices();
  assert.deepEqual(options.skills.map((skill) => skill.name).sort(), [
    "overridden",
    "presentation",
    "sorting",
    "unselected",
  ]);
  assert.ok(options.blueprintsError);
  await skill("newly-added", "New instructions.");
  assert.ok((await h.profiles.choices()).skills.some((skill) => skill.name === "newly-added"));
  const snacks = await h.profiles.save(
    { ...SNACKS, alwaysLoadedSkills: ["sorting"], availableSkills: ["presentation"] },
    context,
  );
  const id = await h.conversations.start(null, "snacks", context, snacks.id);
  await skill("sorting", "ALWAYS_CHANGED");
  await skill("presentation", "AVAILABLE_CHANGED");
  h.model.script([
    (request) => {
      const prompt = JSON.stringify(request.messages);
      assert.match(prompt, /ALWAYS_ORIGINAL_INSTRUCTIONS/);
      assert.ok(prompt.includes(`Skill directory: ${join(userRoot, "sorting")}`));
      assert.ok(prompt.includes(join(userRoot, "overridden", "SKILL.md")));
      assert.ok(!prompt.includes(join(bundled, "overridden")));
      assert.doesNotMatch(prompt, /- disabled:/);
      assert.match(prompt, /presentation description/);
      assert.doesNotMatch(prompt, /AVAILABLE_ORIGINAL_INSTRUCTIONS|ALWAYS_CHANGED|AVAILABLE_CHANGED|unselected/);
      return fauxAssistantMessage([fauxToolCall("profile", { op: "read_skill", name: "presentation" })], {
        stopReason: "toolUse",
      });
    },
    (request) => {
      const prompt = JSON.stringify(request.messages);
      assert.match(prompt, /AVAILABLE_ORIGINAL_INSTRUCTIONS/);
      assert.ok(prompt.includes(`Skill directory: ${join(userRoot, "presentation")}`));
      return fauxAssistantMessage("Ready to sort.");
    },
  ]);
  const source = await h.send(id, "Get ready.");
  const entries = await source.entries({ order: "ascending" }, 100, undefined, context);
  const at = entries.items.at(-1)?.id;
  assert.ok(at);
  const fork = await source.fork(at, { ownership: { kind: "ownerless" } }, context);
  assert.deepEqual(await boundProfile(h.harness, fork.id, context), await boundProfile(h.harness, id, context));
  const user = entries.items.find((entry) => entry.kind === "pi.user");
  assert.ok(user);
  h.model.script([fauxAssistantMessage("Conversation summarized: ready to sort.")]);
  const summary = await h.history.summarizeFrom(id, user.id, "summary-profile", context);
  assert.ok(summary.kind === "new" || summary.kind === "fork");
  assert.deepEqual(
    await boundProfile(h.harness, summary.conversation, context),
    await boundProfile(h.harness, id, context),
  );
  h.model.script([
    (request) => {
      assert.match(JSON.stringify(request.messages), /ALWAYS_ORIGINAL_INSTRUCTIONS/);
      assert.doesNotMatch(JSON.stringify(request.messages), /ALWAYS_CHANGED|AVAILABLE_CHANGED/);
      return fauxAssistantMessage("Still ready.");
    },
  ]);
  await h.send(summary.conversation, "Continue.");
  const fresh = await h.conversations.start(null, "fresh", context, snacks.id);
  assert.match((await boundProfile(h.harness, fresh, context)).skills[0].instructions, /ALWAYS_CHANGED/);
});

test("the agent saves into the live human catalog and missing skills fail before a conversation starts", async (t) => {
  const dir = await mkdtemp(join(tmpdir(), "dimos-agent-profile-tool-"));
  t.after(() => rm(dir, { recursive: true, force: true }));
  const h = await open(join(dir, "agent.sqlite"), []);
  t.after(() => h.harness.close(context));
  const id = await h.conversations.start(null, "agent", context);
  h.model.script([
    fauxAssistantMessage([fauxToolCall("profile", { op: "save", profile: SNACKS })], { stopReason: "toolUse" }),
    fauxAssistantMessage("Saved."),
  ]);
  await h.send(id, "Save this setup as a profile.");
  const snacks = (await profileCatalog(h.harness, context)).profiles.find((profile) => profile.name === SNACKS.name);
  assert.ok(snacks);
  assert.deepEqual(snacks, { ...SNACKS, id: snacks.id });
  await h.profiles.save({ ...snacks, alwaysLoadedSkills: ["missing"] }, context);
  await assert.rejects(h.conversations.start(null, "broken", context, snacks.id), /requires missing skill/);
  assert.equal(await h.conversations.created("broken", context), null);
  assert.equal((await boundProfile(h.harness, id, context)).profile.id, "general");
});

test("a profile named General cannot replace the built-in default or its exact-id lookup", async (t) => {
  const dir = await mkdtemp(join(tmpdir(), "dimos-profile-general-"));
  t.after(() => rm(dir, { recursive: true, force: true }));
  const h = await open(join(dir, "agent.sqlite"), []);
  t.after(() => h.harness.close(context));
  const custom = await h.profiles.save({ ...SNACKS, name: "General" }, context);
  assert.equal((await h.profiles.resolve(null, context)).profile.id, "general");
  assert.equal((await h.profiles.resolve("general", context)).profile.id, "general");
  assert.equal((await h.profiles.resolve("General", context)).profile.id, custom.id);
  const id = await h.conversations.start(null, "general-precedence", context);
  h.model.script([
    fauxAssistantMessage([fauxToolCall("profile", { op: "get", name: "general" })], { stopReason: "toolUse" }),
    (request) => {
      const result = request.messages.findLast((message) => message.role === "toolResult");
      assert.ok(result);
      const text = result.content.flatMap((block) => (block.type === "text" ? [block.text] : [])).join("");
      const profile: unknown = JSON.parse(text);
      assert.ok(Value.Check(ProfileSchema, profile));
      assert.equal(profile.id, "general");
      return fauxAssistantMessage("General agent found.");
    },
  ]);
  await h.send(id, "Find the General agent.");
});
