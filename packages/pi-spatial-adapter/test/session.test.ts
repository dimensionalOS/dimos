import test from "node:test";
import assert from "node:assert/strict";
import { existsSync, mkdirSync, readFileSync, rmSync, statSync, symlinkSync, writeFileSync } from "node:fs";
import { join } from "node:path";
import { execFileSync } from "node:child_process";
import type { Message } from "@earendil-works/pi-ai";
import { SessionManager } from "@earendil-works/pi-coding-agent";
import {
  PINNED_PI_VERSION,
  SESSION_DIR_ENV,
  createSessionManager,
  modelProviderForAuthMode,
  requiredApiForAuthMode,
  retainSystemPromptEvidence,
  resolvePinnedPiExportCommand,
  sessionEvidenceForManager,
} from "../src/session.js";

test("auth mode selects the matching Pi provider and request API", () => {
  assert.equal(modelProviderForAuthMode("codex-oauth"), "openai-codex");
  assert.equal(requiredApiForAuthMode("codex-oauth"), "openai-codex-responses");
  assert.equal(modelProviderForAuthMode("openai-api-key"), "openai");
  assert.equal(requiredApiForAuthMode("openai-api-key"), "openai-responses");
});

function withSessionDirectory<T>(name: string, callback: (directory: string) => T): T {
  const previous = process.env[SESSION_DIR_ENV];
  process.env[SESSION_DIR_ENV] = name;
  const directory = join(process.cwd(), name);
  try {
    return callback(directory);
  } finally {
    if (previous === undefined) delete process.env[SESSION_DIR_ENV];
    else process.env[SESSION_DIR_ENV] = previous;
    rmSync(directory, { recursive: true, force: true });
    rmSync(join(process.cwd(), "pi-prompt"), { recursive: true, force: true });
  }
}

test("file-backed session directory accepts only a simple relative name", () => {
  for (const value of ["", ".", "..", "/tmp/pi", "../pi", "nested/pi", "pi\\session"]) {
    process.env[SESSION_DIR_ENV] = value;
    assert.throws(() => createSessionManager(), /PI_SPATIAL_SESSION_DIR/);
  }
  delete process.env[SESSION_DIR_ENV];
  assert.throws(() => createSessionManager(), /PI_SPATIAL_SESSION_DIR/);
});

test("precreated symlink and non-directory session children are rejected", () => {
  const symlinkName = `pi-session-link-${process.pid}`;
  const fileName = `pi-session-file-${process.pid}`;
  const symlink = join(process.cwd(), symlinkName);
  const file = join(process.cwd(), fileName);
  rmSync(symlink, { recursive: true, force: true });
  rmSync(file, { recursive: true, force: true });
  mkdirSync(join(process.cwd(), `pi-session-target-${process.pid}`), { mode: 0o700 });
  try {
    const previous = process.env[SESSION_DIR_ENV];
    process.env[SESSION_DIR_ENV] = symlinkName;
    // The symlink is created outside the adapter so the real lstat admission path is tested.
    symlinkSync(`pi-session-target-${process.pid}`, symlink);
    assert.throws(() => createSessionManager(), /real directory/);
    writeFileSync(file, "not a directory");
    process.env[SESSION_DIR_ENV] = fileName;
    assert.throws(() => createSessionManager(), /real directory/);
    if (previous === undefined) delete process.env[SESSION_DIR_ENV];
    else process.env[SESSION_DIR_ENV] = previous;
  } finally {
    rmSync(symlink, { recursive: true, force: true });
    rmSync(file, { recursive: true, force: true });
    rmSync(join(process.cwd(), `pi-session-target-${process.pid}`), { recursive: true, force: true });
    delete process.env[SESSION_DIR_ENV];
  }
});

test("fresh sessions are persisted, distinct, and discoverable through public APIs", () => {
  withSessionDirectory(`pi-session-test-${process.pid}`, (directory) => {
    const first = createSessionManager();
    const second = createSessionManager();
    assert.equal(first.isPersisted(), true);
    assert.equal(second.isPersisted(), true);
    assert.ok(first.getSessionFile());
    assert.ok(second.getSessionFile());
    assert.notEqual(first.getSessionFile(), second.getSessionFile());
    assert.equal(first.getSessionDir(), directory);
    assert.equal(second.getSessionDir(), directory);
    assert.equal(existsSync(first.getSessionFile() ?? ""), false);
    assert.equal(existsSync(second.getSessionFile() ?? ""), false);
  });
});

test("a persisted manager with a delayed nonexistent file is unavailable", () => {
  withSessionDirectory(`pi-session-delayed-${process.pid}`, () => {
    const manager = createSessionManager();
    assert.equal(manager.isPersisted(), true);
    assert.ok(manager.getSessionFile());
    assert.equal(existsSync(manager.getSessionFile() ?? ""), false);
    assert.deepEqual(sessionEvidenceForManager(manager, true), { state: "unavailable", persisted: false });
  });
});

test("system prompt evidence preserves exact unicode bytes with bounded metadata", () => {
  withSessionDirectory(`pi-session-prompt-${process.pid}`, () => {
    const prompt = "system π\n用户—✅";
    const metadata = retainSystemPromptEvidence(prompt);
    const path = join(process.cwd(), metadata.relativePath);
    assert.equal(metadata.relativePath, "pi-prompt/system.txt");
    assert.deepEqual(readFileSync(path), Buffer.from(prompt, "utf8"));
    assert.equal(metadata.byteCount, Buffer.byteLength(prompt, "utf8"));
    assert.match(metadata.sha256, /^[a-f0-9]{64}$/);
    assert.equal(statSync(path).mode & 0o777, 0o600);
    assert.equal(statSync(join(process.cwd(), "pi-prompt")).mode & 0o777, 0o700);
    assert.throws(() => retainSystemPromptEvidence("replacement"), /EEXIST/);
    assert.deepEqual(readFileSync(path), Buffer.from(prompt, "utf8"));
  });
});

test("native JSONL remains after a model-independent public-API append", () => {
  withSessionDirectory(`pi-session-survival-${process.pid}`, (directory) => {
    const manager = createSessionManager();
    const user: Message = { role: "user", content: [{ type: "text", text: "hello" }], timestamp: Date.now() };
    const assistant: Message = {
      role: "assistant",
      content: [{ type: "text", text: "done" }],
      api: "openai-responses",
      provider: "openai-codex",
      model: "gpt-5.6-luna",
      usage: {
        input: 0,
        output: 0,
        cacheRead: 0,
        cacheWrite: 0,
        totalTokens: 0,
        cost: { input: 0, output: 0, cacheRead: 0, cacheWrite: 0, total: 0 },
      },
      stopReason: "stop",
      timestamp: Date.now(),
    };
    manager.appendMessage(user);
    manager.appendMessage(assistant);
    const file = manager.getSessionFile();
    assert.ok(file);
    assert.equal(existsSync(file), true);
    assert.equal(manager.isPersisted(), true);
    assert.equal(manager.getEntries().length, 2);
    assert.equal(existsSync(file), true);
    assert.equal(statSync(file).mode & 0o777, 0o600);
    assert.equal(manager.getSessionDir(), directory);
  });
});

test("pinned SessionManager.open reopens an unchanged native v3 session", () => {
  withSessionDirectory(`pi-session-reopen-${process.pid}`, (directory) => {
    const original = createSessionManager();
    const timestamp = 1_700_000_000_000;
    const userId = original.appendMessage({ role: "user", content: [{ type: "text", text: "native user" }], timestamp });
    const assistantId = original.appendMessage({
      role: "assistant",
      content: [{ type: "text", text: "native assistant" }],
      api: "openai-responses",
      provider: "openai-codex",
      model: "gpt-5.6-luna",
      usage: {
        input: 1,
        output: 2,
        cacheRead: 0,
        cacheWrite: 0,
        totalTokens: 3,
        cost: { input: 0, output: 0, cacheRead: 0, cacheWrite: 0, total: 0 },
      },
      stopReason: "stop",
      timestamp: timestamp + 1,
    });
    const thinkingId = original.appendThinkingLevelChange("medium");
    const modelId = original.appendModelChange("openai-codex", "gpt-5.6-luna");
    const customId = original.appendCustomEntry("adapter-test", { stable: true });
    const file = original.getSessionFile();
    assert.ok(file);
    const sourceBytes = readFileSync(file);

    // SessionManager writes synchronously and has no separate close operation;
    // opening the generated file is the documented handoff lifecycle.
    const reopened = SessionManager.open(file, directory);
    assert.deepEqual(readFileSync(file), sourceBytes);
    assert.equal(reopened.getSessionFile(), file);
    assert.equal(reopened.getSessionDir(), directory);
    assert.equal(reopened.getHeader()?.type, "session");
    assert.equal(reopened.getHeader()?.version, 3);
    assert.equal(reopened.getHeader()?.id, original.getSessionId());
    assert.equal(reopened.getSessionId(), original.getSessionId());
    assert.deepEqual(reopened.getEntries().map((entry) => entry.id), [userId, assistantId, thinkingId, modelId, customId]);
    assert.equal(reopened.getEntry(assistantId)?.parentId, userId);
    assert.equal(reopened.getLeafId(), customId);
    assert.equal(reopened.getLeafEntry()?.id, customId);
    assert.equal(reopened.getTree().length, 1);
    assert.equal(reopened.getTree()[0]?.children.length, 1);
    assert.deepEqual(reopened.getBranch(), reopened.getEntries());
    assert.deepEqual(readFileSync(file), sourceBytes);
  });
});

test("pinned Pi CLI exports a synthetic native session without auth or rewriting JSONL", () => {
  withSessionDirectory(`pi-session-export-${process.pid}`, (directory) => {
    const manager = createSessionManager();
    manager.appendMessage({ role: "user", content: [{ type: "text", text: "hello" }], timestamp: Date.now() });
    manager.appendMessage({
      role: "assistant", content: [{ type: "text", text: "done" }], api: "openai-responses", provider: "openai-codex", model: "gpt-5.6-luna",
      usage: { input: 0, output: 0, cacheRead: 0, cacheWrite: 0, totalTokens: 0, cost: { input: 0, output: 0, cacheRead: 0, cacheWrite: 0, total: 0 } }, stopReason: "stop", timestamp: Date.now(),
    });
    const file = manager.getSessionFile();
    assert.ok(file);
    const before = readFileSync(file);
    const html = join(directory, "export.html");
    const command = resolvePinnedPiExportCommand(file, html);
    assert.equal(command.executable, process.execPath);
    assert.deepEqual(command.args.slice(1), ["--export", file, html]);
    assert.equal(command.packageVersion, PINNED_PI_VERSION);
    execFileSync(command.executable, command.args, { stdio: "pipe" });
    assert.ok(readFileSync(html).length > 0);
    assert.deepEqual(readFileSync(file), before);
  });
});

test("pinned package command validates executable and bin metadata", () => {
  const command = resolvePinnedPiExportCommand("input.jsonl", "output.html");
  assert.equal(command.executable, process.execPath);
  assert.match(command.args[0], /node_modules[\\/]@earendil-works[\\/]pi-coding-agent[\\/]dist[\\/]cli\.js$/);
  assert.equal(PINNED_PI_VERSION, "0.80.10");
});
