import assert from "node:assert/strict";
import { execFile, spawn } from "node:child_process";
import { once } from "node:events";
import { mkdtemp, rm } from "node:fs/promises";
import { createServer, type Socket } from "node:net";
import { tmpdir } from "node:os";
import { join, resolve } from "node:path";
import { test } from "node:test";
import { promisify } from "node:util";
import { paths, saveConfig, configSchema } from "../src/config.js";
import { reachable, readLines, requestSchema, send } from "../src/protocol.js";

const exec = promisify(execFile);
test(
  "CLI opens tui, reports an existing gateway and rejects typos without starting one",
  { timeout: 30000 },
  async (t) => {
    const home = await mkdtemp(join(tmpdir(), "dimcode-cli-"));
    const env = {
      ...process.env,
      DIMCODE_HOME: join(home, "config"),
      XDG_STATE_HOME: join(home, "state"),
      XDG_CACHE_HOME: join(home, "cache"),
      XDG_RUNTIME_DIR: home,
    };
    const cli = resolve(".test/src/main.js");
    const run = (...args: string[]) =>
      exec(process.execPath, [cli, ...args], { env });
    t.after(async () => {
      try {
        await run("stop");
      } catch {}
      await rm(home, { recursive: true, force: true });
    });
    assert.match((await run("--help")).stdout, /dimcode \[tui\]/);
    await assert.rejects(run("typo"), /Unknown command/);
    assert(!(await reachable(paths(env).socket)));
    const key = "cli-fixture-key";
    const configured = await exec(
      process.execPath,
      [
        cli,
        "setup",
        "--provider",
        "openai",
        "--key-env",
        "DIMCODE_FIXTURE_KEY",
        "--cwd",
        home,
      ],
      { env: { ...env, DIMCODE_FIXTURE_KEY: key } },
    );
    assert(!configured.stdout.includes(key));
    const child = spawn(process.execPath, [cli, "tui"], {
      env,
      stdio: ["pipe", "pipe", "pipe"],
    });
    t.after(() => {
      child.kill();
    });
    let output = "",
      sent = false,
      exited = false,
      stderr = "";
    child.stderr.on("data", (chunk) => {
      stderr += chunk;
    });
    child.stdout.on("data", (chunk) => {
      output += chunk;
      if (!sent && output.includes("Ready")) {
        sent = true;
        child.stdin.write("/help\r");
      }
      if (sent && !exited && output.includes("/resume ID")) {
        exited = true;
        child.stdin.end("/exit\r");
      }
    });
    const timer = setTimeout(() => child.kill(), 15000);
    const [code] = await once(child, "exit");
    clearTimeout(timer);
    assert.equal(code, 0, stderr + output);
    assert.match(output, /Build apps/);
    assert.match(output, /\/resume ID/);
    assert.match((await run("gateway")).stdout, /Gateway already running/);
  },
);

test(
  "terminal drops old-session events queued during a session switch",
  { timeout: 15000 },
  async (t) => {
    const home = await mkdtemp(join(tmpdir(), "dimcode-switch-"));
    const env = { ...process.env, DIMCODE_HOME: home, XDG_RUNTIME_DIR: home };
    const p = paths(env);
    await saveConfig(p, configSchema.parse({ workspace: home }));
    const sockets = new Set<Socket>();
    let sessions = 0;
    const server = createServer((socket) => {
      sockets.add(socket);
      socket.on("error", () => {});
      socket.on("close", () => sockets.delete(socket));
      readLines(socket, (value) => {
        const request = requestSchema.parse(value);
        if (request.command.type !== "new_session") return;
        sessions++;
        if (sessions === 2)
          send(socket, {
            type: "event",
            sessionId: "session-a",
            seq: 100,
            event: { type: "notice", message: "STALE_A_EVENT" },
          });
        send(socket, {
          type: "response",
          id: request.id,
          data: {
            sessionId: sessions === 1 ? "session-a" : "session-b",
            seq: 0,
            cwd: home,
            busy: false,
            writable: true,
            messages: [],
            text: "",
            notices: [],
            tools: [],
          },
        });
        if (sessions === 2)
          send(socket, {
            type: "event",
            sessionId: "session-b",
            seq: 1,
            event: { type: "notice", message: "CURRENT_B_EVENT" },
          });
      });
    });
    t.after(async () => {
      for (const socket of sockets) socket.destroy();
      await new Promise<void>((resolve) => server.close(() => resolve()));
      await rm(home, { recursive: true, force: true });
    });
    server.listen(p.socket);
    await once(server, "listening");
    const child = spawn(
      process.execPath,
      [resolve(".test/src/main.js"), "tui"],
      { env, stdio: ["pipe", "pipe", "pipe"] },
    );
    t.after(() => {
      child.kill();
    });
    let output = "",
      switched = false,
      exited = false;
    child.stdout.on("data", (chunk) => {
      output += chunk;
      if (!switched && output.includes("Ready")) {
        switched = true;
        child.stdin.write("/new\r");
      }
      if (!exited && output.includes("CURRENT_B_EVENT")) {
        exited = true;
        child.stdin.end("/exit\r");
      }
    });
    const timer = setTimeout(() => child.kill(), 10000);
    const [code] = await once(child, "exit");
    clearTimeout(timer);
    assert.equal(code, 0, output);
    assert(!output.includes("STALE_A_EVENT"));
    assert(output.includes("CURRENT_B_EVENT"));
  },
);
