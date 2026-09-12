import assert from "node:assert/strict";
import { execFile, spawn } from "node:child_process";
import { once } from "node:events";
import { mkdtemp, mkdir, rm, stat } from "node:fs/promises";
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
    assert(
      !(await reachable(paths(env).socket)),
      "provider-only setup does not launch an agent",
    );
    const workspace = join(home, "app with spaces");
    await mkdir(workspace);
    await run("workspace", workspace);
    assert.equal(JSON.parse((await run("config")).stdout).workspace, workspace);
    await assert.rejects(run("workspace", cli), /must be a directory/);
    await assert.rejects(
      run("install-dimos", join(home, "unexpected-env")),
      /Unknown command/,
    );
    await assert.rejects(stat(join(home, "unexpected-env")), {
      code: "ENOENT",
    });
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
  "interactive setup hands off once to a normal agent session without installing DimOS",
  { timeout: 20000, skip: process.platform !== "linux" },
  async (t) => {
    const home = await mkdtemp(join(tmpdir(), "dimcode-onboard-"));
    t.after(() => rm(home, { recursive: true, force: true }));
    const env = {
      ...process.env,
      DIMCODE_HOME: join(home, "config"),
      XDG_STATE_HOME: join(home, "state"),
      XDG_CACHE_HOME: join(home, "cache"),
      XDG_CONFIG_HOME: join(home, "xdg-config"),
      XDG_RUNTIME_DIR: home,
      DIMCODE_FIXTURE_KEY: "onboarding-private-fixture",
      TERM: "xterm-256color",
    };
    const cli = resolve(".test/src/main.js");
    await exec(
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
      { env },
    );
    const sockets = new Set<Socket>();
    let prompts = 0;
    const server = createServer((socket) => {
      sockets.add(socket);
      socket.on("error", () => {});
      socket.on("close", () => sockets.delete(socket));
      readLines(socket, (value) => {
        const request = requestSchema.parse(value);
        if (request.command.type === "new_session") {
          send(socket, {
            type: "response",
            id: request.id,
            data: {
              sessionId: "onboarding",
              cwd: home,
              seq: 0,
              busy: false,
              writable: true,
              messages: [],
              text: "",
              notices: [],
              tools: [],
            },
          });
        } else if (request.command.type === "prompt") {
          prompts++;
          assert.match(request.command.message, /dimensional-install skill/);
          assert.match(request.command.message, /Wait for my answer/);
          send(socket, { type: "response", id: request.id, data: null });
          send(socket, {
            type: "event",
            sessionId: "onboarding",
            seq: 1,
            event: {
              type: "notice",
              message: "fixture: agent received setup request",
            },
          });
        }
      });
    });
    await new Promise<void>((done) => server.listen(paths(env).socket, done));
    t.after(() => {
      for (const socket of sockets) socket.destroy();
      return new Promise<void>((done) => server.close(() => done()));
    });
    const quote = (arg: string) => "'" + arg.replaceAll("'", "'\\''") + "'";
    const child = spawn(
      "script",
      [
        "-q",
        "-e",
        "-c",
        [process.execPath, cli, "setup"].map(quote).join(" "),
        "/dev/null",
      ],
      { env, stdio: ["pipe", "pipe", "pipe"] },
    );
    t.after(() => child.kill());
    let output = "",
      stage = 0;
    const screens = [
      "1. Choose your model provider",
      "Sign in",
      "2. Choose a model",
      "3. Start the gateway at login?",
      "fixture: agent received setup request",
    ];
    child.stdout.on("data", (chunk) => {
      output += chunk;
      if (stage < screens.length && output.includes(screens[stage])) {
        stage++;
        child.stdin.write(stage === screens.length ? "/exit\r" : "\r");
      }
    });
    let errors = "";
    child.stderr.on("data", (chunk) => {
      errors += chunk;
    });
    const timer = setTimeout(() => child.kill(), 15000);
    const [code] = await once(child, "exit");
    clearTimeout(timer);
    assert.equal(code, 0, errors + output);
    assert.equal(stage, screens.length, output);
    assert.equal(prompts, 1);
    assert(!output.includes(env.DIMCODE_FIXTURE_KEY));
    await assert.rejects(stat(join(home, ".venv")), { code: "ENOENT" });
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
