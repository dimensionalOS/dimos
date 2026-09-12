import { spawn } from "node:child_process";
import { mkdir, rm, rmdir, stat } from "node:fs/promises";
import { dirname, resolve } from "node:path";
import { parseArgs } from "node:util";
import { setTimeout as delay } from "node:timers/promises";
import { paths, loadConfig, saveConfig } from "./config.js";
import { Gateway } from "./gateway.js";
import { Connection, reachable } from "./protocol.js";
import { setup, needsSetup } from "./setup.js";
import { service } from "./service.js";
import { installDimos } from "./install.js";
import { terminal } from "./terminal.js";

const { values, positionals } = parseArgs({
  allowPositionals: true,
  options: {
    foreground: { type: "boolean" },
    view: { type: "boolean" },
    session: { type: "string" },
    cwd: { type: "string" },
    provider: { type: "string" },
    oauth: { type: "boolean" },
    "key-env": { type: "string" },
    help: { type: "boolean" },
  },
});
const p = paths();
let config = await loadConfig(p);
if (values.cwd) config.workspace = resolve(values.cwd);
async function ensureGateway(): Promise<void> {
  if (await reachable(p.socket)) return;
  const lock = p.socket + ".starting";
  try {
    if (Date.now() - (await stat(lock)).mtimeMs > 30000)
      await rm(lock, { recursive: true, force: true });
  } catch {}
  await mkdir(dirname(p.socket), { recursive: true, mode: 0o700 });
  try {
    await mkdir(lock, { mode: 0o700 });
    try {
      if (await reachable(p.socket)) return;
      const child = spawn(
        process.execPath,
        [resolve(process.argv[1]), "gateway"],
        { detached: true, stdio: "ignore", env: process.env },
      );
      child.unref();
      for (let attempt = 0; attempt < 100; attempt++) {
        if (await reachable(p.socket)) return;
        await delay(50);
      }
      throw new Error(
        "Gateway failed to start; run dimcode gateway to inspect the error.",
      );
    } finally {
      await rmdir(lock);
    }
  } catch (error) {
    if (!(error instanceof Error && "code" in error && error.code === "EEXIST"))
      throw error;
    for (let attempt = 0; attempt < 100; attempt++) {
      if (await reachable(p.socket)) return;
      await delay(50);
    }
    throw new Error("Gateway bootstrap did not complete. Inspect " + lock);
  }
}
async function main(): Promise<void> {
  const [command, ...args] = positionals;
  if (values.help || command === "help") {
    console.log(
      "dimcode [tui] [--session ID] [--cwd DIR] [--view]\ndimcode setup [--provider NAME] [--oauth | --key-env VAR]\ndimcode gateway | --foreground | service install/status/uninstall\ndimcode connect NAME MCP_URL | dimos PATH | python PATH | relay URL [ROBOT]\ndimcode install-dimos [VENV] | sessions | stop | run PROMPT | config",
    );
    return;
  }
  if (command === "setup") {
    await setup(p, {
      provider: values.provider,
      oauth: values.oauth,
      keyEnv: values["key-env"],
      workspace: values.cwd,
    });
    return;
  }
  if (command === "config") {
    console.log(JSON.stringify(config, null, 2));
    return;
  }
  if (command === "connect") {
    const [name, url] = args;
    config.mcp = [
      ...config.mcp.filter((endpoint) => endpoint.name !== name),
      { name, url },
    ];
    await saveConfig(p, config);
    console.log("Saved endpoint; reload sessions to discover tools.");
    return;
  }
  if (command === "dimos" || command === "python") {
    if (!args[0]) throw new Error("Executable path required");
    config[command] = resolve(args[0]);
    await saveConfig(p, config);
    return;
  }
  if (command === "relay") {
    [config.relay, config.robot] = args;
    await saveConfig(p, config);
    return;
  }
  if (command === "install-dimos") {
    await installDimos(args[0] ?? config.workspace + "/.venv", config, p);
    return;
  }
  if (command === "service") {
    await service(args[0], config, p, process.argv[1]);
    return;
  }
  if (
    command &&
    !["gateway", "stop", "sessions", "run", "tui"].includes(command)
  )
    throw new Error("Unknown command: " + command + ". Run dimcode --help.");
  if (
    (!command || command === "tui" || values.foreground) &&
    (await needsSetup(p))
  ) {
    await setup(p, { workspace: values.cwd });
    config = await loadConfig(p);
  }
  if (command === "gateway" || values.foreground) {
    if (!values.foreground && (await reachable(p.socket))) {
      console.log(
        "Gateway already running. Open the terminal with dimcode tui.",
      );
      return;
    }
    if (values.foreground) p.socket = p.socket + "." + process.pid;
    const gateway = new Gateway(config, p);
    await gateway.start();
    if (values.foreground) {
      try {
        await terminal(p.socket, { cwd: config.workspace });
      } finally {
        await gateway.close();
      }
      return;
    }
    let closing = false;
    for (const signal of ["SIGTERM", "SIGINT"] as const)
      process.on(signal, () => {
        if (!closing) {
          closing = true;
          void gateway.close().catch(console.error);
        }
      });
    return;
  }
  if (command === "stop") {
    const c = new Connection(p.socket);
    try {
      await c.call({ type: "shutdown" });
    } finally {
      c.close();
    }
    return;
  }
  await ensureGateway();
  if (command === "sessions") {
    const c = new Connection(p.socket);
    try {
      console.log(await c.call({ type: "list_sessions" }));
    } finally {
      c.close();
    }
    return;
  }
  if (command === "run") {
    const c = new Connection(p.socket);
    try {
      await c.call(
        values.session
          ? { type: "attach", sessionId: values.session, writable: true }
          : { type: "new_session", cwd: config.workspace },
      );
      const done = new Promise<void>((resolve, reject) => {
        c.onEvent = (_seq, event) => {
          if (event.type === "text_delta") process.stdout.write(event.delta);
          if (event.type === "notice")
            process.stderr.write(event.message + "\n");
          if (event.type === "turn_error") reject(new Error(event.message));
          if (event.type === "idle") resolve();
        };
        c.onClose = () =>
          reject(new Error("Gateway disconnected before completion"));
      });
      await c.call({ type: "prompt", message: args.join(" ") });
      await done;
      process.stdout.write("\n");
    } finally {
      c.close();
    }
    return;
  }
  await terminal(p.socket, {
    sessionId: values.session,
    cwd: config.workspace,
    view: values.view,
  });
}
await main().catch((error) => {
  console.error(error instanceof Error ? error.message : String(error));
  process.exitCode = 1;
});
