import { spawn } from "node:child_process";
import { mkdir, unlink, writeFile } from "node:fs/promises";
import { homedir } from "node:os";
import { join, resolve } from "node:path";
import { setTimeout as delay } from "node:timers/promises";
import { Connection, reachable } from "./protocol.js";
import type { Config, Paths } from "./config.js";

export function execute(
  command: string,
  args: readonly string[],
  options: { cwd?: string; env?: NodeJS.ProcessEnv } = {},
): Promise<void> {
  return new Promise((resolve, reject) => {
    const child = spawn(command, args, { ...options, stdio: "inherit" });
    child.once("error", reject);
    child.once("exit", (code, signal) =>
      code === 0
        ? resolve()
        : reject(new Error(command + " exited " + (code ?? signal))),
    );
  });
}
const quote = (value: string) =>
  '"' +
  value.replace(/%/g, "%%").replace(/\\/g, "\\\\").replace(/"/g, '\\"') +
  '"';
export function serviceUnit(
  config: Config,
  paths: Paths,
  executable: string,
): string {
  return [
    "[Unit]",
    "Description=dimcode agent gateway",
    "",
    "[Service]",
    "Type=simple",
    "ExecStart=" +
      quote(process.execPath) +
      " " +
      quote(executable) +
      " gateway",
    "WorkingDirectory=" + config.workspace.replace(/%/g, "%%"),
    "Environment=" + quote("DIMCODE_HOME=" + paths.config),
    "Restart=on-failure",
    "RestartSec=2",
    "UMask=0077",
    "",
    "[Install]",
    "WantedBy=default.target",
    "",
  ].join("\n");
}
export async function service(
  action: string,
  config: Config,
  paths: Paths,
  executable: string,
): Promise<void> {
  const dir = join(
    process.env.XDG_CONFIG_HOME ?? join(homedir(), ".config"),
    "systemd/user",
  );
  const unit = join(dir, "dimcode.service");
  if (action === "install") {
    const existing = new Connection(paths.socket);
    try {
      await existing.call({ type: "shutdown" });
    } catch {
    } finally {
      existing.close();
    }
    for (let attempt = 0; await reachable(paths.socket); attempt++) {
      if (attempt === 100)
        throw new Error(
          "Existing gateway did not stop; inspect dimcode service status.",
        );
      await delay(50);
    }
    await mkdir(dir, { recursive: true });
    await writeFile(unit, serviceUnit(config, paths, resolve(executable)), {
      mode: 0o600,
    });
    await execute("systemctl", ["--user", "daemon-reload"]);
    await execute("systemctl", [
      "--user",
      "enable",
      "--now",
      "dimcode.service",
    ]);
  } else if (action === "uninstall") {
    await execute("systemctl", [
      "--user",
      "disable",
      "--now",
      "dimcode.service",
    ]);
    await unlink(unit);
    await execute("systemctl", ["--user", "daemon-reload"]);
  } else if (action === "status")
    await execute("systemctl", ["--user", "status", "dimcode.service"]);
  else throw new Error("Use service install, status or uninstall");
}
