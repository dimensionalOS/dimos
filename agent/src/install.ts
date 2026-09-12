import { access } from "node:fs/promises";
import { join, resolve } from "node:path";
import { saveConfig, type Config, type Paths } from "./config.js";
import { execute } from "./service.js";

export async function installDimos(
  destination: string,
  config: Config,
  paths: Paths,
): Promise<void> {
  const venv = resolve(destination);
  try {
    await access(venv);
  } catch (error) {
    if (!(error instanceof Error && "code" in error && error.code === "ENOENT"))
      throw error;
    await execute("uv", ["venv", "--python", "3.12", venv]);
    const python = join(venv, "bin/python");
    await execute("uv", ["pip", "install", "--python", python, "dimos"]);
    config.python = python;
    config.dimos = join(venv, "bin/dimos");
    await saveConfig(paths, config);
    return;
  }
  throw new Error(
    "Destination exists; select its dimos/python executables or choose a new environment.",
  );
}
