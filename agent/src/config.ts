import { mkdir, readFile, writeFile } from "node:fs/promises";
import { homedir } from "node:os";
import { join, resolve } from "node:path";
import { z } from "zod";

export const endpointSchema = z.object({
  name: z.string().regex(/^[a-z][a-z0-9_-]*$/),
  url: z.url(),
});
export const configSchema = z.object({
  workspace: z.string().default(process.cwd()),
  python: z.string().optional(),
  dimos: z.string().optional(),
  mcp: z.array(endpointSchema).default([]),
  relay: z.url().optional(),
  robot: z.string().optional(),
});
export type Config = z.infer<typeof configSchema>;
export type Endpoint = z.infer<typeof endpointSchema>;
export const paths = (env: NodeJS.ProcessEnv = process.env) => {
  const home = homedir();
  const config = resolve(
    env.DIMCODE_HOME ??
      join(env.XDG_CONFIG_HOME ?? join(home, ".config"), "dimcode"),
  );
  const state = join(
    env.XDG_STATE_HOME ?? join(home, ".local/state"),
    "dimcode",
  );
  return {
    config,
    state,
    cache: join(env.XDG_CACHE_HOME ?? join(home, ".cache"), "dimcode"),
    sessions: join(state, "sessions"),
    socket: join(env.XDG_RUNTIME_DIR ?? state, "dimcode.sock"),
  };
};
export type Paths = ReturnType<typeof paths>;
export async function loadConfig(p: Paths): Promise<Config> {
  try {
    return configSchema.parse(
      JSON.parse(await readFile(join(p.config, "config.json"), "utf8")),
    );
  } catch (error) {
    if (error instanceof Error && "code" in error && error.code === "ENOENT")
      return configSchema.parse({});
    throw error;
  }
}
export async function saveConfig(p: Paths, config: Config): Promise<void> {
  await mkdir(p.config, { recursive: true, mode: 0o700 });
  await writeFile(
    join(p.config, "config.json"),
    JSON.stringify(configSchema.parse(config), null, 2) + "\n",
    { mode: 0o600 },
  );
}
