import type { AgentPaths } from "./paths.ts";
import type { ModelRef } from "@earendil-works/pi-durable";
import type { ModelThinkingLevel } from "@earendil-works/pi-ai";
import { ConfigStore } from "./store.ts";
import { ModelThinkingLevelSchema } from "@earendil-works/pi-ai/providers/model-schema";
import { Value } from "typebox/value";
import { access, chmod, copyFile, mkdir } from "node:fs/promises";
import { constants } from "node:fs";
import { join } from "node:path";
import { readJsonFile } from "./lib/json-file.ts";

type MigrationResult =
  | { readonly kind: "copied"; readonly files: readonly string[] }
  /** config.json already exists, so the migration ran before or the user configured the agent. */
  | { readonly kind: "already" }
  | { readonly kind: "nothing" };

type LegacySettings = {
  readonly workspace: string | null;
  readonly model: ModelRef | null;
  readonly thinkingLevel: ModelThinkingLevel | null;
  readonly theme: string | null;
};

async function exists(path: string): Promise<boolean> {
  try {
    await access(path);
    return true;
  } catch {
    return false;
  }
}

function field(json: unknown, name: string): unknown {
  return typeof json === "object" && json !== null && !Array.isArray(json)
    ? Object.getOwnPropertyDescriptor(json, name)?.value
    : undefined;
}

function text(value: unknown): string | null {
  return typeof value === "string" && value !== "" ? value : null;
}

/** dimcode's `config.json` gave the workspace; Pi's `settings.json` the default model, thinking level and theme. Fields of the wrong type are left out. */
function legacySettings(config: unknown, settings: unknown): LegacySettings {
  const provider = text(field(settings, "defaultProvider"));
  const modelId = text(field(settings, "defaultModel"));
  const thinking = field(settings, "defaultThinkingLevel");
  return {
    workspace: text(field(config, "workspace")),
    model: provider !== null && modelId !== null ? { provider, modelId } : null,
    thinkingLevel: Value.Check(ModelThinkingLevelSchema, thinking) ? thinking : null,
    theme: text(field(settings, "theme")),
  };
}

/**
 * Copies `~/.dimcode/auth.json` and folds its `config.json` and `settings.json` into config.json, which is written
 * last, so an interrupted migration runs again. The originals are only read.
 */
export async function migrateDimcode(paths: AgentPaths): Promise<MigrationResult> {
  if (await exists(paths.configFile)) return { kind: "already" };
  const legacyAuth = join(paths.legacyDir, "auth.json");
  const legacyConfig = join(paths.legacyDir, "config.json");
  const legacySettingsFile = join(paths.legacyDir, "settings.json");
  const copied: string[] = [];
  if ((await exists(legacyAuth)) && !(await exists(paths.authFile))) {
    await mkdir(paths.configDir, { recursive: true, mode: 0o700 });
    await copyFile(legacyAuth, paths.authFile, constants.COPYFILE_EXCL);
    await chmod(paths.authFile, 0o600);
    copied.push(legacyAuth);
  }
  const config = await readJsonFile(legacyConfig).catch(() => undefined);
  const settings = await readJsonFile(legacySettingsFile).catch(() => undefined);
  if (config !== undefined) copied.push(legacyConfig);
  if (settings !== undefined) copied.push(legacySettingsFile);
  if (copied.length === 0) return { kind: "nothing" };
  const store = await ConfigStore.open(paths.configFile);
  await store.update(legacySettings(config, settings));
  return { kind: "copied", files: copied };
}
