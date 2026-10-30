import type { ModelRef } from "@earendil-works/pi-durable";
import type { Live } from "../../shared/protocol/status.ts";
import type { Settings } from "../../shared/protocol/settings.ts";
import { ModelThinkingLevelSchema } from "@earendil-works/pi-ai/providers/model-schema";
import { Status } from "../../shared/protocol/status.ts";
import { Type } from "typebox";
import { Value } from "typebox/value";
import { readJsonFile, writeJsonFile } from "./lib/json-file.ts";

const ModelRefSchema = Type.Unsafe<ModelRef>(Type.Object({ provider: Type.String(), modelId: Type.String() }));

/** `config.json`. The DimOS checkout and Python come from `dimos agent gateway`, not from here. */
const SettingsSchema = Type.Object({
  workspace: Type.Union([Type.String(), Type.Null()]),
  model: Type.Union([ModelRefSchema, Type.Null()]),
  thinkingLevel: Type.Union([ModelThinkingLevelSchema, Type.Null()]),
  theme: Type.Union([Type.String(), Type.Null()]),
});

const DEFAULTS: Settings = { workspace: null, model: null, thinkingLevel: null, theme: null };

/** A missing field takes its default; a field of the wrong type is an error naming it. */
function parseSettings(path: string, json: unknown): Settings {
  const merged = typeof json === "object" && json !== null && !Array.isArray(json) ? { ...DEFAULTS, ...json } : json;
  const [error] = Value.Errors(SettingsSchema, merged);
  if (error !== undefined) throw new Error(`${path}: ${error.instancePath || "the file"} ${error.message}`);
  return Value.Parse(SettingsSchema, merged);
}

/** A field the change leaves out keeps its value. */
function merged(current: Settings, change: Partial<Settings>): Settings {
  return {
    workspace: change.workspace === undefined ? current.workspace : change.workspace,
    model: change.model === undefined ? current.model : change.model,
    thinkingLevel: change.thinkingLevel === undefined ? current.thinkingLevel : change.thinkingLevel,
    theme: change.theme === undefined ? current.theme : change.theme,
  };
}

/**
 * The one reader and writer of config.json; a missing file is the defaults, a malformed one is an error naming the
 * field. A status source of Settings, so a change made by any terminal reaches every terminal.
 */
export class ConfigStore {
  private readonly path: string;
  private readonly settings: Status<Settings>;
  /** Settles when the last update did; the next one merges into what it wrote. */
  private written: Promise<void> = Promise.resolve();
  readonly status: Live<Settings>;

  private constructor(path: string, settings: Settings) {
    this.path = path;
    this.settings = new Status(settings, (error) => {
      throw error;
    });
    this.status = this.settings;
  }

  static async open(path: string): Promise<ConfigStore> {
    const json = await readJsonFile(path);
    return new ConfigStore(path, json === undefined ? DEFAULTS : parseSettings(path, json));
  }

  /** Writes with a temp file and rename, mode 0600, then tells listeners; one update at a time, in call order. */
  update(change: Partial<Settings>): Promise<Settings> {
    const update = this.written.then(async () => {
      const next = merged(this.settings.value, change);
      await writeJsonFile(this.path, next);
      this.settings.set(next);
      return next;
    });
    this.written = update.then(
      () => undefined,
      () => undefined,
    );
    return update;
  }
}
