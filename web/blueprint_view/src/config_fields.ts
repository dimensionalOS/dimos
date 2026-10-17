// The settings the config dialog edits: GlobalConfig's fields (from its JSON schema) and each module's config args,
// each with the kind of input it takes and the value it falls back to.
import {
  type JsonSchema,
  pythonTypeKind,
  resolveSchema,
  schemaKind,
  schemaNullable,
  type SettingField,
} from "dimos-react";

export interface FieldSpec extends SettingField {
  id: string;
  scope: "global" | "module";
  module?: string;
  nullable: boolean;
  description: string | null;
  base: unknown;
  group: string;
  groupLabel?: string;
  secret: boolean;
  label?: string;
  docs?: string;
}

export interface GlobalConfig {
  schema?: JsonSchema;
  defaults?: Record<string, unknown>;
  overrides?: Record<string, unknown>;
  secrets?: string[];
}

export interface ConfigArg {
  name: string;
  type?: string | null;
  default?: unknown;
  description?: string | null;
  base?: boolean;
  json_compatible?: boolean;
  secret?: boolean;
  choices?: unknown[];
  schema?: JsonSchema;
  value?: unknown;
}

export interface BlueprintConfig {
  name?: string;
  modules: { module: string; class?: string; args: ConfigArg[]; error?: string }[];
  overrides?: Record<string, Record<string, unknown>>;
}

function settable(arg: ConfigArg): boolean {
  if (arg.json_compatible !== undefined) {
    return arg.json_compatible;
  }
  const type = arg.type ?? "";
  return pythonTypeKind(type).kind !== "json" || !!arg.choices ||
    /^(list|dict|tuple)\[(str|int|float|bool|,|\s)*\]$/.test(type);
}

/** Global keys sharing a first word (`robot_ip`, `robot_name`) group under it; the rest are "general". */
function globalGroups(keys: string[]): Map<string, string> {
  const word = (key: string) => key.toLowerCase().split("_")[0];
  const counts = new Map<string, number>();
  for (const key of keys) {
    counts.set(word(key), (counts.get(word(key)) ?? 0) + 1);
  }
  return new Map(keys.map((key) => [key, counts.get(word(key))! > 1 ? word(key) : "general"]));
}

export function globalFields(config: GlobalConfig): FieldSpec[] {
  const properties: JsonSchema = config.schema?.properties ?? {};
  const keys = [...new Set([...Object.keys(properties), ...Object.keys(config.defaults ?? {})])];
  const groups = globalGroups(keys);
  const order = (key: string) =>
    `${groups.get(key) === "general" ? "" : groups.get(key)}\u0000${key.toLowerCase()}`;
  keys.sort((a, b) => order(a).localeCompare(order(b)));
  return keys.flatMap((key): FieldSpec[] => {
    const raw = properties[key] ?? {};
    const schema = resolveSchema(raw, config.schema ?? {});
    const kind = schemaKind(schema);
    if (kind === "json") {
      return [];
    }
    return [{
      id: key,
      scope: "global",
      name: key,
      kind,
      options: schema.enum,
      nullable: schemaNullable(raw),
      description: schema.description ?? null,
      base: config.defaults?.[key] ?? schema.default ?? null,
      group: groups.get(key) ?? "general",
      secret: config.secrets?.includes(key) ?? false,
    }];
  });
}

export function moduleFields(config: BlueprintConfig): FieldSpec[] {
  return config.modules.flatMap((module) =>
    module.args.filter((arg) => !arg.base && settable(arg)).map((arg): FieldSpec => {
      const fromSchema = arg.schema ? schemaKind(resolveSchema(arg.schema)) : null;
      const annotation = pythonTypeKind(arg.type);
      return {
        id: `${module.module}.${arg.name}`,
        scope: "module",
        module: module.module,
        name: arg.name,
        kind: arg.choices ? "enum" : fromSchema ?? annotation.kind,
        options: arg.choices,
        nullable: arg.schema ? schemaNullable(arg.schema) : annotation.nullable,
        description: arg.description ?? null,
        base: "value" in arg ? arg.value : arg.default,
        group: module.module,
        groupLabel: module.class ? module.class.split(".").pop() : undefined,
        secret: arg.secret ?? false,
      };
    })
  );
}
