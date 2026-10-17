// A blueprint as GET /blueprints/{name} describes it, and how the view names its parts.

export interface Stream {
  name: string;
  type: string;
  direction: string;
  wire?: string | null;
  topic?: string | null;
}

export interface Method {
  name: string;
  params: { name: string; type: string | null; default: string | null }[];
  return_type: string | null;
  doc: string;
}

export interface Module {
  name: string;
  class: string;
  doc?: string;
  summary?: string;
  streams: Stream[];
  rpcs?: Method[];
  skills?: Method[];
  file?: string | null;
  line?: number | null;
}

export interface Blueprint {
  name: string;
  file: string | null;
  line: number | null;
  modules: Module[];
}

export const reads = (stream: Stream): boolean => stream.direction !== "out";
export const writes = (stream: Stream): boolean => stream.direction !== "in";
export const className = (module: Module): string => module.class.split(".").pop() || module.name;
export const topicOf = (stream: Stream): string =>
  (stream.topic ?? stream.wire ?? stream.name).replace(/^\/+/, "");
/** The module's own name, when it isn't just its class lowercased. */
export const aliasOf = (module: Module): string | null =>
  module.name !== className(module).toLowerCase() ? module.name : null;

/** A path inside the dimos checkout, from the repo root (what the editor and the gateway's /source take). */
export function projectPath(file: string): string {
  const at = file.lastIndexOf("/dimos/");
  return at >= 0 ? file.slice(at + 1) : file;
}

const moduleKey = (name: string) => name.toLowerCase().replace(/[^a-z0-9]/g, "");

/** How many blueprints in the catalog use each module. */
export function moduleUsage(
  catalog: { blueprints?: { modules?: string[] }[] },
): Map<string, number> {
  const uses = new Map<string, number>();
  for (const blueprint of catalog.blueprints ?? []) {
    for (const key of new Set((blueprint.modules ?? []).map(moduleKey))) {
      uses.set(key, (uses.get(key) ?? 0) + 1);
    }
  }
  return uses;
}

/** Rarest first: what makes this blueprint different leads the list. */
export function byRarity(modules: Module[], uses: Map<string, number>): Module[] {
  const count = (module: Module) => uses.get(moduleKey(module.name)) ?? 0;
  return modules.map((module, index) => ({ module, index }))
    .sort((a, b) => count(a.module) - count(b.module) || a.index - b.index)
    .map(({ module }) => module);
}
