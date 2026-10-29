import type { Fact, FactKey, Facts, RuntimeChange } from "#dimos/agents/shared/protocol/runtime/context.ts";

/** The one fact while DimOS cannot be reached; the system's facts stay as they were last stated. */
export const STATUS_KEY: FactKey = "dimos:status";

/** Kinds about DimOS itself: the status and `run:<run id>` per file in the runs directory. A change states them as one notice line. */
const NOTICE_KINDS = new Set(["dimos", "run"]);

/** Kinds the gateway states itself rather than DimOS's inventory: the notice kinds and `job:<name>` per cron job. */
const GATEWAY_KINDS = new Set([...NOTICE_KINDS, "job"]);

/** What the gateway saw of DimOS for one input: the compact inventory, or why it got none, and its own facts. */
export type Observation =
  | { readonly kind: "up"; readonly system: Facts; readonly gateway: Facts }
  | { readonly kind: "down"; readonly status: Fact; readonly gateway: Facts };

export function factEntries(facts: Facts): [FactKey, Fact][] {
  return Object.entries(facts) as [FactKey, Fact][];
}

function kindOf(key: FactKey): string {
  return key.slice(0, key.indexOf(":"));
}

export function isNoticeFact(key: FactKey): boolean {
  return NOTICE_KINDS.has(kindOf(key));
}

/** The facts among `facts` from DimOS's inventory rather than the gateway's own. */
export function systemFacts(facts: Facts): Facts {
  return Object.fromEntries(factEntries(facts).filter(([key]) => !GATEWAY_KINDS.has(kindOf(key))));
}

/** The facts an observation states to a conversation told `told`: while DimOS is down, its system as last seen. */
export function observedFacts(told: Facts, observation: Observation): Facts {
  if (observation.kind === "up") return { ...observation.system, ...observation.gateway };
  return { ...systemFacts(told), [STATUS_KEY]: observation.status, ...observation.gateway };
}

/** Null when nothing differs. A full restatement only for the first system the conversation sees; a delta otherwise. */
export function diffFacts(told: Facts, now: Facts): RuntimeChange | null {
  const set: { [key: FactKey]: Fact } = {};
  for (const [key, fact] of factEntries(now)) {
    const before = told[key];
    if (before === undefined || before.line !== fact.line || before.section !== fact.section) set[key] = fact;
  }
  const removed: { [key: FactKey]: Fact } = {};
  for (const [key, fact] of factEntries(told)) if (!(key in now)) removed[key] = fact;
  if (Object.keys(set).length + Object.keys(removed).length === 0) return null;
  if (Object.keys(systemFacts(told)).length === 0 && Object.keys(systemFacts(now)).length > 0) {
    return { kind: "full", facts: now, summary: "full runtime state" };
  }
  return { kind: "delta", set, removed, summary: deltaSummary(told, set, removed) };
}

export function applyChange(told: Facts, change: RuntimeChange): Facts {
  if (change.kind === "full") return change.facts;
  const next: { [key: FactKey]: Fact } = { ...told, ...change.set };
  for (const [key] of factEntries(change.removed)) delete next[key];
  return next;
}

/** What happened to DimOS itself, then per section "+added ~changed −removed", joined by " · ". */
function deltaSummary(told: Facts, set: Facts, removed: Facts): string {
  const events = [
    STATUS_KEY in set ? "DimOS unreachable" : STATUS_KEY in removed ? "DimOS reachable" : "",
    Object.keys(removed).some((key) => key.startsWith("run:")) ? "run ended" : "",
    Object.keys(set).some((key) => key.startsWith("run:") && !(key in told)) ? "run started" : "",
  ];
  const counts = new Map<string, [number, number, number]>();
  const count = (section: string, index: 0 | 1 | 2): void => {
    const entry = counts.get(section) ?? [0, 0, 0];
    entry[index] += 1;
    counts.set(section, entry);
  };
  for (const [key, fact] of factEntries(set)) if (!isNoticeFact(key)) count(fact.section, key in told ? 1 : 0);
  for (const [key, fact] of factEntries(removed)) if (!isNoticeFact(key)) count(fact.section, 2);
  const sections = [...counts].map(([section, [added, changed, gone]]) =>
    [section, added ? `+${added}` : "", changed ? `~${changed}` : "", gone ? `−${gone}` : ""].filter(Boolean).join(" "),
  );
  return [...events.filter(Boolean), ...sections].join(" · ");
}
