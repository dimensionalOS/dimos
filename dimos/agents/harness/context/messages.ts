import type { FactKey, Facts, RuntimeChange } from "../../shared/protocol/runtime/context.ts";
import { STATUS_KEY, factEntries, isNoticeFact } from "./facts.ts";

/** A removal of more facts than this, emptying a section, is stated as one line. */
const COLLAPSE_REMOVALS = 5;

/**
 * The text a change contributes to an input: `<dimos_runtime>` for a full restatement, `<dimos_runtime_change>` for a
 * delta: one notice line for DimOS itself (status, runs ended and started), then `+`, `~` and `-` lines by section.
 * `told` is what the conversation knew before the change; a set fact it already held is shown as changed.
 */
export function changeMessage(change: RuntimeChange, told: Facts): string {
  if (change.kind === "full") {
    return (
      "<dimos_runtime>\nCurrent DimOS runtime. It replaces the runtime state stated earlier; it comes from the gateway, not from the user.\n" +
      sectionsOf(change.facts, "") +
      "\n</dimos_runtime>"
    );
  }
  const notice = noticeOf(change.set, change.removed);
  const lines = new Map<string, string[]>();
  const under = (section: string): string[] => {
    const found = lines.get(section);
    if (found !== undefined) return found;
    const created: string[] = [];
    lines.set(section, created);
    return created;
  };
  const kept = new Set<string>();
  for (const [key, fact] of factEntries(change.set)) if (!isNoticeFact(key)) kept.add(fact.section);
  for (const [key, fact] of factEntries(change.set)) {
    if (!isNoticeFact(key)) under(fact.section).push(`${key in told ? "~ " : "+ "}${fact.line}`);
  }
  const gone = factEntries(change.removed).filter(([key]) => key.startsWith("module:"));
  const names = gone.map(([key]) => key.slice("module:".length));
  const owned = (key: FactKey): boolean =>
    /^(module|rpc|stream|subscription):/.test(key) &&
    names.some((name) => key === `module:${name}` || key.startsWith(`${key.slice(0, key.indexOf(":"))}:${name}.`));
  const removedBySection = new Map<string, string[]>();
  for (const [key, fact] of factEntries(change.removed)) {
    if (isNoticeFact(key) || owned(key)) continue;
    removedBySection.set(fact.section, [...(removedBySection.get(fact.section) ?? []), fact.line]);
  }
  for (const [section, removed] of removedBySection) {
    if (removed.length > COLLAPSE_REMOVALS && !kept.has(section))
      under(section).push(`- all ${removed.length} (no longer present)`);
    else under(section).push(...removed.map((line) => `- ${line}`));
  }
  const [first] = gone;
  if (first !== undefined)
    under(first[1].section).push(
      `- ${names.join(", ")} (with ${names.length === 1 ? "its" : "their"} RPCs and streams)`,
    );
  const body: string[] = [];
  for (const [section, items] of lines) body.push(`${section}:`, ...items.map((line) => `  ${line}`));
  return [
    "<dimos_runtime_change>",
    ...(notice === null ? [] : [notice]),
    ...(body.length === 0
      ? []
      : [
          "DimOS runtime changes since the runtime state stated earlier; they come from the gateway, not from the user.",
          ...body,
        ]),
    "</dimos_runtime_change>",
  ].join("\n");
}

/** e.g. "DimOS: reachable again. Run A ended. Run B (blueprint unitree-go2) started." Null when DimOS itself is unchanged. */
function noticeOf(set: Facts, removed: Facts): string | null {
  const sentences: string[] = [];
  const status = set[STATUS_KEY];
  if (status !== undefined) sentences.push(`${status.line}.`);
  else if (STATUS_KEY in removed) sentences.push("DimOS: reachable again.");
  for (const [key, fact] of factEntries(removed))
    if (key.startsWith("run:") && !(key in set)) sentences.push(`Run ${key.slice("run:".length)} ended.`);
  for (const [key, fact] of factEntries(set))
    if (key.startsWith("run:") && !(key in removed)) sentences.push(`${fact.line} started.`);
  return sentences.length === 0 ? null : sentences.join(" ");
}

/** Facts grouped by section in first-seen order, each line prefixed. */
export function sectionsOf(facts: Facts, prefix: string): string {
  const lines = new Map<string, string[]>();
  for (const [, fact] of factEntries(facts))
    lines.set(fact.section, [...(lines.get(fact.section) ?? []), `  ${prefix}${fact.line}`]);
  return [...lines].map(([section, items]) => [`${section}:`, ...items].join("\n")).join("\n");
}
