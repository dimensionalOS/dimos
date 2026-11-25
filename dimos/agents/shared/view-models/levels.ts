/** How much a datum needs a person. Renderers map it to a colour or a CSS class; shared/view-models/ never names one. */
export type Level = "ok" | "warn" | "critical";

const RANK: { readonly [L in Level]: number } = { ok: 0, warn: 1, critical: 2 };

export function worst(levels: readonly Level[]): Level {
  let top: Level = "ok";
  for (const level of levels) if (RANK[level] > RANK[top]) top = level;
  return top;
}

/** A segment of a status line: text plus how much it needs a person. */
export type Segment = { readonly text: string; readonly level: Level };
