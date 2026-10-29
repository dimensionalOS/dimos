import type { Level } from "#dimos/agents/shared/view-models/levels.ts";
import type { MenuRow } from "#dimos/agents/frontends/tui/menu/base.ts";

/** A row with nothing to do on it. */
export function plain(text: string, level: Level = "ok"): MenuRow {
  return { text, level, keys: [], open: null };
}
