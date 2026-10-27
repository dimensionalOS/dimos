import type { Level } from "../../../../shared/view-models/levels.ts";
import type { MenuRow } from "../base.ts";

/** A row with nothing to do on it. */
export function plain(text: string, level: Level = "ok"): MenuRow {
  return { text, level, keys: [], open: null };
}
