import type { EpochMs } from "../../../../shared/protocol/time.ts";
import type { JobMark } from "../../../../shared/view-models/jobs/timeline.ts";
import { clock } from "../../../../shared/view-models/time.ts";

const RUN_MARKS = { submitted: "✓", ok: "✓", error: "✖", interrupted: "⚠" } as const;

/** The column of `at` on a ruler of `columns` cells over [from, to]; null outside it. */
function columnOf(at: EpochMs, from: EpochMs, to: EpochMs, columns: number): number | null {
  if (at < from || at > to) return null;
  return Math.min(columns - 1, Math.floor(((at - from) / (to - from)) * columns));
}

/**
 * A job's marks on one ruler: `─` for time, `│` at now, a run as its outcome's glyph (✓ ok, ✖ error, ⚠ interrupted),
 * a due time as `○`. A later mark on the same cell wins.
 */
export function laneText(marks: readonly JobMark[], from: EpochMs, now: EpochMs, to: EpochMs, columns: number): string {
  const cells: string[] = Array.from({ length: columns }, () => "─");
  const nowColumn = columnOf(now, from, to, columns);
  if (nowColumn !== null) cells[nowColumn] = "│";
  for (const mark of marks) {
    const column = columnOf(mark.at, from, to, columns);
    if (column === null) continue;
    cells[column] = mark.kind === "run" ? RUN_MARKS[mark.outcome] : "○";
  }
  return cells.join("");
}

/** The first whole local hour at or after `from` that is a multiple of `everyHours`. */
function firstTick(from: EpochMs, everyHours: number): Date {
  const tick = new Date(from);
  tick.setMinutes(0, 0, 0);
  const hour = Math.ceil(tick.getHours() / everyHours) * everyHours;
  tick.setHours(hour);
  if (tick.getTime() < from) tick.setHours(tick.getHours() + everyHours);
  return tick;
}

/** Clock labels every `everyHours` local hours in [from, to], each starting at its column, over `columns` cells. */
export function axisText(from: EpochMs, to: EpochMs, columns: number, everyHours: number): string {
  const cells: string[] = Array.from({ length: columns }, () => " ");
  for (const tick = firstTick(from, everyHours); tick.getTime() <= to; tick.setHours(tick.getHours() + everyHours)) {
    const at = tick.getTime() as EpochMs;
    const column = columnOf(at, from, to, columns);
    if (column === null) continue;
    const label = clock(at);
    if (column + label.length > columns) continue;
    for (let index = 0; index < label.length; index += 1) cells[column + index] = label[index] ?? " ";
  }
  return cells.join("");
}
