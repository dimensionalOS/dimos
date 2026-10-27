import assert from "node:assert/strict";
import type { EpochMs } from "../../../../shared/protocol/time.ts";
import type { JobMark } from "../../../../shared/view-models/jobs/timeline.ts";
import { axisText, laneText } from "./lane.ts";
import { test } from "node:test";

const HOUR = 60 * 60 * 1000;

/** Local midnight, since the axis labels are local clock times. */
const T0 = new Date(2026, 9, 19, 0, 0, 0).getTime() as EpochMs;

const at = (hours: number): EpochMs => (T0 + hours * HOUR) as EpochMs;

test("laneText places runs by outcome, dues and now on a 24-cell ruler of 24 hours", () => {
  const marks: JobMark[] = [
    { kind: "run", at: at(1), outcome: "ok", level: "ok" },
    { kind: "run", at: at(3), outcome: "error", level: "critical" },
    { kind: "run", at: at(5), outcome: "interrupted", level: "warn" },
    { kind: "run", at: at(7), outcome: "submitted", level: "ok" },
    { kind: "due", at: at(14) },
    { kind: "due", at: at(30) },
  ];
  assert.equal(laneText(marks, at(0), at(12), at(24), 24), "─✓─✖─⚠─✓────│─○─────────");
});

test("laneText with nothing in the window is a bare ruler with now", () => {
  assert.equal(laneText([], at(0), at(0), at(24), 6), "│─────");
  assert.equal(laneText([], at(0), at(48), at(24), 6), "──────");
});

test("axisText writes local clock labels at their columns", () => {
  assert.equal(axisText(at(0), at(24), 48, 6).trimEnd(), "00:00       06:00       12:00       18:00");
  assert.equal(axisText(at(2.5), at(14.5), 48, 3).trimEnd(), "  03:00       06:00       09:00       12:00");
});
