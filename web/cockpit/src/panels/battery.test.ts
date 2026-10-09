import { describe, expect, it } from "vitest";
import { EMPTY_HISTORY, estimateRemainingS, fmtDuration, foldBattery } from "./battery.ts";

describe("battery history", () => {
  it("folds frames into run-relative samples and keeps the latest frame", () => {
    let h = foldBattery(EMPTY_HISTORY, { ts: 100, percentage: 0.8, voltage: 28, current: -2 });
    h = foldBattery(h, { ts: 101, percentage: 0.79 });
    expect(h.startTs).toBe(100);
    expect(h.samples).toEqual([{ t: 0, pct: 80 }, { t: 1, pct: 79 }]);
    expect(h.frame?.voltage).toBeNull();
    expect(foldBattery(h, "junk").unreadable).toBe(true);
  });

  it("estimates time left from a steady drain, none while flat or too early", () => {
    // 1 %/min from 80 %: 80 min left.
    const drain = Array.from({ length: 301 }, (_, i) => ({ t: i, pct: 80 - i / 60 }));
    expect(estimateRemainingS(drain)).toBeCloseTo(75 * 60, -1);
    const flat = Array.from({ length: 301 }, (_, i) => ({ t: i, pct: 80 }));
    expect(estimateRemainingS(flat)).toBeNull();
    expect(estimateRemainingS(drain.slice(0, 30))).toBeNull();
    const charge = Array.from({ length: 301 }, (_, i) => ({ t: i, pct: 50 + i / 60 }));
    expect(estimateRemainingS(charge)).toBeNull();
  });

  it("formats durations", () => {
    expect(fmtDuration(4500)).toBe("1 h 15 min");
    expect(fmtDuration(90)).toBe("1 min");
    expect(fmtDuration(42)).toBe("42 s");
  });
});
