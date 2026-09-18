import { describe, expect, it } from "vitest";
import {
  childKey,
  type ChildStats,
  COORDINATOR_KEY,
  cpuColor,
  EMPTY_SNAPSHOT,
  fmtIo,
  fmtMem,
  fmtPct,
  fmtSecs,
  foldSnapshot,
  heat,
  parseStats,
  pushSample,
  WINDOW,
  workerKey,
} from "./stats.ts";

const PROCESS = {
  pid: 1234,
  alive: true,
  cpu_percent: 12.3,
  cpu_time_user: 1.2,
  cpu_time_system: 0.3,
  cpu_time_iowait: 0,
  pss: 47_400_000,
  num_threads: 4,
  num_children: 0,
  num_fds: 32,
  io_read_bytes: 12_582_912,
  io_write_bytes: 4_194_304,
};
const WORKER = {
  ...PROCESS,
  pid: 1235,
  cpu_percent: 34,
  worker_id: 0,
  modules: ["nav", "lidar"],
  dedicated: false,
  children: [{ pid: 1300, name: "ffmpeg", cpu_percent: 5.5 }],
};

describe("parseStats", () => {
  it("accepts the stats.json.v1 shape", () => {
    const frame = parseStats({ coordinator: PROCESS, workers: [WORKER] });
    expect(frame?.coordinator.pss).toBe(47_400_000);
    expect(frame?.workers[0].children[0].name).toBe("ffmpeg");
  });

  it("rejects anything off-contract", () => {
    expect(parseStats(null)).toBeNull();
    expect(parseStats([])).toBeNull();
    expect(parseStats({ coordinator: PROCESS })).toBeNull();
    expect(parseStats({ coordinator: { ...PROCESS, pss: "47 MB" }, workers: [] })).toBeNull();
    expect(parseStats({ coordinator: PROCESS, workers: [{ ...WORKER, modules: [1] }] }))
      .toBeNull();
    expect(parseStats({ coordinator: PROCESS, workers: [{ ...WORKER, children: [{ pid: 1 }] }] }))
      .toBeNull();
  });
});

describe("dtop formatting", () => {
  it("matches the TUI's units", () => {
    expect(fmtPct(12.3)).toBe("12%");
    expect(fmtMem(47_400_000)).toBe("45.2 MB");
    expect(fmtMem(1_073_741_824)).toBe("1.0 GB");
    expect(fmtSecs(1.234)).toBe("1.2s");
    expect(fmtSecs(90)).toBe("1.5m");
    expect(fmtSecs(5400)).toBe("1.5h");
    expect(fmtIo(12_582_912, 4_194_304)).toBe("12/4 MB");
  });

  it("colors load cyan -> yellow -> red on dtop's scale", () => {
    expect(heat(0)).toBe("#00eeee");
    expect(heat(0.5)).toBe("#ffcc00");
    expect(heat(1)).toBe("#ff0000");
    expect(heat(2)).toBe("#ff0000");
    expect(cpuColor(25)).toBe("#80dd77");
  });
});

describe("pushSample", () => {
  it("keeps the newest window, as a new array", () => {
    const a = pushSample([], 1, 3);
    const b = pushSample(a, 2, 3);
    const c = pushSample(b, 3, 3);
    expect(pushSample(c, 4, 3)).toEqual([2, 3, 4]);
    expect(c).toEqual([1, 2, 3]);
  });
});

describe("foldSnapshot", () => {
  const frame = (cpu: number, children: ChildStats[] = []) => ({
    coordinator: { ...PROCESS, cpu_percent: cpu },
    workers: [{ ...WORKER, cpu_percent: cpu * 2, children }],
  });

  it("appends one CPU sample per frame to every row and drops vanished rows", () => {
    const child = { pid: 7, name: "ffmpeg", cpu_percent: 1 };
    const one = foldSnapshot(EMPTY_SNAPSHOT, frame(10, [child]));
    expect(one.cpu.get(childKey(7))).toEqual([1]);
    const two = foldSnapshot(one, frame(20));
    expect(two.frame?.coordinator.cpu_percent).toBe(20);
    expect(two.cpu.get(COORDINATOR_KEY)).toEqual([10, 20]);
    expect(two.cpu.get(workerKey(0))).toEqual([20, 40]);
    expect(two.cpu.has(childKey(7))).toBe(false);
  });

  it("bounds every row to the window", () => {
    let snapshot = EMPTY_SNAPSHOT;
    for (let i = 1; i <= WINDOW + 5; i++) snapshot = foldSnapshot(snapshot, frame(i));
    const samples = snapshot.cpu.get(COORDINATOR_KEY)!;
    expect(samples).toHaveLength(WINDOW);
    expect(samples[0]).toBe(6);
    expect(samples[WINDOW - 1]).toBe(WINDOW + 5);
  });

  it("flags an unreadable payload and keeps the last good frame", () => {
    const good = foldSnapshot(EMPTY_SNAPSHOT, frame(10));
    const bad = foldSnapshot(good, { workers: "nope" });
    expect(bad.unreadable).toBe(true);
    expect(bad.frame).toBe(good.frame);
    expect(bad.cpu).toBe(good.cpu);
    const next = foldSnapshot(bad, frame(30));
    expect(next.unreadable).toBe(false);
    expect(next.cpu.get(COORDINATOR_KEY)).toEqual([10, 30]);
  });
});
