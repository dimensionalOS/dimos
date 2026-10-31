import { describe, expect, it } from "vitest";
import { processRows, type StatsAnswer } from "./process_rows.ts";

const stats: StatsAnswer = {
  up: true,
  age: 0.4,
  stale: false,
  coordinator: { pid: 10, cpu_percent: 5, pss: 0, rss: 2048, num_threads: 4, num_fds: 30 },
  workers: [
    {
      worker_id: 0,
      pid: 11,
      alive: true,
      modules: ["nav", "lidar"],
      cpu_percent: 40,
      pss: 4096,
      rss: 9999,
      io_read_bytes: 100,
      io_write_bytes: 50,
      children: [{ pid: 12, name: "ffmpeg", cpu_percent: 3 }],
    },
    { worker_id: 1, pid: 13, alive: false, dedicated: true, modules: ["vision"], cpu_percent: 0 },
  ],
  cpuHistory: { coordinator: [1, 5], "worker:0": [30, 40], "child:12": [3] },
};

describe("process rows", () => {
  it("lists the coordinator, then each worker followed by its children", () => {
    const rows = processRows(stats);
    expect(rows.map((row) => row.label)).toEqual([
      "coordinator",
      "worker 0 · nav, lidar",
      "↳ ffmpeg",
      "worker 1 · vision",
    ]);
    expect(rows.map((row) => row.history)).toEqual([[1, 5], [30, 40], [3], []]);
    expect(rows[2].child).toBe(true);
    expect(rows[3].dead).toBe(true);
    expect(rows[3].title).toContain("dedicated, dead");
  });

  it("shows PSS where the OS reports it, else RSS, and io as read plus written", () => {
    const [coordinator, worker] = processRows(stats);
    expect(coordinator.memory).toBe(2048);
    expect(worker.memory).toBe(4096);
    expect(worker.io).toBe(150);
    expect(coordinator.io).toBe(null);
  });
});
