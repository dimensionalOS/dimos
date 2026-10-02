// @vitest-environment happy-dom
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { act } from "react";
import { createRoot, type Root } from "react-dom/client";
import type { PanelSpec } from "@dimos/shared";
import { ChannelStore } from "@dimos/sdk";
import { StatsPanel } from "./StatsPanel.tsx";

(globalThis as { IS_REACT_ACT_ENVIRONMENT?: boolean }).IS_REACT_ACT_ENVIRONMENT = true;

const CH = "resource_stats";
const SPEC: PanelSpec = { id: "p1", kind: "stats", title: "Stats", channels: [CH], params: {} };
// dtop's preview data, as the bridge encodes it.
const FRAME = {
  coordinator: {
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
  },
  workers: [
    {
      pid: 1235,
      alive: true,
      cpu_percent: 34,
      cpu_time_user: 5.1,
      cpu_time_system: 1,
      cpu_time_iowait: 0.2,
      pss: 125_829_120,
      num_threads: 8,
      num_children: 2,
      num_fds: 64,
      io_read_bytes: 47_185_920,
      io_write_bytes: 12_582_912,
      worker_id: 0,
      modules: ["nav", "lidar"],
      dedicated: false,
      children: [{ pid: 1300, name: "ffmpeg", cpu_percent: 5.5 }],
    },
    {
      pid: 0,
      alive: false,
      cpu_percent: 0,
      cpu_time_user: 0,
      cpu_time_system: 0,
      cpu_time_iowait: 0,
      pss: 0,
      num_threads: 0,
      num_children: 0,
      num_fds: 0,
      io_read_bytes: 0,
      io_write_bytes: 0,
      worker_id: 1,
      modules: ["vision"],
      dedicated: true,
      children: [],
    },
  ],
};

describe("StatsPanel", () => {
  let container: HTMLElement;
  let root: Root;
  let now: number;
  let store: ChannelStore;
  let ctx: Record<string, ReturnType<typeof vi.fn>>;
  const testId = (id: string) => container.querySelector(`[data-testid="${id}"]`)!;
  const cells = (row: Element) => [...row.querySelectorAll("td")].map((td) => td.textContent);

  beforeEach(() => {
    container = document.createElement("div");
    document.body.appendChild(container);
    root = createRoot(container);
    now = 1_000_000;
    store = new ChannelStore(() => now);
    // The sparklines grab a 2D context; happy-dom has no real one.
    ctx = {
      clearRect: vi.fn(),
      beginPath: vi.fn(),
      moveTo: vi.fn(),
      lineTo: vi.fn(),
      stroke: vi.fn(),
      closePath: vi.fn(),
      fill: vi.fn(),
      fillRect: vi.fn(),
    };
    vi.spyOn(HTMLCanvasElement.prototype, "getContext").mockReturnValue(
      ctx as unknown as CanvasRenderingContext2D,
    );
  });

  afterEach(() => {
    act(() => root.unmount());
    container.remove();
    vi.restoreAllMocks();
  });

  function frame(seq: number, value: unknown = FRAME): void {
    act(() => {
      store.ingest(CH, { ch: CH, seq, ts: now / 1000, delivery: "latest" }, value, true);
      store.publishUi();
    });
  }

  it("waits, then lists every process with dtop's units and a sparkline each", () => {
    act(() => root.render(<StatsPanel spec={SPEC} store={store} />));
    expect(container.textContent).toContain("waiting for resource stats");
    expect(testId(`stats-${CH}-badge`).textContent).toBe("waiting");

    frame(1);
    expect(container.textContent).not.toContain("waiting for resource stats");
    const coordinator = testId(`stats-${CH}-row-coordinator`);
    expect(coordinator.textContent).toContain("coordinator");
    expect(coordinator.textContent).toContain("[1234]");
    expect(cells(coordinator).slice(3)).toEqual([
      "12%",
      "45.2 MB",
      "4",
      "32",
      "0",
      "1.2s",
      "0.3s",
      "0.0s",
      "12/4 MB",
    ]);
    const worker = testId(`stats-${CH}-row-worker-0`);
    expect(worker.getAttribute("data-dead")).toBeNull();
    expect(worker.textContent).toContain("worker 0");
    expect(worker.textContent).toContain("nav, lidar");
    expect(cells(worker)[3]).toBe("34%");
    expect(cells(worker)[4]).toBe("120.0 MB");
    const child = testId(`stats-${CH}-row-child-1300`);
    expect(child.textContent).toContain("ffmpeg");
    expect(child.textContent).toContain("[1300]");
    expect(cells(child).slice(3)).toEqual(["6%", "", "", "", "", "", "", "", ""]);
    // A single sample draws a dot; the second frame draws the line.
    expect(ctx.fillRect).toHaveBeenCalled();
    expect(ctx.stroke).not.toHaveBeenCalled();
    frame(2);
    expect(ctx.stroke).toHaveBeenCalled();
    expect(testId(`stats-${CH}-badge`).textContent).toMatch(/Hz$/);
  });

  it("marks a dead worker and shows dashes for its zeros", () => {
    act(() => root.render(<StatsPanel spec={SPEC} store={store} />));
    frame(1);
    const dead = testId(`stats-${CH}-row-worker-1`);
    expect(dead.getAttribute("data-dead")).toBe("true");
    expect(dead.textContent).toContain("worker 1");
    expect(dead.textContent).toContain("dedicated");
    expect(dead.textContent).toContain("dead");
    expect(dead.textContent).not.toContain("[0]");
    expect(cells(dead).slice(2)).toEqual(["", "-", "-", "-", "-", "-", "-", "-", "-", "-"]);
  });

  it("dims the table and flags the badge once snapshots stop", () => {
    act(() => root.render(<StatsPanel spec={SPEC} store={store} />));
    frame(1);
    expect(testId(`stats-${CH}`).getAttribute("data-stale")).toBeNull();
    act(() => {
      now += 5000;
      store.publishUi();
    });
    expect(testId(`stats-${CH}`).getAttribute("data-stale")).toBe("true");
    expect(testId(`stats-${CH}-badge`).textContent).toMatch(/^stale/);
    // The rows stay: the last snapshot is still the best information.
    expect(testId(`stats-${CH}-row-coordinator`).textContent).toContain("[1234]");
  });

  it("reports an unreadable frame without dropping the last good one", () => {
    act(() => root.render(<StatsPanel spec={SPEC} store={store} />));
    frame(1);
    frame(2, { workers: "nope" });
    expect(container.querySelector("[role=alert]")!.textContent).toBe("unreadable stats frame");
    expect(testId(`stats-${CH}-row-coordinator`).textContent).toContain("[1234]");
  });

  it("forgets everything on a store reset", () => {
    act(() => root.render(<StatsPanel spec={SPEC} store={store} />));
    frame(1);
    expect(testId(`stats-${CH}-table`)).not.toBeNull();
    act(() => store.reset());
    expect(container.textContent).toContain("waiting for resource stats");
    // The window restarts from the next frame: a single-sample dot per row,
    // never a line joining it to the pre-reset sample.
    frame(1);
    expect(testId(`stats-${CH}-table`)).not.toBeNull();
    expect(ctx.stroke).not.toHaveBeenCalled();
  });

  it("degrades to a hint when the manifest binds no stats channel", () => {
    act(() => root.render(<StatsPanel spec={{ ...SPEC, channels: [] }} store={store} />));
    expect(container.textContent).toContain("stats panel p1: no stats channel bound");
  });
});
