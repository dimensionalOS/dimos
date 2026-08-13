import { describe, expect, it, vi } from "vitest";
import type { FrameHeader } from "@dimos/shared";
import { ChannelStore, REBASELINE_WINDOW_MS, StatusStore } from "./store.ts";

function header(seq: number, ts = seq / 10): FrameHeader {
  return { ch: "odom", seq, ts, delivery: "reliable" };
}

describe("ChannelStore", () => {
  it("keeps the latest frame by seq and counts every arrival", () => {
    let now = 1000;
    const store = new ChannelStore(() => now);
    store.ingest("odom", header(5), { x: 5 }, true, '{"x":5}');
    now = 1010;
    store.ingest("odom", header(3), { x: 3 }, true);

    expect(store.get("odom")).toMatchObject({
      value: { x: 5 },
      preview: '{"x":5}',
      seq: 5,
      version: 1,
    });
    store.publishUi();
    expect(store.getUiSnapshot("odom").stats.frames).toBe(2);
  });

  it("notifies direct subscribers per accepted frame, not for stale seqs", () => {
    const store = new ChannelStore(() => 0);
    const cb = vi.fn();
    store.subscribe("odom", cb);
    store.ingest("odom", header(1), 1, true);
    store.ingest("odom", header(2), 2, true);
    store.ingest("odom", header(1), 1, true);
    // A decode failure never touches the slot, so it must not notify either.
    store.ingest("odom", header(3), undefined, false);
    expect(cb).toHaveBeenCalledTimes(2);
  });

  it("notifies UI subscribers only from publishUi", () => {
    const store = new ChannelStore(() => 0);
    const cb = vi.fn();
    store.subscribeUi("odom", cb);
    store.ingest("odom", header(1), 1, true);
    store.ingest("odom", header(2), 2, true);
    expect(cb).not.toHaveBeenCalled();
    store.publishUi();
    expect(cb).toHaveBeenCalledTimes(1);
  });

  it("keeps snapshot identity stable between UI ticks", () => {
    let now = 0;
    const store = new ChannelStore(() => now);
    const before = store.getUiSnapshot("odom");
    expect(store.getUiSnapshot("odom")).toBe(before);

    store.ingest("odom", header(1), 1, true);
    expect(store.getUiSnapshot("odom")).toBe(before);
    store.publishUi();
    const after = store.getUiSnapshot("odom");
    expect(after).not.toBe(before);
    expect(store.getUiSnapshot("odom")).toBe(after);

    // Nothing arrived and the age bucket has not moved: same snapshot object.
    store.publishUi();
    expect(store.getUiSnapshot("odom")).toBe(after);

    // Silence ages the channel: the next tick publishes a fresh snapshot so
    // staleness keeps rising on screen.
    now = 5000;
    store.publishUi();
    expect(store.getUiSnapshot("odom")).not.toBe(after);
  });

  it("computes hz from header timestamps, not arrival times", () => {
    let now = 0;
    const store = new ChannelStore(() => now);
    // The source stamps 10 Hz; delivery drains at 20 Hz (catching up on a
    // backlog). Arrival rate must not leak into the figure.
    for (let i = 1; i <= 40; i++) {
      now = i * 50;
      store.ingest("odom", header(i, i / 10), i, true);
    }
    store.publishUi();
    expect(store.getUiSnapshot("odom").stats.hz).toBe(10);
  });

  it("reads a burst-delivered backlog at its source rate", () => {
    const now = 10_000;
    const store = new ChannelStore(() => now);
    // 30 frames stamped 100 ms apart at the source, all arriving at once.
    for (let i = 1; i <= 30; i++) store.ingest("odom", header(i, i / 10), i, true);
    store.publishUi();
    expect(store.getUiSnapshot("odom").stats.hz).toBe(10);
  });

  it("decays hz to zero and grows age while the channel is silent", () => {
    let now = 1000;
    const store = new ChannelStore(() => now);
    store.ingest("odom", header(1, 1), { x: 1 }, true);
    store.publishUi();
    expect(store.getUiSnapshot("odom").stats.hz).toBe(0.5);
    expect(store.getUiSnapshot("odom").stats.ageMs).toBe(0);

    now = 7000;
    store.publishUi();
    expect(store.getUiSnapshot("odom").stats.hz).toBe(0);
    expect(store.getUiSnapshot("odom").stats.ageMs).toBe(6000);
  });

  it("ages a delayed frame by its header timestamp, not its arrival", () => {
    let now = 1000;
    const store = new ChannelStore(() => now);
    // Live frame delivered instantly: the skew estimate learns 0.
    store.ingest("odom", header(1, 1), { x: 1 }, true);
    // 5 s later a frame stamped 4 s ago drains out of a backlog.
    now = 6000;
    store.ingest("odom", header(2, 2), { x: 2 }, true);
    store.publishUi();
    expect(store.getUiSnapshot("odom").stats.ageMs).toBe(4000);
  });

  it("corrects age and hz for a source clock far ahead of the browser", () => {
    let now = 1000;
    const store = new ChannelStore(() => now);
    store.ingest("odom", header(1, (now + 600_000) / 1000), 1, true);
    now = 1500;
    store.ingest("odom", header(2, (now + 600_000) / 1000), 2, true);
    store.publishUi();
    expect(store.getUiSnapshot("odom").stats.hz).toBe(1);
    expect(store.getUiSnapshot("odom").stats.ageMs).toBe(0);

    now = 3500;
    store.publishUi();
    expect(store.getUiSnapshot("odom").stats.ageMs).toBe(2000);
  });

  it("keeps the slot on the last good frame and flags decode failures", () => {
    const store = new ChannelStore(() => 0);
    store.ingest("odom", header(1), { x: 1 }, true, '{"x":1}');
    store.ingest("odom", header(2), undefined, false);

    // The slot stays internally consistent: value, preview, seq, ts, and
    // version all describe the decoded frame, not the corrupt one.
    expect(store.get("odom")).toMatchObject({
      value: { x: 1 },
      preview: '{"x":1}',
      seq: 1,
      ts: 0.1,
      version: 1,
    });
    store.publishUi();
    expect(store.getUiSnapshot("odom").stats).toMatchObject({
      frames: 2,
      decodeErrors: 1,
      decodeFailing: true,
      lastSeq: 2,
      lastTs: 0.2,
    });

    // Recovery: the next good frame takes the slot and clears the flag.
    store.ingest("odom", header(3), { x: 3 }, true);
    expect(store.get("odom")).toMatchObject({ value: { x: 3 }, seq: 3, version: 2 });
    store.publishUi();
    expect(store.getUiSnapshot("odom").stats.decodeFailing).toBe(false);
  });

  it("tracks received frames separately while nothing has ever decoded", () => {
    const store = new ChannelStore(() => 0);
    store.ingest("odom", header(4), undefined, false);
    expect(store.get("odom")).toBeNull();
    store.publishUi();
    expect(store.getUiSnapshot("odom").stats).toMatchObject({
      frames: 1,
      decodeErrors: 1,
      decodeFailing: true,
      lastSeq: 4,
      lastTs: 0.4,
      ageMs: null,
    });
  });

  it("clamps age at zero when the slot predates a re-learned skew", () => {
    let now = 1000;
    const store = new ChannelStore(() => now);
    // Producer A's clock runs ~1000 s ahead; its frame owns the slot.
    store.ingest("odom", header(100, 1001), { a: 1 }, true);
    store.rebaseline();
    // Producer B (clock in sync with the browser) sends a corrupt frame: the
    // skew re-learns from its header while the slot still holds A's frame.
    now = 1500;
    store.ingest("odom", header(5, 1.5), undefined, false);
    store.publishUi();
    expect(store.getUiSnapshot("odom").stats.ageMs).toBe(0);
    expect(store.getUiSnapshot("odom").stats.decodeFailing).toBe(true);
    expect(store.get("odom")).toMatchObject({ seq: 100 });
  });

  it("prunes rate state on ingest: high-rate flow with publishUi paused", () => {
    let now = 0;
    const store = new ChannelStore(() => now);
    // 1024 Hz (power of two: every float here is exact) for ~50 s of browser
    // time without a single UI tick - background tabs throttle timers while
    // networking callbacks keep firing. The ring is fixed-size, so this must
    // neither grow memory nor defer O(n) pruning to the next publish.
    const stepMs = 1000 / 1024;
    for (let i = 1; i <= 51_199; i++) {
      now = i * stepMs;
      store.ingest("odom", header(i, i / 1024), i, true);
    }
    store.publishUi();
    const stats = store.getUiSnapshot("odom").stats;
    expect(stats.frames).toBe(51_199);
    expect(stats.hz).toBe(1024);
    expect(stats.ageMs).toBe(0);
  });

  it("accepts any seq inside the rebaseline window (robot restarted)", () => {
    let now = 0;
    const store = new ChannelStore(() => now);
    store.ingest("odom", header(500), { x: 500 }, true);
    // Same producer: a lower seq is a reordered stale stream -> dropped.
    store.ingest("odom", header(1), { x: 1 }, true);
    expect(store.get("odom")).toMatchObject({ seq: 500 });

    // Producer changed: a late high-seq frame from the dead one may still
    // drain in, but the restarted counter must win by arrival order.
    store.rebaseline();
    store.ingest("odom", header(501), { x: 501 }, true);
    expect(store.get("odom")).toMatchObject({ seq: 501 });
    store.ingest("odom", header(1), { x: 1 }, true);
    expect(store.get("odom")).toMatchObject({ seq: 1, value: { x: 1 } });

    // Window over: the latest-wins guard is back.
    now = REBASELINE_WINDOW_MS;
    store.ingest("odom", header(0), { x: 0 }, true);
    expect(store.get("odom")).toMatchObject({ seq: 1 });
    store.ingest("odom", header(2), { x: 2 }, true);
    expect(store.get("odom")).toMatchObject({ seq: 2, value: { x: 2 } });
  });

  it("reset drops data and notifies both subscriber kinds", () => {
    const store = new ChannelStore(() => 0);
    const direct = vi.fn();
    const ui = vi.fn();
    store.ingest("odom", header(1), 1, true);
    store.ingest("odom", header(2), undefined, false);
    store.subscribe("odom", direct);
    store.subscribeUi("odom", ui);
    store.reset();
    expect(store.get("odom")).toBeNull();
    expect(store.getUiSnapshot("odom").stats).toMatchObject({
      frames: 0,
      decodeErrors: 0,
      decodeFailing: false,
      lastSeq: -1,
    });
    expect(direct).toHaveBeenCalledTimes(1);
    expect(ui).toHaveBeenCalledTimes(1);
  });

  it("unsubscribe stops notifications", () => {
    const store = new ChannelStore(() => 0);
    const cb = vi.fn();
    const unsub = store.subscribe("odom", cb);
    unsub();
    store.ingest("odom", header(1), 1, true);
    expect(cb).not.toHaveBeenCalled();
  });
});

describe("StatusStore", () => {
  it("shallow-merges updates and notifies", () => {
    const store = new StatusStore();
    const cb = vi.fn();
    store.subscribe(cb);
    const before = store.get();
    store.update({ lastError: "boom" });
    expect(cb).toHaveBeenCalledTimes(1);
    expect(store.get()).not.toBe(before);
    expect(store.get().lastError).toBe("boom");
    expect(store.get().epoch).toBe(0);
    expect(store.get()).toBe(store.get());
  });
});
