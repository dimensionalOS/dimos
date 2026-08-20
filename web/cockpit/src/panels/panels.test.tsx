// @vitest-environment happy-dom
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { act } from "react";
import { createRoot, type Root } from "react-dom/client";
import type { FrameHeader, PanelSpec } from "@dimos/shared";
import { ChannelStore } from "../session/store.ts";
import { PanelGrid } from "./PanelGrid.tsx";
import { getPanel } from "./registry.ts";
import { type DrawHealth, startVideoSink, VideoPanel } from "./VideoPanel.tsx";

(globalThis as { IS_REACT_ACT_ENVIRONMENT?: boolean }).IS_REACT_ACT_ENVIRONMENT = true;

const CH = "color_image";

function header(seq: number, ts = seq): FrameHeader {
  return { ch: CH, seq, ts, delivery: "latest" };
}

function bitmap(w = 4, h = 3): ImageBitmap {
  return { width: w, height: h, close: vi.fn() } as unknown as ImageBitmap;
}

function frame(store: ChannelStore, seq: number, ts = seq): Uint8Array {
  const payload = new Uint8Array([seq]);
  store.ingest(CH, header(seq, ts), payload, true);
  return payload;
}

/** Decode stub whose promises settle only when the test says so. */
function deferredDecode() {
  const calls: Uint8Array[] = [];
  const settlers: { resolve: (b: ImageBitmap) => void; reject: (e: Error) => void }[] = [];
  const decode = (payload: Uint8Array): Promise<ImageBitmap> => {
    calls.push(payload);
    return new Promise((resolve, reject) => settlers.push({ resolve, reject }));
  };
  return { decode, calls, settlers };
}

const flush = () => new Promise((resolve) => setTimeout(resolve, 0));

describe("startVideoSink", () => {
  let store: ChannelStore;
  let canvas: HTMLCanvasElement;
  let ctx: { drawImage: ReturnType<typeof vi.fn> };
  let health: DrawHealth;
  let stop: (() => void) | null;

  beforeEach(() => {
    store = new ChannelStore();
    canvas = document.createElement("canvas");
    // The sink grabs the 2D context at start; happy-dom has no real one.
    ctx = { drawImage: vi.fn() };
    vi.spyOn(HTMLCanvasElement.prototype, "getContext").mockReturnValue(
      ctx as unknown as CanvasRenderingContext2D,
    );
    health = { lastDrawOkAtMs: 0, failures: 0 };
    stop = null;
  });

  afterEach(() => {
    stop?.();
    vi.restoreAllMocks();
  });

  it("decodes one frame at a time and skips straight to the newest", async () => {
    const { decode, calls, settlers } = deferredDecode();
    stop = startVideoSink(store, CH, canvas, health, { decode, hidden: () => false });

    const first = frame(store, 1);
    expect(calls).toEqual([first]);
    frame(store, 2);
    frame(store, 3);
    const newest = frame(store, 4);
    expect(calls.length).toBe(1); // one decode in flight, burst sheds

    const bmp = bitmap(8, 6);
    settlers[0].resolve(bmp);
    await flush();
    expect(canvas.width).toBe(8); // first frame drew and resized the canvas
    expect(ctx.drawImage).toHaveBeenCalledWith(bmp, 0, 0); // and actually painted
    expect(calls.length).toBe(2);
    expect(calls[1]).toBe(newest); // frames 2 and 3 were never decoded

    settlers[1].resolve(bitmap(8, 6));
    await flush();
    expect(calls.length).toBe(2); // caught up, nothing left to decode
  });

  it("draws the decoded bitmap and always releases it", async () => {
    const { decode, settlers } = deferredDecode();
    const nowSpy = vi.spyOn(Date, "now").mockReturnValue(1000);
    stop = startVideoSink(store, CH, canvas, health, { decode, hidden: () => false });
    expect(health.lastDrawOkAtMs).toBe(1000); // stamped at start, never "stalled" fresh

    nowSpy.mockReturnValue(2500);
    frame(store, 1);
    const bmp = bitmap();
    settlers[0].resolve(bmp);
    await flush();
    expect(ctx.drawImage).toHaveBeenCalledWith(bmp, 0, 0);
    expect(bmp.close).toHaveBeenCalledTimes(1);
    expect(health.failures).toBe(0);
    expect(health.lastDrawOkAtMs).toBe(2500); // advanced by the draw
  });

  it("releases the bitmap and counts a draw failure when drawImage throws", async () => {
    const { decode, calls, settlers } = deferredDecode();
    stop = startVideoSink(store, CH, canvas, health, { decode, hidden: () => false });
    ctx.drawImage.mockImplementationOnce(() => {
      throw new Error("canvas lost");
    });

    frame(store, 1);
    const bmp = bitmap();
    settlers[0].resolve(bmp);
    await flush();
    expect(bmp.close).toHaveBeenCalledTimes(1); // the finally still released it
    expect(health.failures).toBe(1);
    expect(calls.length).toBe(1); // the bad frame is not retried

    frame(store, 2);
    settlers[1].resolve(bitmap());
    await flush();
    expect(health.failures).toBe(0); // the next frame recovers
  });

  it("counts decode rejections without touching lastDrawOkAtMs", async () => {
    const { decode, settlers } = deferredDecode();
    stop = startVideoSink(store, CH, canvas, health, { decode, hidden: () => false });
    const stamp = health.lastDrawOkAtMs;

    frame(store, 1);
    settlers[0].reject(new Error("not a jpeg"));
    await flush();
    expect(health.failures).toBe(1);
    expect(health.lastDrawOkAtMs).toBe(stamp); // only successes stamp it

    frame(store, 2);
    settlers[1].resolve(bitmap());
    await flush();
    expect(health.failures).toBe(0);
  });

  it("skips an undecodable frame without spinning on it", async () => {
    const { decode, calls, settlers } = deferredDecode();
    stop = startVideoSink(store, CH, canvas, health, { decode, hidden: () => false });

    frame(store, 1);
    settlers[0].reject(new Error("not a jpeg"));
    await flush();
    expect(calls.length).toBe(1); // no retry of the same slot

    frame(store, 2);
    expect(calls.length).toBe(2); // the next frame decodes normally
  });

  it("does not decode while hidden and catches up on visibilitychange", async () => {
    const { decode, calls, settlers } = deferredDecode();
    let hidden = true;
    stop = startVideoSink(store, CH, canvas, health, { decode, hidden: () => hidden });

    frame(store, 1);
    const newest = frame(store, 2);
    expect(calls.length).toBe(0); // a backgrounded panel costs no decode

    hidden = false;
    document.dispatchEvent(new Event("visibilitychange"));
    expect(calls.length).toBe(1);
    expect(calls[0]).toBe(newest);
    settlers[0].resolve(bitmap());
    await flush();
  });

  it("stops decoding and drawing after cleanup", async () => {
    const { decode, calls, settlers } = deferredDecode();
    stop = startVideoSink(store, CH, canvas, health, { decode, hidden: () => false });
    frame(store, 1);
    stop();
    stop = null;

    settlers[0].resolve(bitmap(9, 9));
    await flush();
    expect(canvas.width).not.toBe(9); // in-flight decode must not touch the canvas

    frame(store, 2);
    expect(calls.length).toBe(1);
  });
});

describe("VideoPanel", () => {
  const SPEC: PanelSpec = { id: "cam", kind: "video", channels: [CH] };
  let container: HTMLElement;
  let root: Root;
  let now: number;
  let store: ChannelStore;
  const badge = () => container.querySelector(`[data-testid="video-${CH}-badge"]`)!;

  beforeEach(() => {
    vi.stubGlobal("createImageBitmap", () => Promise.resolve(bitmap()));
    container = document.createElement("div");
    document.body.appendChild(container);
    root = createRoot(container);
    now = 1_000_000;
    store = new ChannelStore(() => now);
  });

  afterEach(() => {
    act(() => root.unmount());
    container.remove();
    vi.unstubAllGlobals();
    vi.restoreAllMocks();
  });

  it("shows waiting, then the fps badge, then flags staleness", () => {
    act(() => root.render(<VideoPanel spec={SPEC} store={store} />));
    expect(container.textContent).toContain("waiting for data");
    expect(badge().textContent).toBe("waiting");
    expect(badge().getAttribute("role")).toBe("status");
    const canvas = container.querySelector("canvas")!;
    expect(canvas.getAttribute("role")).toBe("img");
    expect(canvas.getAttribute("aria-label")).toBe("cam");

    // Frames at 10 Hz of source time, arriving with zero skew.
    act(() => {
      for (let i = 0; i < 10; i++) frame(store, i, now / 1000 - (9 - i) / 10);
      store.publishUi();
    });
    expect(container.textContent).not.toContain("waiting for data");
    expect(badge().textContent).toMatch(/fps$/);
    expect(badge().getAttribute("data-stale")).toBeNull();

    // Silence: source age climbs past the threshold on a later UI tick.
    act(() => {
      now += 5000;
      store.publishUi();
    });
    expect(badge().textContent).toMatch(/^stale/);
    expect(badge().getAttribute("data-stale")).toBe("true");
  });

  it("flags decode failures in the badge and recovers", async () => {
    act(() => root.render(<VideoPanel spec={SPEC} store={store} />));
    await act(async () => {
      frame(store, 1, now / 1000);
      await flush();
      store.publishUi();
    });
    expect(badge().textContent).toMatch(/fps$/);

    // The jpeg.v1 decoder rejected a frame at ingest: nothing reaches the
    // slot, but the store counts it.
    act(() => {
      store.ingest(CH, header(2, now / 1000), undefined, false);
      store.publishUi();
    });
    expect(badge().textContent).toBe("decode failing");
    expect(badge().getAttribute("data-error")).toBe("true");

    await act(async () => {
      frame(store, 3, now / 1000);
      await flush();
      store.publishUi();
    });
    expect(badge().textContent).toMatch(/fps$/);
    expect(badge().getAttribute("data-error")).toBeNull();
  });

  it("shows stalled when frames arrive but nothing draws", () => {
    // A decoder that never settles: frames keep arriving, nothing paints.
    vi.stubGlobal("createImageBitmap", () => new Promise<ImageBitmap>(() => {}));
    // The sink stamps health with Date.now; align it with the store's clock.
    vi.spyOn(Date, "now").mockImplementation(() => now);
    act(() => root.render(<VideoPanel spec={SPEC} store={store} />));
    act(() => {
      frame(store, 1, now / 1000);
      store.publishUi();
    });
    expect(badge().getAttribute("data-stale")).toBeNull();

    now += 3000;
    act(() => {
      frame(store, 2, now / 1000);
      store.publishUi();
    });
    expect(badge().textContent).toBe("stalled");
    expect(badge().textContent).not.toMatch(/^stale/);
    expect(badge().getAttribute("data-stale")).toBe("true");
  });

  it("surfaces a createImageBitmap rejection as decode failing", async () => {
    vi.stubGlobal("createImageBitmap", () => Promise.reject(new Error("codec")));
    act(() => root.render(<VideoPanel spec={SPEC} store={store} />));
    await act(async () => {
      frame(store, 1, now / 1000);
      await flush();
      store.publishUi();
    });
    expect(badge().textContent).toBe("decode failing");
    expect(badge().getAttribute("data-error")).toBe("true");
  });

  it("renders a visible note instead of a canvas when no channel is bound", () => {
    act(() =>
      root.render(
        <VideoPanel spec={{ id: "cam", kind: "video", channels: [] }} store={store} />,
      )
    );
    expect(container.textContent).toContain("no channel bound");
    expect(container.querySelector("canvas")).toBeNull();
  });
});

describe("PanelGrid", () => {
  let container: HTMLElement;
  let root: Root;
  let store: ChannelStore;

  beforeEach(() => {
    vi.stubGlobal("createImageBitmap", () => Promise.resolve(bitmap()));
    container = document.createElement("div");
    document.body.appendChild(container);
    root = createRoot(container);
    store = new ChannelStore();
  });

  afterEach(() => {
    act(() => root.unmount());
    container.remove();
    vi.unstubAllGlobals();
  });

  it("renders known panel kinds in manifest order and skips unknown ones", () => {
    const panels: PanelSpec[] = [
      { id: "cam", kind: "video", channels: [CH] },
      { id: "mystery", kind: "hologram", channels: [] },
    ];
    act(() => root.render(<PanelGrid panels={panels} store={store} />));
    expect(container.querySelector('[data-testid="panel-cam"]')).not.toBeNull();
    expect(container.textContent).not.toContain("mystery");

    // No known panels: the grid contributes nothing (ChannelList still shows
    // the channels).
    act(() => root.render(<PanelGrid panels={[panels[1]]} store={store} />));
    expect(container.innerHTML).toBe("");
  });

  it("has the video panel registered", () => {
    expect(getPanel("video")).toBe(VideoPanel);
    expect(getPanel("hologram")).toBeUndefined();
  });
});
