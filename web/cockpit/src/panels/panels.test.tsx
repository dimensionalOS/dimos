// @vitest-environment happy-dom
import { afterEach, beforeAll, beforeEach, describe, expect, it, vi } from "vitest";
import { act } from "react";
import { createRoot, type Root } from "react-dom/client";
import type { FrameHeader, PanelSpec } from "@dimos/shared";
import type { CostmapValue } from "@dimos/sdk";
import { ChannelStore } from "@dimos/sdk";
import type { DrawHealth } from "../layout/PanelFrame.tsx";
import { MapPanel, startMapSink } from "./MapPanel.tsx";
import { fitTransform, posePath } from "./mapRenderer.ts";
import { getPanel, UnknownPanel } from "./registry.tsx";
import { startVideoSink, VideoPanel } from "./VideoPanel.tsx";

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
  const SPEC: PanelSpec = { id: "cam", kind: "video", title: "", channels: [CH], params: {} };
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
        <VideoPanel
          spec={{ id: "cam", kind: "video", title: "", channels: [], params: {} }}
          store={store}
        />,
      )
    );
    expect(container.textContent).toContain("no channel bound");
    expect(container.querySelector("canvas")).toBeNull();
  });
});

describe("registry", () => {
  let container: HTMLElement;
  let root: Root;
  let store: ChannelStore;

  beforeEach(() => {
    container = document.createElement("div");
    document.body.appendChild(container);
    root = createRoot(container);
    store = new ChannelStore();
  });

  afterEach(() => {
    act(() => root.unmount());
    container.remove();
  });

  it("has the video and map2d panels registered; unknown kinds stay undefined", () => {
    // The subscription gate depends on getPanel returning undefined here: an
    // UnknownPanel fallback in the registry itself would subscribe every
    // channel of a newer bridge (see channelSubscribable in session.ts).
    expect(getPanel("video")).toBe(VideoPanel);
    expect(getPanel("map2d")).toBe(MapPanel);
    expect(getPanel("hologram")).toBeUndefined();
  });

  it("UnknownPanel renders visible chrome and binds nothing", () => {
    const spec: PanelSpec = {
      id: "mystery",
      kind: "hologram",
      title: "",
      channels: [],
      params: {},
    };
    act(() => root.render(<UnknownPanel spec={spec} store={store} />));
    expect(container.querySelector('[data-testid="panel-mystery"]')).not.toBeNull();
    expect(container.textContent).toContain("unknown panel kind hologram");
    expect(container.querySelector("canvas")).toBeNull();
  });
});

// ---------------------------------------------------------------------------
// Map panel

const MAP_CH = "global_costmap";
const POSE_CH = "odom";

function costmapValue(seq: number, w = 2, h = 2): CostmapValue {
  return { bytes: new Uint8Array([seq]), w, h, res: 0.5, origin: [0.25, -0.5, 0.0] };
}

function gridFrame(store: ChannelStore, seq: number, ts = seq): CostmapValue {
  const value = costmapValue(seq);
  store.ingest(MAP_CH, { ch: MAP_CH, seq, ts, delivery: "latest" }, value, true);
  return value;
}

function poseFrame(store: ChannelStore, seq: number): void {
  const value = { x: 0.5, y: 0.5, z: 0.1, yaw: 0.25, ts: seq };
  store.ingest(POSE_CH, { ch: POSE_CH, seq, ts: seq, delivery: "reliable" }, value, true);
}

/** Inflate stub whose promises settle only when the test says so. */
function deferredInflate() {
  const calls: CostmapValue[] = [];
  const settlers: { resolve: (cells: Uint8Array) => void; reject: (e: Error) => void }[] = [];
  const inflate = (value: CostmapValue): Promise<Uint8Array> => {
    calls.push(value);
    return new Promise((resolve, reject) => settlers.push({ resolve, reject }));
  };
  return { inflate, calls, settlers };
}

/** happy-dom has no layout; pin the CSS size the sink reads. */
function defineSize(canvas: HTMLCanvasElement, w: number, h: number): void {
  Object.defineProperty(canvas, "clientWidth", { configurable: true, value: w });
  Object.defineProperty(canvas, "clientHeight", { configurable: true, value: h });
}

describe("startMapSink", () => {
  interface FakeCtx {
    drawImage: ReturnType<typeof vi.fn>;
    putImageData: ReturnType<typeof vi.fn>;
    clearRect: ReturnType<typeof vi.fn>;
    save: ReturnType<typeof vi.fn>;
    restore: ReturnType<typeof vi.fn>;
    translate: ReturnType<typeof vi.fn>;
    rotate: ReturnType<typeof vi.fn>;
    beginPath: ReturnType<typeof vi.fn>;
    moveTo: ReturnType<typeof vi.fn>;
    lineTo: ReturnType<typeof vi.fn>;
    closePath: ReturnType<typeof vi.fn>;
    fill: ReturnType<typeof vi.fn>;
  }
  let store: ChannelStore;
  let canvas: HTMLCanvasElement;
  let contexts: FakeCtx[];
  let health: DrawHealth;
  let stop: (() => void) | null;
  // getContext creation order in startMapSink: display canvas, then backing.
  const display = () => contexts[0];
  const backing = () => contexts[1];

  beforeEach(() => {
    store = new ChannelStore();
    canvas = document.createElement("canvas");
    defineSize(canvas, 100, 80);
    contexts = [];
    vi.spyOn(HTMLCanvasElement.prototype, "getContext").mockImplementation(() => {
      const fake: FakeCtx = {
        drawImage: vi.fn(),
        putImageData: vi.fn(),
        clearRect: vi.fn(),
        save: vi.fn(),
        restore: vi.fn(),
        translate: vi.fn(),
        rotate: vi.fn(),
        beginPath: vi.fn(),
        moveTo: vi.fn(),
        lineTo: vi.fn(),
        closePath: vi.fn(),
        fill: vi.fn(),
      };
      contexts.push(fake);
      return fake as unknown as CanvasRenderingContext2D;
    });
    health = { lastDrawOkAtMs: 0, failures: 0 };
    stop = null;
  });

  afterEach(() => {
    stop?.();
    vi.restoreAllMocks();
    vi.unstubAllGlobals();
  });

  it("inflates one grid at a time and skips straight to the newest", async () => {
    const { inflate, calls, settlers } = deferredInflate();
    stop = startMapSink(store, MAP_CH, POSE_CH, canvas, health, { inflate, hidden: () => false });

    const first = gridFrame(store, 1);
    expect(calls).toEqual([first]);
    gridFrame(store, 2);
    const newest = gridFrame(store, 3);
    expect(calls.length).toBe(1); // one inflate in flight, burst sheds

    settlers[0].resolve(new Uint8Array(4));
    await flush();
    expect(backing().putImageData).toHaveBeenCalledTimes(1);
    const img = backing().putImageData.mock.calls[0][0] as ImageData;
    expect([img.width, img.height]).toEqual([2, 2]);
    expect(display().drawImage).toHaveBeenCalledTimes(1);
    expect(canvas.width).toBe(100); // sized from the layout, not the grid
    expect(calls.length).toBe(2);
    expect(calls[1]).toBe(newest); // frame 2 was never inflated

    settlers[1].resolve(new Uint8Array(4));
    await flush();
    expect(calls.length).toBe(2); // caught up
  });

  it("redraws the pose from the cached bitmap without a new inflate", async () => {
    const { inflate, calls, settlers } = deferredInflate();
    stop = startMapSink(store, MAP_CH, POSE_CH, canvas, health, { inflate, hidden: () => false });
    gridFrame(store, 1);
    settlers[0].resolve(new Uint8Array(4));
    await flush();
    expect(display().fill).not.toHaveBeenCalled(); // no pose yet, no triangle

    poseFrame(store, 1);
    expect(display().drawImage).toHaveBeenCalledTimes(2);
    expect(display().fill).toHaveBeenCalledTimes(1); // the triangle
    expect(backing().putImageData).toHaveBeenCalledTimes(1); // bitmap reused
    expect(calls.length).toBe(1);
  });

  it("ignores pose frames until a grid has drawn", () => {
    const { inflate } = deferredInflate();
    stop = startMapSink(store, MAP_CH, POSE_CH, canvas, health, { inflate, hidden: () => false });
    poseFrame(store, 1);
    expect(display().drawImage).not.toHaveBeenCalled();
  });

  it("counts inflate rejections and recovers on the next grid", async () => {
    const { inflate, settlers } = deferredInflate();
    stop = startMapSink(store, MAP_CH, POSE_CH, canvas, health, { inflate, hidden: () => false });
    const stamp = health.lastDrawOkAtMs;
    gridFrame(store, 1);
    settlers[0].reject(new Error("corrupt zlib"));
    await flush();
    expect(health.failures).toBe(1);
    expect(health.lastDrawOkAtMs).toBe(stamp); // only successes stamp it

    gridFrame(store, 2);
    settlers[1].resolve(new Uint8Array(4));
    await flush();
    expect(health.failures).toBe(0);
  });

  it("skips a slot that is not a costmap value without spinning", () => {
    const { inflate, calls } = deferredInflate();
    stop = startMapSink(store, MAP_CH, POSE_CH, canvas, health, { inflate, hidden: () => false });
    store.ingest(
      MAP_CH,
      { ch: MAP_CH, seq: 1, ts: 1, delivery: "latest" },
      new Uint8Array(3),
      true,
    );
    expect(calls.length).toBe(0);
  });

  it("does not inflate while hidden and catches up on visibilitychange", async () => {
    const { inflate, calls, settlers } = deferredInflate();
    let hidden = true;
    stop = startMapSink(store, MAP_CH, POSE_CH, canvas, health, { inflate, hidden: () => hidden });
    gridFrame(store, 1);
    const newest = gridFrame(store, 2);
    expect(calls.length).toBe(0); // a backgrounded panel costs no inflate

    hidden = false;
    document.dispatchEvent(new Event("visibilitychange"));
    expect(calls.length).toBe(1);
    expect(calls[0]).toBe(newest);
    settlers[0].resolve(new Uint8Array(4));
    await flush();
  });

  it("redraws on resize with the cached bitmap and disposes the observer", async () => {
    const { inflate, calls, settlers } = deferredInflate();
    let resize: (() => void) | null = null;
    const dispose = vi.fn();
    stop = startMapSink(store, MAP_CH, POSE_CH, canvas, health, {
      inflate,
      hidden: () => false,
      observeResize: (_el, cb) => {
        resize = cb;
        return dispose;
      },
    });
    gridFrame(store, 1);
    settlers[0].resolve(new Uint8Array(4));
    await flush();
    expect(canvas.width).toBe(100);

    defineSize(canvas, 250, 80);
    resize!();
    expect(canvas.width).toBe(250); // backing store follows the layout size
    expect(display().drawImage).toHaveBeenCalledTimes(2);
    expect(calls.length).toBe(1); // no re-inflate on resize

    stop!();
    stop = null;
    expect(dispose).toHaveBeenCalledTimes(1);
  });

  it("stops inflating and drawing after cleanup", async () => {
    const { inflate, calls, settlers } = deferredInflate();
    stop = startMapSink(store, MAP_CH, POSE_CH, canvas, health, { inflate, hidden: () => false });
    gridFrame(store, 1);
    stop();
    stop = null;

    settlers[0].resolve(new Uint8Array(4));
    await flush();
    expect(backing().putImageData).not.toHaveBeenCalled(); // in-flight inflate must not paint

    gridFrame(store, 2);
    poseFrame(store, 1);
    expect(calls.length).toBe(1);
    expect(display().drawImage).not.toHaveBeenCalled();
  });

  it("rotates the grid blit by -yaw and restores before the pose", async () => {
    const { inflate, settlers } = deferredInflate();
    stop = startMapSink(store, MAP_CH, POSE_CH, canvas, health, { inflate, hidden: () => false });
    const value = { ...costmapValue(1), origin: [0.25, -0.5, 0.25] as [number, number, number] };
    store.ingest(MAP_CH, { ch: MAP_CH, seq: 1, ts: 1, delivery: "latest" }, value, true);
    settlers[0].resolve(new Uint8Array(4));
    await flush();
    poseFrame(store, 1);

    expect(display().rotate.mock.calls[0][0]).toBeCloseTo(-0.25, 9);
    const [, dx, dy, dw, dh] = display().drawImage.mock.calls[0];
    expect(dx).toBe(0);
    expect(dy).toBeCloseTo(-dh, 9); // drawn upward from the rotated anchor
    expect(dw).toBeCloseTo(dh, 9); // square grid
    // The pose triangle must not inherit the grid rotation: the last restore
    // (this draw's) precedes the triangle fill.
    const restores = display().restore.mock.invocationCallOrder;
    expect(restores[restores.length - 1]).toBeLessThan(
      display().fill.mock.invocationCallOrder[0],
    );
  });

  it("reuses the ImageData buffer across same-size grids", async () => {
    const { inflate, settlers } = deferredInflate();
    stop = startMapSink(store, MAP_CH, POSE_CH, canvas, health, { inflate, hidden: () => false });
    gridFrame(store, 1);
    settlers[0].resolve(new Uint8Array(4));
    await flush();
    gridFrame(store, 2);
    settlers[1].resolve(new Uint8Array(4));
    await flush();
    const first = backing().putImageData.mock.calls[0][0];
    expect(backing().putImageData.mock.calls[1][0]).toBe(first);

    const wider = costmapValue(3, 3, 2);
    store.ingest(MAP_CH, { ch: MAP_CH, seq: 3, ts: 3, delivery: "latest" }, wider, true);
    settlers[2].resolve(new Uint8Array(6));
    await flush();
    expect(backing().putImageData.mock.calls[2][0]).not.toBe(first);
  });

  it("sizes the backing store and pose marker by devicePixelRatio", async () => {
    vi.stubGlobal("devicePixelRatio", 2);
    const { inflate, settlers } = deferredInflate();
    stop = startMapSink(store, MAP_CH, POSE_CH, canvas, health, { inflate, hidden: () => false });
    gridFrame(store, 1);
    settlers[0].resolve(new Uint8Array(4));
    await flush();
    expect([canvas.width, canvas.height]).toEqual([200, 160]); // css 100x80 * dpr 2

    poseFrame(store, 1);
    const t = fitTransform({ w: 2, h: 2, res: 0.5, origin: [0.25, -0.5, 0] }, 200, 160);
    const [ex, ey] = posePath(t, { x: 0.5, y: 0.5, yaw: 0.25 }, 2)[0];
    const [nx, ny] = display().moveTo.mock.calls[0];
    expect(nx).toBeCloseTo(ex, 9); // the sink passed its dpr to the marker
    expect(ny).toBeCloseTo(ey, 9);
  });
});

describe("MapPanel", () => {
  const SPEC: PanelSpec = {
    id: "map",
    kind: "map2d",
    title: "",
    channels: [MAP_CH, POSE_CH],
    params: {},
  };
  let container: HTMLElement;
  let root: Root;
  let now: number;
  let store: ChannelStore;
  let deflated: Uint8Array;
  const badge = () => container.querySelector(`[data-testid="map2d-${MAP_CH}-badge"]`)!;

  beforeAll(async () => {
    // Real zlib bytes so the panel's default inflate path runs end to end.
    const stream = new Blob([Uint8Array.from([0, 50, 100, 255]) as BlobPart]).stream()
      .pipeThrough(new CompressionStream("deflate"));
    deflated = new Uint8Array(await new Response(stream).arrayBuffer());
  });

  beforeEach(() => {
    container = document.createElement("div");
    document.body.appendChild(container);
    root = createRoot(container);
    now = 1_000_000;
    store = new ChannelStore(() => now);
  });

  afterEach(() => {
    act(() => root.unmount());
    container.remove();
    vi.restoreAllMocks();
  });

  function realGridFrame(seq: number, ts = seq): void {
    const value: CostmapValue = {
      bytes: deflated,
      w: 2,
      h: 2,
      res: 0.5,
      origin: [0.25, -0.5, 0.0],
    };
    store.ingest(MAP_CH, { ch: MAP_CH, seq, ts, delivery: "latest" }, value, true);
  }

  it("shows waiting, then the Hz badge, then flags staleness", async () => {
    act(() => root.render(<MapPanel spec={SPEC} store={store} />));
    expect(container.textContent).toContain("waiting for data");
    expect(badge().textContent).toBe("waiting");
    expect(badge().getAttribute("role")).toBe("status");
    const canvas = container.querySelector("canvas")!;
    expect(canvas.getAttribute("role")).toBe("img");
    expect(canvas.getAttribute("aria-label")).toBe("map");

    // Grids at 5 Hz of source time, arriving with zero skew.
    await act(async () => {
      for (let i = 0; i < 5; i++) realGridFrame(i, now / 1000 - (4 - i) / 5);
      await flush();
      store.publishUi();
    });
    expect(container.textContent).not.toContain("waiting for data");
    expect(badge().textContent).toMatch(/Hz$/);
    expect(badge().getAttribute("data-stale")).toBeNull();

    // Silence: source age climbs past the threshold on a later UI tick.
    act(() => {
      now += 12_000;
      store.publishUi();
    });
    expect(badge().textContent).toMatch(/^stale/);
    expect(badge().getAttribute("data-stale")).toBe("true");
  });

  it("flags a failing inflate in the badge and recovers on the next grid", async () => {
    act(() => root.render(<MapPanel spec={SPEC} store={store} />));
    await act(async () => {
      // Not a zlib stream: the panel's real inflate rejects.
      store.ingest(
        MAP_CH,
        { ch: MAP_CH, seq: 1, ts: now / 1000, delivery: "latest" },
        costmapValue(9),
        true,
      );
      await flush();
      await flush();
      store.publishUi();
    });
    expect(badge().textContent).toBe("decode failing");
    expect(badge().getAttribute("data-error")).toBe("true");

    await act(async () => {
      realGridFrame(2, now / 1000);
      await flush();
      await flush();
      store.publishUi();
    });
    expect(badge().textContent).toMatch(/Hz$/);
    expect(badge().getAttribute("data-error")).toBeNull();
  });

  it("renders without a pose binding (single-channel spec)", async () => {
    act(() =>
      root.render(
        <MapPanel
          spec={{ id: "map", kind: "map2d", title: "", channels: [MAP_CH], params: {} }}
          store={store}
        />,
      )
    );
    await act(async () => {
      realGridFrame(1, now / 1000);
      await flush();
      store.publishUi();
    });
    expect(badge().textContent).toMatch(/Hz$/);
  });

  it("renders a visible note instead of a canvas when no channel is bound", () => {
    act(() =>
      root.render(
        <MapPanel
          spec={{ id: "map", kind: "map2d", title: "", channels: [], params: {} }}
          store={store}
        />,
      )
    );
    expect(container.textContent).toContain("no channel bound");
    expect(container.querySelector("canvas")).toBeNull();
  });
});
