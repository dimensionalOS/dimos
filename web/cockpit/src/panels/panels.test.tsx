// @vitest-environment happy-dom
import { afterEach, beforeAll, beforeEach, describe, expect, it, vi } from "vitest";
import { act } from "react";
import { createRoot, type Root } from "react-dom/client";
import type { FrameHeader, PanelSpec } from "@dimos/shared";
import type { CostmapValue, VoxelsValue } from "@dimos/sdk";
import { ChannelStore, PublishError } from "@dimos/sdk";
import type { DrawHealth } from "../layout/PanelFrame.tsx";
import { FakeSession } from "../testing/fakeSession.ts";
import { Map3DPanel, startVoxelSink, type VoxelSink } from "./Map3DPanel.tsx";
import { MapPanel, startMapSink } from "./MapPanel.tsx";
import { fitTransform, posePath } from "./mapRenderer.ts";
import { ChatPanel } from "./ChatPanel.tsx";
import { getPanel, UnknownPanel } from "./registry.tsx";
import { startVideoSink, VideoPanel } from "./VideoPanel.tsx";
import type { VoxelScene } from "./voxelView.ts";

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
    expect(badge().getAttribute("data-state")).toBe("waiting");
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
    expect(badge().getAttribute("data-state")).toBe("live");
    expect(badge().getAttribute("data-stale")).toBeNull();

    // Silence: source age climbs past the threshold on a later UI tick.
    act(() => {
      now += 5000;
      store.publishUi();
    });
    expect(badge().textContent).toMatch(/^stale/);
    expect(badge().getAttribute("data-stale")).toBe("true");
    expect(badge().getAttribute("data-state")).toBe("stale");
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
    expect(badge().getAttribute("data-state")).toBe("error");

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
    expect(badge().getAttribute("data-state")).toBe("stale");
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
    expect(badge().getAttribute("data-state")).toBe("error");
  });

  it("plays a track value in a <video> and drives the badge from its frames", () => {
    // happy-dom's MediaStreamTrack has an illegal constructor; a prototype
    // instance still passes the panel's instanceof check, and its real
    // MediaStream accepts it (the srcObject setter insists on the real class).
    const track = Object.create(MediaStreamTrack.prototype, {
      kind: { value: "video" },
    }) as MediaStreamTrack;
    const frameCallbacks: (() => void)[] = [];
    const proto = HTMLVideoElement.prototype as unknown as Record<string, unknown>;
    proto.requestVideoFrameCallback = (cb: () => void) => {
      frameCallbacks.push(cb);
      return frameCallbacks.length;
    };
    proto.cancelVideoFrameCallback = vi.fn();
    vi.spyOn(HTMLMediaElement.prototype, "play").mockResolvedValue();
    vi.spyOn(Date, "now").mockImplementation(() => now);
    act(() => root.render(<VideoPanel spec={SPEC} store={store} />));
    expect(container.querySelector("canvas")).not.toBeNull();
    act(() => {
      store.ingest(CH, header(1, now / 1000), track, true);
      store.publishUi();
    });
    const video = container.querySelector<HTMLVideoElement>(`[data-testid="video-${CH}-track"]`)!;
    expect(container.querySelector("canvas")).toBeNull();
    expect((video.srcObject as MediaStream).getVideoTracks()).toEqual([track]);
    // Connecting: the store saw the track, but no frame was presented yet.
    expect(badge().textContent).toBe("waiting");

    // Three presented frames within a second (600 ms: the slot age crosses a
    // 500 ms bucket, which is what republishes the badge).
    act(() => {
      for (let i = 0; i < 3; i++) {
        now += 200;
        frameCallbacks.shift()!();
      }
      store.publishUi();
    });
    expect(badge().textContent).toBe("3.0 fps");
    expect(badge().getAttribute("data-state")).toBe("live");

    // Silence past the threshold reads as stale, off the sink's clock.
    act(() => {
      now += 5000;
      store.publishUi();
    });
    expect(badge().textContent).toMatch(/^stale/);
    expect(badge().getAttribute("data-state")).toBe("stale");
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
    expect(getPanel("chat")).toBe(ChatPanel);
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
const PATH_CH = "path";
const CHS = { costmap: MAP_CH, pose: POSE_CH };

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

function pathFrame(store: ChannelStore, seq: number, points: [number, number][]): void {
  store.ingest(PATH_CH, { ch: PATH_CH, seq, ts: seq, delivery: "latest" }, points, true);
}

/** happy-dom has no layout; pin the on-screen rect the click handler reads. */
function defineRect(canvas: HTMLCanvasElement, left: number, top: number, w: number, h: number) {
  Object.defineProperty(canvas, "getBoundingClientRect", {
    configurable: true,
    value: () => ({ left, top, width: w, height: h }),
  });
}

function click(el: Element, clientX: number, clientY: number): void {
  el.dispatchEvent(new MouseEvent("click", { clientX, clientY, bubbles: true }));
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
    stroke: ReturnType<typeof vi.fn>;
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
        stroke: vi.fn(),
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
    stop = startMapSink(store, CHS, canvas, health, { inflate, hidden: () => false });

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
    stop = startMapSink(store, CHS, canvas, health, { inflate, hidden: () => false });
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
    stop = startMapSink(store, CHS, canvas, health, { inflate, hidden: () => false });
    poseFrame(store, 1);
    expect(display().drawImage).not.toHaveBeenCalled();
  });

  it("counts inflate rejections and recovers on the next grid", async () => {
    const { inflate, settlers } = deferredInflate();
    stop = startMapSink(store, CHS, canvas, health, { inflate, hidden: () => false });
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
    stop = startMapSink(store, CHS, canvas, health, { inflate, hidden: () => false });
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
    stop = startMapSink(store, CHS, canvas, health, { inflate, hidden: () => hidden });
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
    stop = startMapSink(store, CHS, canvas, health, {
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
    stop = startMapSink(store, CHS, canvas, health, { inflate, hidden: () => false });
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
    stop = startMapSink(store, CHS, canvas, health, { inflate, hidden: () => false });
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
    stop = startMapSink(store, CHS, canvas, health, { inflate, hidden: () => false });
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
    stop = startMapSink(store, CHS, canvas, health, { inflate, hidden: () => false });
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

  it("draws the path under the pose and clears it on an empty path", async () => {
    const { inflate, calls, settlers } = deferredInflate();
    const opts = { ...CHS, path: PATH_CH };
    stop = startMapSink(store, opts, canvas, health, { inflate, hidden: () => false });
    gridFrame(store, 1);
    settlers[0].resolve(new Uint8Array(4));
    await flush();
    poseFrame(store, 1);
    expect(display().stroke).not.toHaveBeenCalled();

    // The 1x1 m grid fits the 100x80 canvas at 80 px/m, letterboxed to
    // x 10..90 with world y -0.5 on canvas row 80.
    pathFrame(store, 1, [[0.5, -0.25], [0.75, 0], [1.0, 0.25]]);
    expect(display().stroke).toHaveBeenCalledTimes(1);
    expect(display().moveTo.mock.calls).toContainEqual([30, 60]);
    expect(display().lineTo.mock.calls).toContainEqual([50, 40]);
    expect(display().lineTo.mock.calls).toContainEqual([70, 20]);
    // Under the pose: the stroke precedes this draw's triangle fill.
    const fills = display().fill.mock.invocationCallOrder;
    expect(display().stroke.mock.invocationCallOrder[0]).toBeLessThan(fills[fills.length - 1]);
    expect(calls.length).toBe(1); // no re-inflate for an overlay

    pathFrame(store, 2, []); // cancel/arrival: the overlay goes, the pose stays
    expect(display().stroke).toHaveBeenCalledTimes(1);
    expect(display().fill).toHaveBeenCalledTimes(3);
  });

  it("maps a click through the fitted transform at the device pixel ratio", async () => {
    vi.stubGlobal("devicePixelRatio", 2);
    const { inflate, settlers } = deferredInflate();
    const clicks: [number, number][] = [];
    const opts = { ...CHS, onClick: (x: number, y: number) => clicks.push([x, y]) };
    stop = startMapSink(store, opts, canvas, health, { inflate, hidden: () => false });
    defineRect(canvas, 10, 20, 100, 80);
    click(canvas, 60, 60);
    expect(clicks).toEqual([]); // no grid yet: no world frame

    gridFrame(store, 1);
    settlers[0].resolve(new Uint8Array(4));
    await flush();
    // The canvas centre is the centre of the fitted grid: world (0.75, 0).
    click(canvas, 60, 60);
    // CSS (10, 60) is backing-store (20, 120): the AABB's left edge, a
    // quarter of the way up.
    click(canvas, 20, 80);
    expect(clicks.length).toBe(2);
    expect(clicks[0][0]).toBeCloseTo(0.75, 9);
    expect(clicks[0][1]).toBeCloseTo(0, 9);
    expect(clicks[1][0]).toBeCloseTo(0.25, 9);
    expect(clicks[1][1]).toBeCloseTo(-0.25, 9);

    stop!();
    stop = null;
    click(canvas, 60, 60);
    expect(clicks.length).toBe(2); // the listener left with the sink
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
    expect(badge().getAttribute("data-state")).toBe("waiting");
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
    expect(badge().getAttribute("data-state")).toBe("live");
    expect(badge().getAttribute("data-stale")).toBeNull();

    // Silence: source age climbs past the threshold on a later UI tick.
    act(() => {
      now += 12_000;
      store.publishUi();
    });
    expect(badge().textContent).toMatch(/^stale/);
    expect(badge().getAttribute("data-stale")).toBe("true");
    expect(badge().getAttribute("data-state")).toBe("stale");
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
    expect(badge().getAttribute("data-state")).toBe("error");

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

  const NAV_SPEC: PanelSpec = {
    ...SPEC,
    params: { path: PATH_CH, click: "clicked_point", stop: "stop_movement" },
  };
  const cancel = () => container.querySelector(`[data-testid="map2d-${MAP_CH}-cancel"]`);

  it("publishes a click as {x, y} on the click channel and reports a rejection", async () => {
    const session = new FakeSession();
    act(() => root.render(<MapPanel spec={NAV_SPEC} store={store} session={session} />));
    const canvas = container.querySelector("canvas")!;
    defineRect(canvas, 0, 0, 100, 80);
    await act(async () => {
      realGridFrame(1, now / 1000);
      await flush();
    });
    act(() => click(canvas, 50, 40)); // the centre of the fitted grid
    expect(session.published).toEqual([["clicked_point", { x: 0.75, y: 0 }]]);
    await act(async () => {});
    expect(container.querySelector('[role="alert"]')).toBeNull();

    session.reject = new PublishError("rejected", "not_connected", "no session");
    act(() => click(canvas, 50, 40));
    await act(async () => {});
    expect(container.querySelector('[role="alert"]')!.textContent).toBe(
      "send failed: not_connected: no session",
    );
  });

  it("shows the cancel button only while a path is active and publishes the stop", () => {
    const session = new FakeSession();
    act(() => root.render(<MapPanel spec={NAV_SPEC} store={store} session={session} />));
    expect(cancel()).toBeNull();
    act(() => {
      pathFrame(store, 1, [[0.5, 0], [1.0, 0]]);
      store.publishUi();
    });
    expect(cancel()).not.toBeNull();
    act(() => (cancel() as HTMLButtonElement).click());
    expect(session.published).toEqual([["stop_movement", true]]);

    act(() => {
      pathFrame(store, 2, []); // the planner cleared it: nothing left to cancel
      store.publishUi();
    });
    expect(cancel()).toBeNull();
  });

  it("renders neither the cancel button nor the crosshair without a session", () => {
    act(() => root.render(<MapPanel spec={NAV_SPEC} store={store} />));
    act(() => {
      pathFrame(store, 1, [[0.5, 0], [1.0, 0]]);
      store.publishUi();
    });
    expect(cancel()).toBeNull();
    expect(container.querySelector("canvas")!.className).not.toContain("clickable");
  });
});

const VOX_CH = "global_map";
const POSITIONS = Float32Array.from([0.025, 0.025, 0.025, 0.075, 0.025, 0.125]);

function fakeScene() {
  return {
    setVoxels: vi.fn(),
    setPose: vi.fn(),
    fit: vi.fn(),
    setFollow: vi.fn(),
    resize: vi.fn(),
    render: vi.fn(),
    dispose: vi.fn(),
  };
}

function voxelFrame(store: ChannelStore, seq: number, ts = seq, n = 2): VoxelsValue {
  const value: VoxelsValue = { bytes: new Uint8Array([seq]), res: 0.05, n, chunks: n > 0 ? 1 : 0 };
  store.ingest(VOX_CH, { ch: VOX_CH, seq, ts, delivery: "latest" }, value, true);
  return value;
}

/** Inflate stub whose promises settle only when the test says so. */
function deferredInflateVoxels() {
  const calls: VoxelsValue[] = [];
  const settlers: { resolve: (p: Float32Array) => void; reject: (e: Error) => void }[] = [];
  const inflate = (value: VoxelsValue): Promise<Float32Array> => {
    calls.push(value);
    return new Promise((resolve, reject) => settlers.push({ resolve, reject }));
  };
  return { inflate, calls, settlers };
}

describe("startVoxelSink", () => {
  const CHS = { cloud: VOX_CH, pose: POSE_CH };
  let store: ChannelStore;
  let canvas: HTMLCanvasElement;
  let health: DrawHealth;
  let scene: ReturnType<typeof fakeScene>;
  let sink: VoxelSink | null;
  const deps = (over: Record<string, unknown> = {}) => ({
    hidden: () => false,
    createScene: () => Promise.resolve(scene as unknown as VoxelScene),
    ...over,
  });

  beforeEach(() => {
    store = new ChannelStore();
    canvas = document.createElement("canvas");
    defineSize(canvas, 100, 80);
    health = { lastDrawOkAtMs: 0, failures: 0 };
    scene = fakeScene();
    sink = null;
  });

  afterEach(() => {
    sink?.stop();
    vi.restoreAllMocks();
  });

  it("inflates one frame at a time once the scene is up and skips to the newest", async () => {
    const { inflate, calls, settlers } = deferredInflateVoxels();
    voxelFrame(store, 1); // a slot may predate the mount
    sink = startVoxelSink(store, CHS, canvas, health, deps({ inflate }));
    expect(calls.length).toBe(0); // the scene (a lazy chunk) is not up yet
    await flush();
    expect(calls.length).toBe(1);
    voxelFrame(store, 2);
    const newest = voxelFrame(store, 3);
    expect(calls.length).toBe(1); // one inflate in flight, burst sheds

    settlers[0].resolve(POSITIONS);
    await flush();
    expect(scene.setVoxels).toHaveBeenCalledTimes(1);
    const [positions, colors, res] = scene.setVoxels.mock.calls[0] as [
      Float32Array,
      Uint8Array,
      number,
    ];
    expect(positions).toBe(POSITIONS);
    expect(colors.length).toBe(6);
    expect(res).toBe(0.05);
    expect(scene.fit).toHaveBeenCalledTimes(1);
    expect(scene.render).toHaveBeenCalled();
    expect(canvas.dataset.voxels).toBe("2");
    expect(health.failures).toBe(0);
    expect(calls.length).toBe(2);
    expect(calls[1]).toBe(newest); // frame 2 was never inflated

    settlers[1].resolve(POSITIONS);
    await flush();
    expect(scene.fit).toHaveBeenCalledTimes(1); // only the first frame frames the camera
    expect(calls.length).toBe(2); // caught up
  });

  it("clears the scene on an empty frame and frames the camera on the first voxels", async () => {
    const { inflate, settlers } = deferredInflateVoxels();
    sink = startVoxelSink(store, CHS, canvas, health, deps({ inflate }));
    await flush();
    const drawn = (call: number) => (scene.setVoxels.mock.calls[call] as [Float32Array])[0].length;

    voxelFrame(store, 1, 1, 0); // a cleared map before any voxels: nothing to frame yet
    settlers[0].resolve(new Float32Array(0));
    await flush();
    expect(drawn(0)).toBe(0);
    expect(canvas.dataset.voxels).toBe("0");
    expect(scene.fit).not.toHaveBeenCalled();

    voxelFrame(store, 2);
    settlers[1].resolve(POSITIONS);
    await flush();
    expect(drawn(1)).toBe(6);
    expect(canvas.dataset.voxels).toBe("2");
    expect(scene.fit).toHaveBeenCalledTimes(1);

    voxelFrame(store, 3, 3, 0); // the producer cleared its map: the old voxels must go
    settlers[2].resolve(new Float32Array(0));
    await flush();
    expect(drawn(2)).toBe(0);
    expect(canvas.dataset.voxels).toBe("0");
    expect(health.failures).toBe(0);

    voxelFrame(store, 4);
    settlers[3].resolve(POSITIONS);
    await flush();
    expect(drawn(3)).toBe(6);
    expect(canvas.dataset.voxels).toBe("2");
    expect(scene.fit).toHaveBeenCalledTimes(1); // the viewer's orbit is left alone
  });

  it("redraws the pose without a new inflate and passes the follow toggle on", async () => {
    const { inflate, calls, settlers } = deferredInflateVoxels();
    sink = startVoxelSink(store, CHS, canvas, health, deps({ inflate }));
    await flush();
    voxelFrame(store, 1);
    settlers[0].resolve(POSITIONS);
    await flush();
    const renders = scene.render.mock.calls.length;

    poseFrame(store, 1);
    expect(scene.setPose).toHaveBeenLastCalledWith({ x: 0.5, y: 0.5, z: 0.1, yaw: 0.25 });
    expect(scene.render.mock.calls.length).toBe(renders + 1);
    expect(calls.length).toBe(1);

    sink.follow(true);
    expect(scene.setFollow).toHaveBeenLastCalledWith(true);
    expect(scene.render.mock.calls.length).toBe(renders + 2);
    sink.follow(false);
    expect(scene.setFollow).toHaveBeenLastCalledWith(false);
  });

  it("carries a follow toggled before the scene is up into it and skips the first fit", async () => {
    const { inflate, settlers } = deferredInflateVoxels();
    sink = startVoxelSink(store, CHS, canvas, health, deps({ inflate }));
    sink.follow(true); // clicked while the three.js chunk was still loading
    expect(scene.setFollow).not.toHaveBeenCalled();
    await flush();
    expect(scene.setFollow).toHaveBeenCalledWith(true);
    voxelFrame(store, 1);
    settlers[0].resolve(POSITIONS);
    await flush();
    expect(scene.setVoxels).toHaveBeenCalledTimes(1);
    expect(scene.fit).not.toHaveBeenCalled(); // the camera stays behind the robot
  });

  it("does not inflate while hidden and catches up on visibilitychange", async () => {
    const { inflate, calls, settlers } = deferredInflateVoxels();
    let hidden = true;
    sink = startVoxelSink(store, CHS, canvas, health, deps({ inflate, hidden: () => hidden }));
    await flush();
    voxelFrame(store, 1);
    const newest = voxelFrame(store, 2);
    expect(calls.length).toBe(0); // a backgrounded panel costs no inflate

    hidden = false;
    document.dispatchEvent(new Event("visibilitychange"));
    expect(calls.length).toBe(1);
    expect(calls[0]).toBe(newest);
    settlers[0].resolve(POSITIONS);
    await flush();
  });

  it("counts inflate rejections and recovers on the next frame", async () => {
    const { inflate, settlers } = deferredInflateVoxels();
    sink = startVoxelSink(store, CHS, canvas, health, deps({ inflate }));
    await flush();
    const stamp = health.lastDrawOkAtMs;
    voxelFrame(store, 1);
    settlers[0].reject(new Error("corrupt zlib"));
    await flush();
    expect(health.failures).toBe(1);
    expect(health.lastDrawOkAtMs).toBe(stamp); // only successes stamp it

    voxelFrame(store, 2);
    settlers[1].resolve(POSITIONS);
    await flush();
    expect(health.failures).toBe(0);
  });

  it("reports an unavailable renderer and never inflates", async () => {
    const { inflate, calls } = deferredInflateVoxels();
    const onError = vi.fn();
    sink = startVoxelSink(store, CHS, canvas, health, {
      inflate,
      hidden: () => false,
      createScene: () => Promise.reject(new Error("no webgl")),
      onError,
    });
    voxelFrame(store, 1);
    await flush();
    expect(onError).toHaveBeenCalledWith("3D view unavailable: no webgl");
    expect(calls.length).toBe(0);
    expect(health.failures).toBe(0); // the badge says "stalled", the overlay says why
  });

  it("resizes the scene from the layout and disposes it on stop", async () => {
    const { inflate } = deferredInflateVoxels();
    let resize: (() => void) | null = null;
    const disposeObserver = vi.fn();
    sink = startVoxelSink(
      store,
      CHS,
      canvas,
      health,
      deps({
        inflate,
        observeResize: (_el: Element, cb: () => void) => {
          resize = cb;
          return disposeObserver;
        },
      }),
    );
    await flush();
    expect(scene.resize).toHaveBeenCalledWith(100, 80, 1);

    defineSize(canvas, 250, 80);
    resize!();
    expect(scene.resize).toHaveBeenLastCalledWith(250, 80, 1);

    sink.stop();
    sink = null;
    expect(scene.dispose).toHaveBeenCalledTimes(1);
    expect(disposeObserver).toHaveBeenCalledTimes(1);
  });

  it("disposes a scene that arrives after stop", async () => {
    let resolveScene: (s: VoxelScene) => void = () => {};
    sink = startVoxelSink(store, CHS, canvas, health, {
      hidden: () => false,
      createScene: () => new Promise((resolve) => (resolveScene = resolve)),
    });
    sink.stop();
    sink = null;
    resolveScene(scene as unknown as VoxelScene);
    await flush();
    expect(scene.dispose).toHaveBeenCalledTimes(1);
    expect(scene.resize).not.toHaveBeenCalled();
  });
});

describe("Map3DPanel", () => {
  const SPEC: PanelSpec = {
    id: "map3d",
    kind: "map3d",
    title: "",
    channels: [VOX_CH, POSE_CH],
    params: {},
  };
  let container: HTMLElement;
  let root: Root;
  let now: number;
  let store: ChannelStore;
  let scene: ReturnType<typeof fakeScene>;
  const badge = () => container.querySelector(`[data-testid="map3d-${VOX_CH}-badge"]`)!;
  const sinkDeps = () => ({
    createScene: () => Promise.resolve(scene as unknown as VoxelScene),
    inflate: () => Promise.resolve(POSITIONS),
  });

  beforeEach(() => {
    container = document.createElement("div");
    document.body.appendChild(container);
    root = createRoot(container);
    now = 1_000_000;
    store = new ChannelStore(() => now);
    scene = fakeScene();
  });

  afterEach(() => {
    act(() => root.unmount());
    container.remove();
    vi.restoreAllMocks();
  });

  it("is registered for the map3d kind", () => {
    expect(getPanel("map3d")).toBe(Map3DPanel);
  });

  it("shows waiting, then the Hz badge, then flags staleness", async () => {
    const deps = sinkDeps();
    act(() => root.render(<Map3DPanel spec={SPEC} store={store} sinkDeps={deps} />));
    expect(container.textContent).toContain("waiting for data");
    expect(badge().textContent).toBe("waiting");
    expect(badge().getAttribute("data-state")).toBe("waiting");
    const canvas = container.querySelector("canvas")!;
    expect(canvas.getAttribute("data-testid")).toBe(`map3d-${VOX_CH}-canvas`);
    expect(canvas.getAttribute("aria-label")).toBe("map3d");

    // Frames at 1 Hz of source time, arriving with zero skew.
    await act(async () => {
      for (let i = 0; i < 3; i++) voxelFrame(store, i, now / 1000 - (2 - i));
      await flush();
      store.publishUi();
    });
    expect(container.textContent).not.toContain("waiting for data");
    expect(badge().textContent).toMatch(/Hz$/);
    expect(badge().getAttribute("data-state")).toBe("live");
    expect(canvas.dataset.voxels).toBe("2");
    expect(scene.setVoxels).toHaveBeenCalled();

    // Silence: source age climbs past the threshold on a later UI tick.
    act(() => {
      now += 20_000;
      store.publishUi();
    });
    expect(badge().textContent).toMatch(/^stale/);
    expect(badge().getAttribute("data-state")).toBe("stale");
  });

  it("toggles follow from its button, which needs a pose channel", async () => {
    const deps = sinkDeps();
    const follow = () => container.querySelector(`[data-testid="map3d-${VOX_CH}-follow"]`);
    act(() => root.render(<Map3DPanel spec={SPEC} store={store} sinkDeps={deps} />));
    await act(flush);
    expect(follow()!.getAttribute("aria-pressed")).toBe("false");
    act(() => follow()!.dispatchEvent(new MouseEvent("click", { bubbles: true })));
    expect(follow()!.getAttribute("aria-pressed")).toBe("true");
    expect(scene.setFollow).toHaveBeenLastCalledWith(true);
    act(() => follow()!.dispatchEvent(new MouseEvent("click", { bubbles: true })));
    expect(follow()!.getAttribute("aria-pressed")).toBe("false");
    expect(scene.setFollow).toHaveBeenLastCalledWith(false);

    const noPose = { ...SPEC, channels: [VOX_CH] };
    act(() => root.render(<Map3DPanel spec={noPose} store={store} sinkDeps={deps} />));
    expect(follow()).toBeNull();
  });

  it("shows why the renderer could not start", async () => {
    const deps = { createScene: () => Promise.reject(new Error("no webgl")) };
    act(() => root.render(<Map3DPanel spec={SPEC} store={store} sinkDeps={deps} />));
    await act(flush);
    const alert = container.querySelector('[role="alert"]')!;
    expect(alert.textContent).toBe("3D view unavailable: no webgl");
  });

  it("renders a bridge authoring mistake instead of crashing", () => {
    act(() => root.render(<Map3DPanel spec={{ ...SPEC, channels: [] }} store={store} />));
    expect(container.textContent).toContain("map3d panel map3d: no channel bound");
  });
});
