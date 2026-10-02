// @vitest-environment happy-dom
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { act } from "react";
import { createRoot, type Root } from "react-dom/client";
import type { Session } from "@dimos/sdk";
import { ChatMic } from "./ChatMic.tsx";
import type { AudioFrame } from "./voiceRecorder.ts";

(globalThis as { IS_REACT_ACT_ENVIRONMENT?: boolean }).IS_REACT_ACT_ENVIRONMENT = true;

class FakeMediaRecorder {
  static instances: FakeMediaRecorder[] = [];
  static supported = ["audio/webm;codecs=opus"];
  static isTypeSupported = (mime: string): boolean => FakeMediaRecorder.supported.includes(mime);
  state: "inactive" | "recording" = "inactive";
  mimeType: string;
  started: number[] = [];
  ondataavailable: ((e: { data: { arrayBuffer(): Promise<ArrayBuffer> } }) => void) | null = null;
  onstop: (() => void) | null = null;
  onerror: ((e: unknown) => void) | null = null;
  constructor(readonly stream: unknown, options?: { mimeType?: string }) {
    this.mimeType = options?.mimeType ?? "";
    FakeMediaRecorder.instances.push(this);
  }
  start(timesliceMs: number): void {
    this.state = "recording";
    this.started.push(timesliceMs);
  }
  stop(): void {
    this.state = "inactive";
    this.onstop?.();
  }
  emit(bytes: Uint8Array): void {
    const buffer = bytes.slice().buffer as ArrayBuffer;
    this.ondataavailable?.({ data: { arrayBuffer: () => Promise.resolve(buffer) } });
  }
  emitUnreadable(): void {
    this.ondataavailable?.({ data: { arrayBuffer: () => Promise.reject(new Error("blob gone")) } });
  }
}

describe("ChatMic", () => {
  let container: HTMLElement;
  let root: Root;
  let published: [string, AudioFrame][];
  let errors: (string | null)[];
  let tracks: { stop: ReturnType<typeof vi.fn> }[];
  let micGrants: number;
  const session = {
    publish: (ch: string, value: unknown) => {
      published.push([ch, value as AudioFrame]);
      return Promise.resolve({ ch, relayTs: 1, bridgeTs: 2 });
    },
  } as unknown as Session;

  beforeEach(() => {
    container = document.createElement("div");
    document.body.appendChild(container);
    root = createRoot(container);
    published = [];
    errors = [];
    FakeMediaRecorder.instances = [];
    FakeMediaRecorder.supported = ["audio/webm;codecs=opus"];
    tracks = [{ stop: vi.fn() }];
    micGrants = 0;
  });

  afterEach(() => {
    act(() => root.unmount());
    container.remove();
    vi.unstubAllGlobals();
  });

  function stubMicApis(): void {
    vi.stubGlobal("isSecureContext", true);
    vi.stubGlobal("MediaRecorder", FakeMediaRecorder);
    Object.defineProperty(navigator, "mediaDevices", {
      configurable: true,
      value: {
        getUserMedia: () => {
          micGrants++;
          return Promise.resolve({ getTracks: () => tracks });
        },
      },
    });
  }

  function mount(connected = true): void {
    act(() =>
      root.render(
        <ChatMic
          session={session}
          ch="audio_in"
          connected={connected}
          onError={(m) => errors.push(m)}
        />,
      )
    );
  }

  function button(): HTMLButtonElement {
    const el = container.querySelector<HTMLButtonElement>('[data-testid="chat-audio_in-mic"]');
    if (el === null) throw new Error("mic button not rendered");
    return el;
  }

  function press(): void {
    act(() => {
      button().dispatchEvent(new Event("pointerdown", { bubbles: true }));
    });
  }

  function release(): void {
    act(() => {
      button().dispatchEvent(new Event("pointerup", { bubbles: true }));
    });
  }

  async function settle(state: string): Promise<void> {
    // Real (not fake) time: the machine paces sends with a 60 ms floor.
    for (let i = 0; i < 100 && button().dataset.state !== state; i++) {
      await act(async () => {
        await new Promise((resolve) => setTimeout(resolve, 5));
      });
    }
    expect(button().dataset.state).toBe(state);
  }

  it("renders nothing without microphone APIs", () => {
    mount();
    expect(container.querySelector("button")).toBeNull();
  });

  it("hold-speak-release publishes the recording and a final frame", async () => {
    stubMicApis();
    mount();
    expect(button().dataset.state).toBe("idle");
    press();
    await settle("recording");
    const [recorder] = FakeMediaRecorder.instances;
    expect(recorder.started).toEqual([500]);
    expect(recorder.mimeType).toBe("audio/webm;codecs=opus");
    act(() => recorder.emit(Uint8Array.from([1, 2, 3, 4])));
    release();
    await settle("idle");
    expect(published.every(([ch]) => ch === "audio_in")).toBe(true);
    const frames = published.map(([, frame]) => frame);
    expect(frames.length).toBe(2);
    expect(frames[0]).toMatchObject({ seq: 0, mime: "audio/webm;codecs=opus", final: false });
    expect(atob(frames[0].data)).toBe("\x01\x02\x03\x04");
    expect(frames[1]).toMatchObject({ seq: 1, data: "", final: true });
    expect(frames[0].sid).toBe(frames[1].sid);

    // The second hold reuses the granted mic and starts a fresh sid.
    press();
    await settle("recording");
    release();
    await settle("idle");
    expect(micGrants).toBe(1);
    expect(published.at(-1)?.[1].sid).not.toBe(frames[0].sid);
  });

  it("falls back to mp4 when webm is unsupported and stamps it on frames", async () => {
    stubMicApis();
    FakeMediaRecorder.supported = ["audio/mp4"];
    mount();
    press();
    await settle("recording");
    const [recorder] = FakeMediaRecorder.instances;
    expect(recorder.mimeType).toBe("audio/mp4");
    act(() => recorder.emit(Uint8Array.from([9])));
    release();
    await settle("idle");
    expect(published.every(([, frame]) => frame.mime === "audio/mp4")).toBe(true);
  });

  it("a failed Blob read errors out instead of wedging in sending", async () => {
    stubMicApis();
    mount();
    press();
    await settle("recording");
    act(() => FakeMediaRecorder.instances[0].emitUnreadable());
    release();
    await settle("error");
    expect(errors.at(-1)).toBe("voice failed: blob gone");
    expect(published.some(([, frame]) => frame.final)).toBe(false);
    // The next press clears the reported error and records again.
    press();
    await settle("recording");
    expect(errors.at(-1)).toBeNull();
    act(() => root.unmount());
  });

  it("Escape cancels the hold without a final frame", async () => {
    stubMicApis();
    mount();
    press();
    await settle("recording");
    act(() => FakeMediaRecorder.instances[0].emit(Uint8Array.from([7])));
    act(() => {
      button().dispatchEvent(
        new KeyboardEvent("keydown", { key: "Escape", bubbles: true, cancelable: true }),
      );
    });
    await settle("idle");
    await act(async () => {});
    expect(published.some(([, frame]) => frame.final)).toBe(false);
  });

  it("stays usable mid-hold if the connection drops, and unmount releases the mic", async () => {
    stubMicApis();
    mount();
    press();
    await settle("recording");
    mount(false); // connected flips mid-hold: the button must not disable
    expect(button().disabled).toBe(false);
    release();
    await settle("idle");
    act(() => root.unmount());
    expect(tracks[0].stop).toHaveBeenCalled();
  });

  it("is disabled while idle and disconnected", () => {
    stubMicApis();
    mount(false);
    expect(button().disabled).toBe(true);
  });
});
