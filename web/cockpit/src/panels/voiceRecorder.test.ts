import { describe, expect, it } from "vitest";
import {
  type AudioFrame,
  MAX_SLICE_BYTES,
  MAX_UTTERANCE_MS,
  MIN_SEND_INTERVAL_MS,
  type RecorderPort,
  TIMESLICE_MS,
  VoiceRecorder,
  type VoiceRecorderDeps,
} from "./voiceRecorder.ts";

const tick = (): Promise<void> => new Promise((resolve) => setTimeout(resolve, 0));

interface Harness {
  recorder: VoiceRecorder;
  published: AudioFrame[];
  mic: { started: number[]; stopped: number };
  slept: number[];
  ondata: (bytes: Uint8Array) => void;
  arm: () => void;
  wakeAutoRelease: () => void;
  rejectSeq: number;
  settle: (phase?: string) => Promise<void>;
}

function harness(opts: { manualArm?: boolean } = {}): Harness {
  const h = {} as Harness;
  h.published = [];
  h.mic = { started: [], stopped: 0 };
  h.slept = [];
  h.ondata = () => {};
  h.arm = () => {};
  h.wakeAutoRelease = () => {};
  h.rejectSeq = -1;
  let sid = 0;
  const deps: VoiceRecorderDeps = {
    openMic: (ondata) => {
      h.ondata = ondata;
      const port: RecorderPort = {
        mimeType: "audio/webm",
        start: (timesliceMs) => h.mic.started.push(timesliceMs),
        stop: () => {
          h.mic.stopped++;
          return Promise.resolve();
        },
      };
      if (!opts.manualArm) return Promise.resolve(port);
      return new Promise((resolve) => {
        h.arm = () => resolve(port);
      });
    },
    publish: (frame) => {
      h.published.push(frame);
      return frame.seq === h.rejectSeq
        ? Promise.reject(new Error("rate_limited: slow down"))
        : Promise.resolve({});
    },
    newSid: () => `sid-${++sid}`,
    sleep: (ms) => {
      h.slept.push(ms);
      // The auto-release hold must not fire on its own; everything else
      // (the inter-send floor) resolves immediately.
      if (ms < MAX_UTTERANCE_MS) return Promise.resolve();
      return new Promise((resolve) => {
        h.wakeAutoRelease = resolve;
      });
    },
  };
  h.recorder = new VoiceRecorder(deps);
  h.settle = async (phase = "idle") => {
    for (let i = 0; i < 200 && h.recorder.getSnapshot().phase !== phase; i++) await tick();
    expect(h.recorder.getSnapshot().phase).toBe(phase);
  };
  return h;
}

function bytesOf(frames: AudioFrame[]): Uint8Array {
  const text = frames.map((f) => atob(f.data)).join("");
  return Uint8Array.from(text, (c) => c.charCodeAt(0));
}

describe("VoiceRecorder", () => {
  it("press arms then records at the timeslice", async () => {
    const h = harness();
    expect(h.recorder.getSnapshot().phase).toBe("idle");
    h.recorder.press();
    expect(h.recorder.getSnapshot().phase).toBe("arming");
    await h.settle("recording");
    expect(h.mic.started).toEqual([TIMESLICE_MS]);
    expect(h.published).toEqual([]);
  });

  it("release stops, drains slices in order, and closes with an empty final frame", async () => {
    const h = harness();
    h.recorder.press();
    await h.settle("recording");
    const source = Uint8Array.from({ length: MAX_SLICE_BYTES * 2 + 100 }, (_, i) => i % 256);
    h.ondata(source);
    h.recorder.release();
    await h.settle();
    expect(h.mic.stopped).toBe(1);
    expect(h.published.map((f) => [f.seq, f.data.length > 0, f.final])).toEqual([
      [0, true, false],
      [1, true, false],
      [2, true, false],
      [3, false, true],
    ]);
    expect(h.published.every((f) => f.sid === "sid-1" && f.mime === "audio/webm")).toBe(true);
    expect(bytesOf(h.published)).toEqual(source);
  });

  it("paces every non-final send by the interval floor", async () => {
    const h = harness();
    h.recorder.press();
    await h.settle("recording");
    h.ondata(new Uint8Array(MAX_SLICE_BYTES * 3));
    h.recorder.release();
    await h.settle();
    expect(h.slept.filter((ms) => ms === MIN_SEND_INTERVAL_MS).length).toBe(3);
  });

  it("cancel before any audio sends nothing", async () => {
    const h = harness();
    h.recorder.press();
    await h.settle("recording");
    h.recorder.cancel();
    expect(h.recorder.getSnapshot().phase).toBe("idle");
    await tick();
    expect(h.published).toEqual([]);
    expect(h.mic.stopped).toBe(1);
  });

  it("cancel mid-utterance never sends a final frame", async () => {
    const h = harness();
    h.recorder.press();
    await h.settle("recording");
    h.ondata(new Uint8Array(10));
    await tick();
    h.recorder.cancel();
    await tick();
    await tick();
    expect(h.published.some((f) => f.final)).toBe(false);
    expect(h.recorder.getSnapshot().phase).toBe("idle");
  });

  it("release while arming aborts cleanly with nothing recorded", async () => {
    const h = harness({ manualArm: true });
    h.recorder.press();
    expect(h.recorder.getSnapshot().phase).toBe("arming");
    h.recorder.release();
    h.arm();
    await h.settle();
    expect(h.published).toEqual([]);
    expect(h.mic.started).toEqual([]);
    expect(h.mic.stopped).toBe(1);
  });

  it("a denied microphone lands in the error phase", async () => {
    const denied = new VoiceRecorder({
      openMic: () => Promise.reject(new Error("Permission denied")),
      publish: () => Promise.resolve({}),
    });
    denied.press();
    for (let i = 0; i < 50 && denied.getSnapshot().phase !== "error"; i++) await tick();
    expect(denied.getSnapshot()).toEqual({
      phase: "error",
      message: "microphone unavailable: Permission denied",
    });
  });

  it("a rejected publish aborts the utterance into the error phase", async () => {
    const h = harness();
    h.rejectSeq = 1;
    h.recorder.press();
    await h.settle("recording");
    h.ondata(new Uint8Array(MAX_SLICE_BYTES * 3));
    h.recorder.release();
    await h.settle("error");
    expect(h.recorder.getSnapshot().message).toBe("rate_limited: slow down");
    expect(h.published.length).toBe(2);
    expect(h.published.some((f) => f.final)).toBe(false);
    // The next press starts fresh and clears the error.
    h.recorder.press();
    expect(h.recorder.getSnapshot().phase).toBe("arming");
    await h.settle("recording");
    h.recorder.cancel();
  });

  it("auto-releases at the utterance cap", async () => {
    const h = harness();
    h.recorder.press();
    await h.settle("recording");
    h.ondata(new Uint8Array(5));
    h.wakeAutoRelease();
    await h.settle();
    expect(h.published.at(-1)?.final).toBe(true);
  });

  it("each utterance gets a fresh sid with seqs from zero", async () => {
    const h = harness();
    for (const _ of [1, 2]) {
      h.recorder.press();
      await h.settle("recording");
      h.ondata(new Uint8Array(4));
      h.recorder.release();
      await h.settle();
    }
    const bySid = new Map<string, number[]>();
    for (const frame of h.published) {
      bySid.set(frame.sid, [...(bySid.get(frame.sid) ?? []), frame.seq]);
    }
    expect([...bySid.entries()]).toEqual([
      ["sid-1", [0, 1]],
      ["sid-2", [0, 1]],
    ]);
  });
});
