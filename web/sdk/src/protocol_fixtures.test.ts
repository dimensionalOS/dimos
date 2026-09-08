// The SDK consumes shared/protocol.ts through the vite/vitest pipeline
// (alias + bundler resolution) instead of Deno's. Running the golden vectors
// here proves that pipeline yields byte-identical framing.

import { describe, expect, it } from "vitest";
import {
  ControlFrameReader,
  DataFrameStreamReader,
  decodeDatagram,
  encodeControlFrame,
  encodeDataFrame,
  encodeDatagram,
  type FrameHeader,
  type Msg,
  msgFromUnknown,
} from "@dimos/shared";
import { ManifestError, parseManifest } from "@dimos/shared/manifest";
import controlFrames from "../../shared/fixtures/control_frames.json";
import dataFrames from "../../shared/fixtures/data_frames.json";
import datagrams from "../../shared/fixtures/datagrams.json";
import manifests from "../../shared/fixtures/manifests.json";
import txMessages from "../../shared/fixtures/tx_messages.json";

function b64ToBytes(b64: string): Uint8Array {
  return Uint8Array.from(atob(b64), (c) => c.charCodeAt(0));
}

describe("control frame golden vectors", () => {
  for (const vector of controlFrames.vectors) {
    it(`encodes ${vector.name} byte-exactly`, () => {
      expect(encodeControlFrame(vector.message as Msg)).toEqual(b64ToBytes(vector.b64));
    });
  }

  it("decodes every vector whole and byte-at-a-time", () => {
    const whole = new ControlFrameReader();
    const trickle = new ControlFrameReader();
    const expected = controlFrames.vectors.map((v) => v.message);

    const all = controlFrames.vectors.flatMap((v) => [...b64ToBytes(v.b64)]);
    expect(whole.push(Uint8Array.from(all))).toEqual(expected);

    const decoded: unknown[] = [];
    for (const byte of all) decoded.push(...trickle.push(Uint8Array.of(byte)));
    expect(decoded).toEqual(expected);
  });
});

describe("datagram golden vectors", () => {
  for (const vector of datagrams.vectors) {
    it(`round-trips ${vector.name}`, () => {
      expect(encodeDatagram(vector.message as Msg)).toEqual(b64ToBytes(vector.b64));
      expect(decodeDatagram(b64ToBytes(vector.b64))).toEqual(vector.message);
    });
  }
});

describe("tx message golden vectors", () => {
  type Vector = { name: string; message: unknown; valid: boolean; b64?: string };
  for (const vector of txMessages.vectors as Vector[]) {
    it(`${vector.valid ? "accepts" : "rejects"} ${vector.name}`, () => {
      const parsed = msgFromUnknown(vector.message);
      expect(parsed !== null).toBe(vector.valid);
      if (!vector.valid) return;
      expect(parsed).toEqual(vector.message);
      const bytes = b64ToBytes(vector.b64!);
      expect(encodeDatagram(vector.message as Msg)).toEqual(bytes);
      expect(decodeDatagram(bytes)).toEqual(vector.message);
    });
  }
});

describe("data frame golden vectors", () => {
  for (const vector of dataFrames.vectors) {
    it(`encodes and chunk-decodes ${vector.name}`, () => {
      const payload = b64ToBytes(vector.payload_b64);
      const frame = b64ToBytes(vector.frame_b64);
      expect(encodeDataFrame(vector.header as FrameHeader, payload)).toEqual(frame);

      const reader = new DataFrameStreamReader();
      const decoded = [];
      for (let i = 0; i < frame.length; i += 3) {
        decoded.push(...reader.push(frame.subarray(i, Math.min(i + 3, frame.length))));
      }
      expect(decoded).toHaveLength(1);
      expect(decoded[0].header).toEqual(vector.header);
      expect(decoded[0].payload).toEqual(payload);
    });
  }
});

describe("manifest golden vectors", () => {
  type Vector = { name: string; data: unknown; manifest?: unknown; error?: string };
  for (const vector of manifests.vectors as Vector[]) {
    it(`handles ${vector.name}`, () => {
      if (vector.error !== undefined) {
        let code: string | null = null;
        try {
          parseManifest(vector.data);
        } catch (e) {
          if (e instanceof ManifestError) code = e.code;
        }
        expect(code).toBe(vector.error);
      } else {
        expect(parseManifest(vector.data)).toEqual(vector.manifest);
      }
    });
  }
});

describe("manifest/session message validation", () => {
  const manifest = {
    t: "manifest",
    robotId: "go2",
    manifest: {
      version: 1,
      channels: [{ ch: "odom", encoding: "pose.json.v1", delivery: "reliable", maxHz: 20 }],
    },
  };

  it("accepts well-formed messages", () => {
    expect(msgFromUnknown(manifest)).toEqual(manifest);
    // Bare = manifest-less robot; the nested manifest is opaque here
    // (structure belongs to parseManifest).
    expect(msgFromUnknown({ t: "manifest", robotId: "go2" })).not.toBeNull();
    expect(msgFromUnknown({ ...manifest, manifest: { alien: true } })).not.toBeNull();
    expect(msgFromUnknown({ t: "robots", robots: [{ id: "a", name: "A", model: "go2" }] }))
      .not.toBeNull();
  });

  it("rejects malformed or unknown messages", () => {
    expect(msgFromUnknown({ t: "nope" })).toBeNull();
    expect(msgFromUnknown({ t: "toString" })).toBeNull();
    expect(msgFromUnknown({ t: "manifest", robotId: "go2", manifest: null })).toBeNull();
    expect(msgFromUnknown({ t: "manifest", robotId: "go2", manifest: 7 })).toBeNull();
    expect(msgFromUnknown({ t: "manifest", manifest: { version: 1 } })).toBeNull();
    expect(msgFromUnknown({ t: "watch" })).toBeNull();
  });
});
