import { assert, assertEquals, assertThrows } from "@std/assert";
import {
  CONTROL_CHANNEL,
  ControlFrameReader,
  type DataFrame,
  DataFrameStreamError,
  DataFrameStreamReader,
  decodeDataFrame,
  decodeDatagram,
  encodeControlFrame,
  encodeDataFrame,
  encodeDatagram,
  type FrameHeader,
  frameHeaderFromUnknown,
  MAX_DATA_FRAME_BYTES,
  MAX_HEADER_LEN,
  type Msg,
  msgFromUnknown,
  peekDataFrameLengths,
  PROTOCOL_VERSION,
  RESERVED_CHANNEL_PREFIX,
} from "./protocol.ts";
import controlFixture from "./fixtures/control_frames.json" with { type: "json" };
import datagramFixture from "./fixtures/datagrams.json" with { type: "json" };
import dataFixture from "./fixtures/data_frames.json" with { type: "json" };

function fromB64(s: string): Uint8Array {
  return Uint8Array.from(atob(s), (c) => c.charCodeAt(0));
}

Deno.test("control frames match golden vectors byte-exactly", () => {
  for (const v of controlFixture.vectors) {
    assertEquals(encodeControlFrame(v.message as Msg), fromB64(v.b64), v.name);
  }
});

Deno.test("control frame reader decodes the golden vectors", () => {
  const reader = new ControlFrameReader();
  const all = controlFixture.vectors.flatMap((v) => [...fromB64(v.b64)]);
  const msgs = reader.push(new Uint8Array(all));
  assertEquals(msgs, controlFixture.vectors.map((v) => v.message as Msg));
});

Deno.test("control frame reader survives every split point", () => {
  const all = new Uint8Array(controlFixture.vectors.flatMap((v) => [...fromB64(v.b64)]));
  const expected = controlFixture.vectors.map((v) => v.message as Msg);
  for (let split = 0; split <= all.length; split++) {
    const reader = new ControlFrameReader();
    const msgs = [...reader.push(all.subarray(0, split)), ...reader.push(all.subarray(split))];
    assertEquals(msgs, expected, `split at ${split}`);
  }
});

Deno.test("control frame reader rejects absurd lengths", () => {
  const bad = new Uint8Array(4);
  new DataView(bad.buffer).setUint32(0, MAX_HEADER_LEN + 1, true);
  assertThrows(() => new ControlFrameReader().push(bad));
});

Deno.test("control frame reader rejects zero-length frames", () => {
  // An encoder can never produce one, so treat it as framing corruption
  // instead of warn-per-4-bytes on a hostile chunk of zeros.
  assertThrows(() => new ControlFrameReader().push(new Uint8Array(4)));
});

Deno.test("datagrams match golden vectors and round-trip", () => {
  for (const v of datagramFixture.vectors) {
    assertEquals(encodeDatagram(v.message as Msg), fromB64(v.b64), v.name);
    assertEquals(decodeDatagram(fromB64(v.b64)), v.message as Msg, v.name);
  }
});

Deno.test("datagram decode returns null for junk", () => {
  assertEquals(decodeDatagram(new Uint8Array([0xff, 0x00, 0x80])), null);
  assertEquals(decodeDatagram(new TextEncoder().encode("[1,2]")), null);
  assertEquals(decodeDatagram(new TextEncoder().encode('{"x":1}')), null);
});

Deno.test("the control_hello vector's payload is the hello datagram encoding", () => {
  // @control payloads reuse the datagram encoding: the vector's payload must
  // decode to the same message the hello_robot datagram vector pins.
  assert(CONTROL_CHANNEL.startsWith(RESERVED_CHANNEL_PREFIX));
  const control = dataFixture.vectors.find((v) => v.name === "control_hello")!;
  assertEquals(control.header.ch, CONTROL_CHANNEL);
  const hello = datagramFixture.vectors.find((v) => v.name === "hello_robot")!;
  assertEquals(decodeDatagram(fromB64(control.payload_b64)), hello.message as Msg);
});

Deno.test("data frames match golden vectors byte-exactly", () => {
  for (const v of dataFixture.vectors) {
    const frame = encodeDataFrame(v.header as FrameHeader, fromB64(v.payload_b64));
    assertEquals(frame, fromB64(v.frame_b64), v.name);
  }
});

Deno.test("data frame decode round-trips the golden vectors", () => {
  for (const v of dataFixture.vectors) {
    const { header, payload } = decodeDataFrame(fromB64(v.frame_b64));
    assertEquals(header, v.header as FrameHeader, v.name);
    assertEquals(payload, fromB64(v.payload_b64), v.name);
  }
});

Deno.test("data frame stream reader completes at exact byte count, split anywhere", () => {
  const v = dataFixture.vectors.find((v) => v.name === "image_latest_meta")!;
  const frame = fromB64(v.frame_b64);
  for (let split = 0; split <= frame.length; split++) {
    const reader = new DataFrameStreamReader();
    const first = reader.push(frame.subarray(0, split));
    const second = reader.push(frame.subarray(split));
    if (split < frame.length) {
      assertEquals(first, [], `complete before full frame at split ${split}`);
    }
    const out = [...first, ...second];
    assertEquals(out.length, 1, `frames after full push at split ${split}`);
    assertEquals(out[0].header, v.header as FrameHeader);
    assertEquals(out[0].payload, fromB64(v.payload_b64));
  }
});

Deno.test("data frame stream reader parses back-to-back frames (persistent stream)", () => {
  const parts = dataFixture.vectors.map((v) => fromB64(v.frame_b64));
  const all = new Uint8Array(parts.reduce((n, f) => n + f.length, 0));
  let off = 0;
  for (const part of parts) {
    all.set(part, off);
    off += part.length;
  }
  const whole = new DataFrameStreamReader().push(all);
  assertEquals(whole.length, dataFixture.vectors.length);
  whole.forEach((frame, i) => {
    assertEquals(frame.header, dataFixture.vectors[i].header as FrameHeader);
    assertEquals(frame.payload, fromB64(dataFixture.vectors[i].payload_b64));
  });

  const trickle = new DataFrameStreamReader();
  const out = [];
  for (const byte of all) out.push(...trickle.push(Uint8Array.of(byte)));
  assertEquals(out.length, dataFixture.vectors.length);
});

Deno.test("data frame stream reader throws on garbage between frames", () => {
  const v = dataFixture.vectors.find((v) => v.name === "odom_reliable")!;
  const reader = new DataFrameStreamReader();
  assertEquals(reader.push(fromB64(v.frame_b64)).length, 1);
  // Framing is unrecoverable mid-stream; the caller drops the stream.
  assertThrows(() => reader.push(new Uint8Array(32)));
});

Deno.test("data frame stream reader assembles a large fragmented frame with one copy", () => {
  const payload = new Uint8Array(8 * 1024 * 1024);
  for (let i = 0; i < payload.length; i++) payload[i] = i & 0xff;
  const header: FrameHeader = { ch: "cam", seq: 1, ts: 0.5, delivery: "latest" };
  const frame = encodeDataFrame(header, payload);
  const reader = new DataFrameStreamReader();
  const out: DataFrame[] = [];
  for (let off = 0; off < frame.length; off += 64 * 1024) {
    out.push(...reader.push(frame.subarray(off, Math.min(off + 64 * 1024, frame.length))));
  }
  assertEquals(out.length, 1);
  assertEquals(out[0].header, header);
  assertEquals(out[0].payload.byteLength, payload.byteLength);
  let mismatch = -1;
  for (let i = 0; i < payload.length; i++) {
    if (out[0].payload[i] !== payload[i]) {
      mismatch = i;
      break;
    }
  }
  assertEquals(mismatch, -1);
});

Deno.test("data frame stream reader surfaces corruption with the decoded batch", () => {
  const v = dataFixture.vectors.find((v) => v.name === "odom_reliable")!;
  const good = fromB64(v.frame_b64);
  const badBody = new TextEncoder().encode("{not json");
  const bad = new Uint8Array(8 + badBody.length);
  new DataView(bad.buffer).setUint32(0, badBody.length, true);
  bad.set(badBody, 8);
  const reader = new DataFrameStreamReader();
  const err = assertThrows(
    () => reader.push(new Uint8Array([...good, ...bad])),
    DataFrameStreamError,
  );
  // The valid frame preceding the corrupt one is delivered, not dropped.
  assertEquals(err.frames.length, 1);
  assertEquals(err.frames[0].header, v.header as FrameHeader);
  assertEquals(err.frames[0].payload, fromB64(v.payload_b64));
  // Poisoned: further input keeps throwing with an empty batch.
  const again = assertThrows(() => reader.push(good), DataFrameStreamError);
  assertEquals(again.frames, []);
});

Deno.test("data frame stream reader handles a prefix split across chunk boundaries", () => {
  const v = dataFixture.vectors.find((v) => v.name === "image_latest_meta")!;
  const frame = fromB64(v.frame_b64);
  const reader = new DataFrameStreamReader();
  // 3-byte chunks force the 8-byte length prefix to straddle chunks.
  const out: DataFrame[] = [];
  for (let off = 0; off < frame.length; off += 3) {
    out.push(...reader.push(frame.subarray(off, Math.min(off + 3, frame.length))));
  }
  assertEquals(out.length, 1);
  assertEquals(out[0].payload, fromB64(v.payload_b64));
  assert(reader.push(new Uint8Array(0)).length === 0);
});

Deno.test("peek and decode guard against truncation and absurd headers", () => {
  assertEquals(peekDataFrameLengths(new Uint8Array(7)), null);
  const v = dataFixture.vectors[0];
  const frame = fromB64(v.frame_b64);
  assertThrows(() => decodeDataFrame(frame.subarray(0, frame.length - 1)));
  const bad = new Uint8Array(8);
  new DataView(bad.buffer).setUint32(0, MAX_HEADER_LEN + 1, true);
  assertThrows(() => peekDataFrameLengths(bad));
});

// ---------- validation policy (mirror of Python's; see protocol.py) ----------

Deno.test("msgFromUnknown validates shape and rejects unknown/malformed", () => {
  assertEquals(msgFromUnknown({ t: "ping", n: 1, ts: 2.5 }), { t: "ping", n: 1, ts: 2.5 });
  assertEquals(msgFromUnknown({ t: "bogus", n: 1 }), null); // unknown type
  assertEquals(msgFromUnknown({ t: "ping", n: "1", ts: 2.5 }), null); // n not a number
  assertEquals(msgFromUnknown({ t: "ping", ts: 2.5 }), null); // missing n
  assertEquals(msgFromUnknown({ t: "hello", v: 1 }), null); // missing role
  assertEquals(msgFromUnknown(null), null);
  assertEquals(msgFromUnknown([1, 2]), null);
  // Prototype-chain keys must not resolve through Object.prototype (the
  // mirrored protocol.py rejects these; test_protocol.py asserts the same).
  assertEquals(msgFromUnknown({ t: "toString" }), null);
  assertEquals(msgFromUnknown({ t: "constructor" }), null);
  assertEquals(msgFromUnknown({ t: "hasOwnProperty" }), null);
});

Deno.test("msgFromUnknown validates nested session-message shapes", () => {
  const robot = { id: "go2-lab", name: "Go2 Lab", model: "unitree-go2" };
  const spec = { ch: "odom", encoding: "pose.json.v1", delivery: "reliable", maxHz: 20.5 };
  // hello stays valid without the optional robot/manifest (viewer form).
  assertEquals(msgFromUnknown({ t: "hello", v: 1, role: "viewer" }) !== null, true);
  const full = {
    t: "hello",
    v: 1,
    role: "robot",
    robot,
    manifest: { version: 1, channels: [spec] },
  };
  assertEquals(msgFromUnknown(full) !== null, true);
  // Optional means absent-or-valid: explicit null is rejected.
  assertEquals(msgFromUnknown({ t: "hello", v: 1, role: "robot", robot: null }), null);
  assertEquals(msgFromUnknown({ ...full, robot: { id: 5, name: "x", model: "m" } }), null);
  // The manifest is opaque at the transport layer: any record passes here
  // (structure is parseManifest's job, so a garbage manifest gets a proper
  // invalid_manifest reply instead of a silent drop), but null and
  // non-records are still protocol violations.
  assertEquals(msgFromUnknown({ ...full, manifest: { channels: "garbage" } }) !== null, true);
  assertEquals(msgFromUnknown({ ...full, manifest: null }), null);
  assertEquals(msgFromUnknown({ ...full, manifest: 5 }), null);
  assertEquals(msgFromUnknown({ ...full, manifest: [spec] }), null);
  assertEquals(msgFromUnknown({ t: "robots", robots: [robot] }) !== null, true);
  assertEquals(msgFromUnknown({ t: "robots", robots: {} }), null);
  assertEquals(msgFromUnknown({ t: "robots", robots: [{ id: "a", name: "b" }] }), null);
  assertEquals(msgFromUnknown({ t: "robots" }), null);
  // ManifestMsg nests the manifest; bare = manifest-less robot.
  assertEquals(msgFromUnknown({ t: "manifest", robotId: "r" }) !== null, true);
  assertEquals(
    msgFromUnknown({ t: "manifest", robotId: "r", manifest: { version: 1, channels: [spec] } }) !==
      null,
    true,
  );
  assertEquals(msgFromUnknown({ t: "manifest", robotId: "r", manifest: null }), null);
  assertEquals(msgFromUnknown({ t: "manifest", robotId: "r", manifest: 7 }), null);
  assertEquals(msgFromUnknown({ t: "manifest", manifest: { channels: [spec] } }), null);
  assertEquals(msgFromUnknown({ t: "watch" }), null);
  assertEquals(msgFromUnknown({ t: "subs", chs: ["a", "b"], n: 1 }) !== null, true);
  assertEquals(msgFromUnknown({ t: "subs", chs: ["a", 5], n: 1 }), null);
  assertEquals(msgFromUnknown({ t: "subs", chs: ["a"] }), null);
  // Teleop gen mirrors hello.robot: absent ok, null/non-number rejected.
  const twist = { t: "twist", vx: 0.5, vy: 0, wz: 0, seq: 1, ts: 2.5 };
  assertEquals(msgFromUnknown(twist) !== null, true);
  assertEquals(msgFromUnknown({ ...twist, gen: 3 }) !== null, true);
  assertEquals(msgFromUnknown({ ...twist, gen: null }), null);
  assertEquals(msgFromUnknown({ ...twist, gen: "1" }), null);
  assertEquals(msgFromUnknown({ t: "teleop_start", gen: 3 }) !== null, true);
  assertEquals(msgFromUnknown({ t: "teleop_stop", gen: null }), null);
});

Deno.test("frameHeaderFromUnknown validates the header shape", () => {
  const ok = { ch: "cam", seq: 1, ts: 2.5, delivery: "latest" };
  assertEquals(frameHeaderFromUnknown(ok), ok as FrameHeader);
  assertEquals(frameHeaderFromUnknown({ ...ok, delivery: "bogus" }), null);
  assertEquals(frameHeaderFromUnknown({ ...ok, seq: "1" }), null);
  assertEquals(frameHeaderFromUnknown({ ...ok, ch: 5 }), null);
  assertEquals(frameHeaderFromUnknown({ ...ok, meta: 7 }), null); // meta not an object
  // ch is bounded like manifest channel ids (64): only unroutable undeclared
  // names are dropped.
  const atBound = { ...ok, ch: "c".repeat(64) };
  assertEquals(frameHeaderFromUnknown(atBound), atBound as FrameHeader);
  assertEquals(frameHeaderFromUnknown({ ...ok, ch: "c".repeat(65) }), null);
});

Deno.test("decodeDataFrame throws on an invalid header", () => {
  const frame = encodeDataFrame(
    { ch: "cam", seq: 1, ts: 2.5, delivery: "bogus" } as unknown as FrameHeader,
    new Uint8Array([1, 2, 3]),
  );
  assertThrows(() => decodeDataFrame(frame));
});

Deno.test("control reader drops an invalid message but keeps its valid neighbors", () => {
  const rawFrame = (bodyStr: string): Uint8Array => {
    const body = new TextEncoder().encode(bodyStr);
    const out = new Uint8Array(4 + body.length);
    new DataView(out.buffer).setUint32(0, body.length, true);
    out.set(body, 4);
    return out;
  };
  const hello = encodeControlFrame({ t: "hello", v: PROTOCOL_VERSION, role: "viewer" });
  const ping = encodeControlFrame({ t: "ping", n: 3, ts: 4.5 });
  const junk = rawFrame("null"); // well-framed, invalid message
  const stream = new Uint8Array([...hello, ...junk, ...ping]);
  const msgs = new ControlFrameReader().push(stream);
  assertEquals(msgs, [
    { t: "hello", v: PROTOCOL_VERSION, role: "viewer" },
    { t: "ping", n: 3, ts: 4.5 },
  ]);
});

Deno.test("control reader drops a frame with invalid UTF-8, framing intact", () => {
  const badBody = new Uint8Array([0xff, 0xfe, 0xfd]); // not valid UTF-8
  const junk = new Uint8Array(4 + badBody.length);
  new DataView(junk.buffer).setUint32(0, badBody.length, true);
  junk.set(badBody, 4);
  const ping = encodeControlFrame({ t: "ping", n: 9, ts: 1.5 });
  const msgs = new ControlFrameReader().push(new Uint8Array([...junk, ...ping]));
  assertEquals(msgs, [{ t: "ping", n: 9, ts: 1.5 }]);
});

Deno.test("peek rejects a frame whose total exceeds MAX_DATA_FRAME_BYTES", () => {
  const bad = new Uint8Array(8);
  const dv = new DataView(bad.buffer);
  dv.setUint32(0, 2, true); // small header
  dv.setUint32(4, MAX_DATA_FRAME_BYTES, true); // payload pushes total over the cap
  assertThrows(() => peekDataFrameLengths(bad));
});
