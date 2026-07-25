import { assertEquals, assertThrows } from "@std/assert";
import {
  ControlFrameReader,
  DataFrameReader,
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

Deno.test("data frame reader completes at exact byte count, split anywhere", () => {
  const v = dataFixture.vectors.find((v) => v.name === "image_latest_meta")!;
  const frame = fromB64(v.frame_b64);
  for (let split = 0; split <= frame.length; split++) {
    const reader = new DataFrameReader();
    const first = reader.push(frame.subarray(0, split));
    const second = reader.push(frame.subarray(split));
    if (split < frame.length) {
      assertEquals(first, null, `complete before full frame at split ${split}`);
    }
    const out = first ?? second;
    assertEquals(out !== null, true, `incomplete after full frame at split ${split}`);
    assertEquals(out!.header, v.header as FrameHeader);
    assertEquals(out!.payload, fromB64(v.payload_b64));
  }
});

Deno.test("data frame reader ignores bytes past the frame (no EOF dependence)", () => {
  const v = dataFixture.vectors.find((v) => v.name === "odom_reliable")!;
  const frame = fromB64(v.frame_b64);
  const padded = new Uint8Array(frame.length + 32);
  padded.set(frame, 0);
  const out = new DataFrameReader().push(padded);
  assertEquals(out !== null, true);
  assertEquals(out!.header, v.header as FrameHeader);
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
});

Deno.test("frameHeaderFromUnknown validates the header shape", () => {
  const ok = { ch: "cam", seq: 1, ts: 2.5, delivery: "latest" };
  assertEquals(frameHeaderFromUnknown(ok), ok as FrameHeader);
  assertEquals(frameHeaderFromUnknown({ ...ok, delivery: "bogus" }), null);
  assertEquals(frameHeaderFromUnknown({ ...ok, seq: "1" }), null);
  assertEquals(frameHeaderFromUnknown({ ...ok, ch: 5 }), null);
  assertEquals(frameHeaderFromUnknown({ ...ok, meta: 7 }), null); // meta not an object
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
