/** CDR codecs for the generated schema artifact shared by browser and Deno.
 * cdr_schemas.json is emitted from the installed canonical DimOS message package;
 * the Python acceptance test verifies it against that package's schema registry.
 */
import { parse } from "@foxglove/rosmsg";
import { MessageReader, MessageWriter } from "@foxglove/rosmsg2-serialization";
import schemas from "./cdr_schemas.json" with { type: "json" };

const writers = new Map<string, MessageWriter>();
const readers = new Map<string, MessageReader>();
function definition(type: string): string {
  const value = schemas[type as keyof typeof schemas];
  if (!value) throw new Error(`Unknown CDR type: ${type}`);
  return value;
}
export function channel(topic: string, type: string): string {
  definition(type);
  return `${topic.replace(/^\/+/, "")}/cdr/${type}`;
}
export function encodeCdr(
  type: string,
  value: Record<string, unknown>,
): Uint8Array {
  let writer = writers.get(type);
  if (!writer) {
    writer = new MessageWriter(parse(definition(type), { ros2: true }));
    writers.set(type, writer);
  }
  return writer.writeMessage(value);
}
export function decodeCdr(
  type: string,
  data: Uint8Array,
): Record<string, unknown> {
  if (
    data.length < 4 || data[0] !== 0 || data[1] > 1 || data[2] !== 0 ||
    data[3] !== 0
  ) {
    throw new Error("invalid XCDR1 header");
  }
  let reader = readers.get(type);
  if (!reader) {
    reader = new MessageReader(parse(definition(type), { ros2: true }));
    readers.set(type, reader);
  }
  const value = reader.readMessage(data);
  if (reader.lastReadByteLength() !== data.length) {
    throw new Error("trailing CDR bytes");
  }
  if (value === null || typeof value !== "object" || Array.isArray(value)) {
    throw new Error("invalid CDR message object");
  }
  return value as Record<string, unknown>;
}
export function headerNow(frameId: string) {
  const now = Date.now();
  return {
    stamp: { sec: Math.floor(now / 1000), nanosec: (now % 1000) * 1_000_000 },
    frame_id: frameId,
  };
}
/** Existing short LCM packet framing over WS; UDP fragmentation stays in the relay. */
export function sensorPacket(
  topic: string,
  value: Record<string, unknown>,
): Uint8Array {
  const split = topic.indexOf("/cdr/");
  if (split < 0) throw new Error("missing CDR channel type");
  const data = encodeCdr(topic.slice(split + 5), value);
  const name = new TextEncoder().encode(topic);
  const packet = new Uint8Array(8 + name.length + 1 + data.length);
  new DataView(packet.buffer).setUint32(0, 0x4c433032, false);
  packet.set(name, 8);
  packet.set(data, 9 + name.length);
  return packet;
}
