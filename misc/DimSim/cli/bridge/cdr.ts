/** Schema-driven ROS 2 CDR codecs shared by the browser and Deno bridge. */
import { parse } from "@foxglove/rosmsg";
import { MessageReader, MessageWriter } from "@foxglove/rosmsg2-serialization";
import { schemas } from "./cdr_schemas.ts";

const writers = new Map<string, MessageWriter>();
const readers = new Map<string, MessageReader>();
export function encodeCdr(type: string, value: Record<string, unknown>): Uint8Array {
  let writer = writers.get(type);
  if (!writer) {
    if (!schemas[type]) throw new Error(`Unknown CDR type: ${type}`);
    writer = new MessageWriter(parse(schemas[type], { ros2: true }));
    writers.set(type, writer);
  }
  return writer.writeMessage(value);
}
export function decodeCdr(type: string, data: Uint8Array): any {
  let reader = readers.get(type);
  if (!reader) {
    if (!schemas[type]) throw new Error(`Unknown CDR type: ${type}`);
    reader = new MessageReader(parse(schemas[type], { ros2: true }));
    readers.set(type, reader);
  }
  return reader.readMessage(data);
}
export function headerNow(frameId: string) {
  const now = Date.now();
  return { stamp: { sec: Math.floor(now / 1000), nanosec: (now % 1000) * 1_000_000 }, frame_id: frameId };
}
/** WebSocket envelope uses the existing LC02 channel framing, without UDP's size limit. */
export function sensorPacket(channel: string, value: Record<string, unknown>): Uint8Array {
  const type = channel.split("#")[1];
  const data = encodeCdr(type, value);
  const name = new TextEncoder().encode(channel);
  const packet = new Uint8Array(8 + name.length + 1 + data.length);
  new DataView(packet.buffer).setUint32(0, 0x4c433032, false);
  packet.set(name, 8);
  packet.set(data, 9 + name.length);
  return packet;
}
