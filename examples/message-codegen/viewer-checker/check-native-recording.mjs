import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import { McapIndexedReader } from '@mcap/core';
import { parse, fixupTypes } from '@foxglove/rosmsg';
import { MessageReader } from '@foxglove/rosmsg2-serialization';
import zstd from '@foxglove/wasm-zstd';

await zstd.isLoaded;
const bytes = await readFile(process.argv[2]);
const reader = await McapIndexedReader.Initialize({
  readable: {
    size: async () => BigInt(bytes.length),
    read: async (offset, size) => bytes.subarray(Number(offset), Number(offset + size)),
  },
  decompressHandlers: { zstd: (data, size) => zstd.decompress(data, Number(size)) },
});
assert.equal(reader.header.profile, 'ros2');
const decoders = new Map();
for (const schema of reader.schemasById.values()) {
  assert.equal(schema.encoding, 'ros2msg');
  const definitions = parse(new TextDecoder().decode(schema.data), { ros2: true, skipTypeFixup: true });
  definitions[0].name = schema.name.replace('/msg/', '/');
  fixupTypes(definitions);
  decoders.set(schema.id, new MessageReader(definitions));
}
let count = 0;
for await (const message of reader.readMessages({ validateCrcs: true })) {
  const channel = reader.channelsById.get(message.channelId);
  assert.equal(channel.messageEncoding, 'cdr');
  assert.equal(channel.topic, 'telemetry');
  const value = decoders.get(channel.schemaId).readMessage(message.data);
  assert.equal(value.application_note, `locally-added-field-${count}`);
  assert.equal(value.header.stamp.sec, 1700000000);
  assert.equal(value.header.stamp.nanosec, 123456789 + count);
  assert.equal(message.logTime, message.publishTime);
  console.log(`Foxglove decoded ${value.application_note} from embedded schema.`);
  count += 1;
}
assert.equal(count, 3);
console.log('PASS: native recorder custom messages need no viewer-side message package');
