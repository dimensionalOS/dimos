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
const counts = new Map();
for await (const message of reader.readMessages({ validateCrcs: true })) {
  const channel = reader.channelsById.get(message.channelId);
  assert.equal(channel.messageEncoding, 'cdr');
  const value = decoders.get(channel.schemaId).readMessage(message.data);
  const index = message.sequence;
  assert.equal(message.logTime - message.publishTime, 1000000n);
  if (channel.topic === '/telemetry') {
    assert.equal(value.sequence, index);
    assert.equal(value.reading.temperature, 20 + index / 10);
    assert.equal(value.label, 'synthetic');
    assert.deepEqual(Array.from(value.hops), [1, 2, 3]);
  } else if (channel.topic === '/camera/image') {
    assert.equal(value.width, 256);
    assert.equal(value.height, 192);
    assert.equal(value.data.length, 256 * 192 * 3);
    assert.equal(value.data[0], index * 8 % 256);
  } else if (channel.topic === '/camera/compressed') {
    assert.equal(value.format, 'png');
    assert.deepEqual(Array.from(value.data.slice(0, 8)), [137,80,78,71,13,10,26,10]);
  } else if (channel.topic === '/robot/pose') {
    assert.equal(value.pose.position.x, Math.cos(index / 5));
    assert.equal(value.pose.orientation.w, 1);
  } else if (channel.topic === '/tf') {
    assert.equal(value.transforms[0].child_frame_id, 'camera');
  }
  counts.set(channel.topic, (counts.get(channel.topic) ?? 0) + 1);
}
assert.equal(counts.size, 5);
for (const [topic, count] of counts) {
  assert.equal(count, 30);
  console.log(`Foxglove libraries decoded ${count} ${topic} messages from embedded definitions.`);
}
