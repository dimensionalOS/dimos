// Decode and re-encode native recorder samples with Foxglove's actual ROS 2 libraries.
import { deepStrictEqual, strictEqual } from "node:assert";
import { parse } from "npm:@foxglove/rosmsg@5.0.5";
import {
  MessageReader,
  MessageWriter,
} from "npm:@foxglove/rosmsg2-serialization@3.1.2";

const samples: { topic: string; schema: string; payload: string }[] = JSON
  .parse(
    Deno.readTextFileSync(Deno.args[0]),
  );
for (const sample of samples) {
  const definitions = parse(Deno.readTextFileSync(sample.schema), {
    ros2: true,
  });
  const payload = Deno.readFileSync(sample.payload);
  const reader = new MessageReader(definitions);
  const decoded = reader.readMessage(payload);
  strictEqual(reader.lastReadByteLength(), payload.length, sample.topic);
  deepStrictEqual(
    new MessageWriter(definitions).writeMessage(decoded),
    payload,
    sample.topic,
  );
}
console.log(JSON.stringify(samples.map((sample) => sample.topic)));
