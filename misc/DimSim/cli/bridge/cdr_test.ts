import { strict as assert } from "node:assert";
import { encodeCdr, decodeCdr, sensorPacket } from "./cdr.ts";
import { decodePacket } from "../vendor/lcm/transport.ts";

const header = { stamp: { sec: 42, nanosec: 123456789 }, frame_id: "camera_optical" };
Deno.test("CDR image preserves raw pixels, source stamp and large WS envelope", () => {
  const pixels = new Uint8Array(640 * 288 * 4).fill(127);
  pixels[0] = 255;
  const packet = sensorPacket("/color_image#sensor_msgs/msg/Image", {
    header, height: 288, width: 640, encoding: "rgba8", is_bigendian: 0,
    step: 640 * 4, data: pixels,
  });
  const framed = decodePacket(packet);
  assert(framed?.type === "small");
  assert.equal(framed.channel, "/color_image#sensor_msgs/msg/Image");
  assert.deepEqual(Array.from(framed.data.slice(0, 4)), [0, 1, 0, 0]);
  const image = decodeCdr("sensor_msgs/msg/Image", framed.data);
  assert.deepEqual(image.header, header);
  assert.equal(image.encoding, "rgba8");
  assert.equal(image.step, 2560);
  assert.deepEqual(image.data, pixels);
});
Deno.test("CDR command decoder preserves all six velocity components", () => {
  const command = { linear: { x: 1.5, y: -2, z: 3 }, angular: { x: 4, y: 5, z: -0.25 } };
  assert.deepEqual(decodeCdr("geometry_msgs/msg/Twist", encodeCdr("geometry_msgs/msg/Twist", command)), command);
});
Deno.test("CDR codec rejects missing schemas and truncated payloads", () => {
  assert.throws(() => encodeCdr("unknown/msg/Type", {}), /Unknown CDR type/);
  assert.throws(() => decodeCdr("geometry_msgs/msg/Twist", new Uint8Array([0, 1, 0, 0])));
});
