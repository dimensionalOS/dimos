import assert from "node:assert/strict";
import sharp from "sharp";
import { cloudFrame, gridFrame, imageFrame, topDownCamera } from "./frames.ts";
import { test } from "node:test";

async function pixels(png: Buffer): Promise<{ data: Buffer; width: number; height: number; channels: number }> {
  const { data, info } = await sharp(png).raw().toBuffer({ resolveWithObject: true });
  return { data, width: info.width, height: info.height, channels: info.channels };
}

function at(image: { data: Buffer; width: number; channels: number }, row: number, column: number): number[] {
  const offset = (row * image.width + column) * image.channels;
  return [...image.data.subarray(offset, offset + image.channels)];
}

test("a cloud is painted from above, +x right and +y up, brighter where it is higher", async () => {
  const xyz = new Float32Array([-1, -1, 0, 1, 1, 2, 0, 0, 1, Number.NaN, 0, 0]);
  const camera = topDownCamera(xyz);
  assert.deepEqual(camera, { center: [0, 0, 1], extent: 2, floor: 0, height: 2, yaw: 0, zoom: 1 });
  const image = await pixels(await cloudFrame(xyz, camera, { width: 101, height: 101 }));
  assert.deepEqual([image.width, image.height, image.channels], [101, 101, 3]);
  assert.deepEqual(at(image, 50, 50), [72, 155, 140]);
  assert.deepEqual(at(image, 0, 100), [106, 229, 207]);
  assert.deepEqual(at(image, 100, 0), [37, 80, 72]);
  assert.deepEqual(at(image, 0, 0), [16, 27, 33]);
});

test("a grid is a grey PNG with +y up, scaled to fit the box", async () => {
  const png = await gridFrame(new Int8Array([-1, 0, 100, 50]), 2, 2, { width: 4, height: 8 });
  assert.equal((await sharp(png).metadata()).channels, 1);
  const image = await pixels(png);
  assert.deepEqual([image.width, image.height], [4, 4]);
  const grey = (row: number, column: number): number => at(image, row, column)[0];
  assert.deepEqual([grey(0, 0), grey(0, 3), grey(3, 0), grey(3, 3)], [0, 127, 128, 255]);
});

test("an image is shrunk to fit inside the box", async () => {
  const jpeg = await sharp({ create: { width: 64, height: 48, channels: 3, background: "#406080" } })
    .jpeg()
    .toBuffer();
  const { format, width, height } = await sharp(await imageFrame(jpeg, { width: 32, height: 32 })).metadata();
  assert.deepEqual([format, width, height], ["png", 32, 24]);
});
