import assert from "node:assert/strict";
import sharp from "sharp";
import type { Channel } from "../../../shared/protocol/streams.ts";
import { FramePainter } from "./painter.ts";
import { epochMs } from "../../../shared/protocol/time.ts";
import { test } from "node:test";

const channel = "dimos/cloud/PointCloud2" as Channel;

const small = new Float32Array([-1, -1, 0, 1, -1, 0, -1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0.5, 0, 0.5, 0, 0]);

const large = small.map((value) => value * 10);

const translated = small.map((value, index) => (index % 3 === 2 ? value : value + 100));

for (const { name, previous, current } of [
  { name: "growth", previous: small, current: large },
  { name: "translation", previous: small, current: translated },
  { name: "shrinkage", previous: large, current: small },
  { name: "an empty first frame", previous: new Float32Array(), current: translated },
]) {
  test(`a cloud fits its current frame after ${name}`, async () => {
    const painter = new FramePainter({ width: 101, height: 101 });
    await painter.paint({ kind: "cloud", channel, at: epochMs(0), xyz: previous });
    const png = await painter.paint({ kind: "cloud", channel, at: epochMs(1), xyz: current });
    const { data, info } = await sharp(png).raw().toBuffer({ resolveWithObject: true });
    assert.deepEqual([info.width, info.height, info.channels], [101, 101, 3]);

    const landmarks: [number, number][] = [];
    for (let row = 0; row < info.height; row++) {
      for (let column = 0; column < info.width; column++) {
        if (data[(row * info.width + column) * info.channels + 1] > 27) landmarks.push([row, column]);
      }
    }
    assert.deepEqual(landmarks, [
      [0, 0],
      [0, 100],
      [25, 50],
      [50, 50],
      [50, 75],
      [100, 0],
      [100, 100],
    ]);
  });
}
