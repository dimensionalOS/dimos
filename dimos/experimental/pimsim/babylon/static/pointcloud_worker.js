// Copyright 2025-2026 Dimensional Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

import { sensor_msgs } from "https://esm.sh/jsr/@dimos/msgs@0.1.4";

let droppedFrames = 0;

function turboColor(t) {
  const x = Math.max(0, Math.min(1, t));
  const r = 34.61 + x * (1172.33 - x * (10793.56 - x * (33300.12 - x * (38394.49 - x * 14825.05))));
  const g = 23.31 + x * (557.33 + x * (1225.33 - x * (3574.96 - x * (1073.77 + x * 707.56))));
  const b = 27.2 + x * (3211.1 - x * (15327.97 - x * (27814 - x * (22569.18 - x * 6838.66))));
  return [
    Math.max(0, Math.min(255, r)),
    Math.max(0, Math.min(255, g)),
    Math.max(0, Math.min(255, b)),
  ];
}

function findPointCloudField(fields, name) {
  for (const f of fields) if (f.name === name) return f;
  return null;
}

function pointCloud2ToRenderPayload(msg) {
  const fx = findPointCloudField(msg.fields, "x");
  const fy = findPointCloudField(msg.fields, "y");
  const fz = findPointCloudField(msg.fields, "z");
  if (!fx || !fy || !fz) return null;

  const step = msg.point_step;
  const raw = msg.data instanceof Uint8Array ? msg.data : Uint8Array.from(msg.data || []);
  if (!step || !raw.byteLength) return null;
  const count = Math.floor(raw.byteLength / step);
  if (count === 0) return null;

  const view = new DataView(raw.buffer, raw.byteOffset, raw.byteLength);
  const littleEndian = !msg.is_bigendian;
  const positions = new Float32Array(count * 3);
  const colors = new Float32Array(count * 4);
  const frgb = findPointCloudField(msg.fields, "rgb");

  let zMin = Infinity;
  let zMax = -Infinity;
  for (let i = 0; i < count; i += 1) {
    const o = i * step;
    const x = view.getFloat32(o + fx.offset, littleEndian);
    const y = view.getFloat32(o + fy.offset, littleEndian);
    const z = view.getFloat32(o + fz.offset, littleEndian);
    const pi = i * 3;
    const ci = i * 4;
    if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) {
      positions[pi + 0] = 0;
      positions[pi + 1] = 0;
      positions[pi + 2] = 0;
      colors[ci + 0] = 0;
      colors[ci + 1] = 0;
      colors[ci + 2] = 0;
      colors[ci + 3] = 0;
      continue;
    }
    positions[pi + 0] = x;
    positions[pi + 1] = y;
    positions[pi + 2] = z;
    colors[ci + 3] = 0.94;
    if (z < zMin) zMin = z;
    if (z > zMax) zMax = z;
  }

  if (frgb) {
    for (let i = 0; i < count; i += 1) {
      const o = i * step + frgb.offset;
      const ci = i * 4;
      colors[ci + 0] = view.getUint8(o + 2) / 255;
      colors[ci + 1] = view.getUint8(o + 1) / 255;
      colors[ci + 2] = view.getUint8(o + 0) / 255;
    }
  } else {
    const denom = (zMax - zMin) || 1.0;
    for (let i = 0; i < count; i += 1) {
      const t = (positions[i * 3 + 2] - zMin) / denom;
      const [r, g, b] = turboColor(t);
      const ci = i * 4;
      colors[ci + 0] = r / 255;
      colors[ci + 1] = g / 255;
      colors[ci + 2] = b / 255;
    }
  }

  return { count, positions, colors };
}

self.onmessage = (event) => {
  const message = event.data || {};
  if (message.type === "dropped") {
    droppedFrames += Number(message.count || 0);
    return;
  }
  if (message.type !== "payload") return;

  try {
    const payload = new Uint8Array(message.buffer, message.byteOffset, message.byteLength);
    const decodeStart = performance.now();
    const msg = sensor_msgs.PointCloud2.decode(payload);
    const decodeMs = performance.now() - decodeStart;

    const buildStart = performance.now();
    const renderPayload = pointCloud2ToRenderPayload(msg);
    const buildMs = performance.now() - buildStart;
    if (!renderPayload) {
      self.postMessage({ type: "empty" });
      return;
    }

    self.postMessage(
      {
        type: "pointcloud",
        count: renderPayload.count,
        positions: renderPayload.positions.buffer,
        colors: renderPayload.colors.buffer,
        stats: { decodeMs, buildMs, dropped: droppedFrames },
      },
      [renderPayload.positions.buffer, renderPayload.colors.buffer],
    );
  } catch (error) {
    self.postMessage({
      type: "error",
      message: error instanceof Error ? `${error.name}: ${error.message}` : String(error),
    });
  }
};
