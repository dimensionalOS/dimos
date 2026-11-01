import assert from "node:assert/strict";
import sharp from "sharp";
import { FileRenderer } from "#dimos/agents/shared/render/render.ts";
import { execFile } from "node:child_process";
import { existsSync } from "node:fs";
import { fitCamera, paintCloud, parseCloud, projectPoint } from "#dimos/agents/shared/render/points.ts";
import { join } from "node:path";
import { mkdtemp, readFile, rm } from "node:fs/promises";
import { promisify } from "node:util";
import { test } from "node:test";
import { tmpdir } from "node:os";

const python = join(import.meta.dirname, "../../../../.venv/bin/python");

test("point presentation validates geometry without mutating the source", async () => {
  const json = {
    points: [
      [0, 0, 0],
      [1, 0, 2],
      [1, 2, 3],
    ],
    frame: "map",
  };
  const before = JSON.stringify(json);
  const cloud = parseCloud(json);
  const png = await paintCloud(cloud, fitCamera(cloud), { width: 900, height: 448 });
  assert.equal(JSON.stringify(json), before);
  const { width, height } = await sharp(png).metadata();
  assert.deepEqual([width, height], [900, 448]);
  assert.throws(
    () => parseCloud({ points: [[NaN, 1, 2]] }),
    /^Error: not a point cloud: \/points\/0\/0 must be number$/,
  );
  assert.throws(
    () => parseCloud({ points: [[0, 1, 2]], selectedIndices: [1] }),
    /^Error: selected index 1 is outside the 1 points$/,
  );
  assert.throws(() => parseCloud({ points: [[0, 1, 2]], colors: [] }), /^Error: 0 colors for 1 points$/);
});

test("clouds fitted together share one camera, and a painted point lands where it projects", async () => {
  const a = parseCloud({
    points: [
      [0, 0, 0],
      [1, 2, 3],
    ],
  });
  const b = parseCloud({
    points: [
      [1, 2, 3],
      [8, 4, 2],
    ],
    colors: [
      [255, 0, 0],
      [0, 0, 255],
    ],
  });
  const camera = fitCamera(a, b);
  assert.deepEqual(camera, { center: [4, 2, 1.5], extent: 8, floor: 0, height: 3, yaw: -0.58, zoom: 1 });
  const area = { width: 900, height: 400 };
  assert.deepEqual(projectPoint([4, 2, 1.5], camera, area), [450, 200]);
  const [x, y] = projectPoint([8, 4, 2], camera, area);
  assert.deepEqual([Math.round(x), Math.round(y)], [580, 178]);

  const { data, info } = await sharp(await paintCloud(b, camera, { width: 900, height: 448 }))
    .raw()
    .toBuffer({ resolveWithObject: true });
  const at = (px: number, py: number): number[] => {
    const offset = (Math.floor(py) * info.width + Math.floor(px)) * info.channels;
    return [...data.subarray(offset, offset + 3)];
  };
  const [blueR, blueG, blueB] = at(x, y);
  assert(blueB > 150 && blueR < 60 && blueG < 60, `blue point painted blue, got ${[blueR, blueG, blueB]}`);
  const [redR, redG, redB] = at(...projectPoint([1, 2, 3], camera, area));
  assert(redR > 150 && redG < 60 && redB < 60, `red point painted red, got ${[redR, redG, redB]}`);
});

test(
  "renders the XYZ JSON that HarnessRuntime exports for a PointCloud2",
  { skip: existsSync(python) ? false : "needs the checkout's .venv", timeout: 120_000 },
  async (t) => {
    const dir = await mkdtemp(join(tmpdir(), "render-export-"));
    t.after(() => rm(dir, { recursive: true, force: true }));
    const script = [
      "import json, sys",
      "from pathlib import Path",
      "import numpy as np",
      "import open3d as o3d",
      "from dimos.agents.harness.runtime.lib.exports import export",
      "from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2",
      "grid = np.array([[x, y, 0.1 * (x + y)] for x in range(30) for y in range(20)], dtype=float) * 0.05",
      "cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(grid))",
      "cloud.colors = o3d.utility.Vector3dVector(np.tile([[1.0, 0.25, 0.0]], (len(grid), 1)))",
      "message = PointCloud2(pointcloud=cloud, frame_id='map', ts=1700000000.25)",
      "print(json.dumps(export(message, 'probe', Path(sys.argv[1])).to_json()))",
    ].join("\n");
    const { stdout } = await promisify(execFile)(python, ["-I", "-c", script, join(dir, "exports")], {
      cwd: dir,
      env: {
        HOME: dir,
        ROBOT_IP: "",
        XDG_STATE_HOME: join(dir, "state"),
        XDG_CONFIG_HOME: join(dir, "config"),
        XDG_CACHE_HOME: join(dir, "cache"),
        XDG_RUNTIME_DIR: join(dir, "runtime"),
      },
    });
    const path = join(dir, "exports", "probe.json");
    const original = await readFile(path);
    const exported: unknown = JSON.parse(stdout);
    assert(typeof exported === "object" && exported !== null && "sha256" in exported);
    assert.deepEqual(exported, { path, kind: "points", points: 600, source_points: 600, sha256: exported.sha256 });

    const renderer = new FileRenderer({
      cacheDir: join(dir, "render"),
      maxCacheBytes: 128 * 1024 * 1024,
      maxSourceBytes: 64 * 1024 * 1024,
    });
    const rendering = await renderer.render([{ path, sha256: null }], "probe notice");
    assert(rendering !== null);
    const [view] = rendering.details.views;
    assert(view?.kind === "points");
    assert.deepEqual(
      { ...view, preview: null },
      {
        kind: "points",
        label: "probe.json",
        source: { path, sha256: exported.sha256 },
        preview: null,
        points: 600,
        frame: "map",
      },
    );
    assert.equal(
      rendering.text,
      `probe.json · ${path} · sha256 ${exported.sha256} · 600 points, frame map, timestamp 1700000000.25, painted projection`,
    );
    assert.deepEqual(await readFile(path), original);

    const [image] = rendering.images;
    assert.equal(image?.mimeType, "image/png");
    const { data, info } = await sharp(Buffer.from(image.data, "base64")).raw().toBuffer({ resolveWithObject: true });
    assert.deepEqual([info.width, info.height], [900, 448]);
    let orange = 0;
    for (let offset = 0; offset < data.length; offset += info.channels) {
      if (data[offset] > 200 && data[offset + 1] > 40 && data[offset + 1] < 100 && data[offset + 2] < 50) orange++;
    }
    assert(orange > 600, `the export's 0-255 colors are painted, ${orange} orange pixels`);
  },
);
