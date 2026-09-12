import assert from "node:assert/strict";
import { execFile } from "node:child_process";
import { createHash } from "node:crypto";
import { mkdtemp, readFile, rm, writeFile } from "node:fs/promises";
import { tmpdir } from "node:os";
import { join, resolve } from "node:path";
import { test } from "node:test";
import { promisify } from "node:util";
import { setCapabilities, visibleWidth } from "@earendil-works/pi-tui";
import sharp from "sharp";
import { paths } from "../src/config.js";
import {
  clipSourceSchema,
  frameDelays,
  previewIndexes,
  renderClip,
} from "../src/clips.js";
import { readArtifact } from "../src/artifacts.js";
import { fitClouds, projectPoint, type Cloud } from "../src/points.js";
import { renderSaved } from "../src/render.js";
import { Playback } from "../src/playback.js";

test("preview timing is bounded and retains source indexes; projection uses shared bounds", () => {
  const times = Array.from({ length: 6000 }, (_, i) => 123 + i / 100);
  const indexes = previewIndexes(times);
  assert(indexes.length <= 120);
  assert.equal(indexes[0], 0);
  assert.equal(indexes.at(-1), times.length - 1);
  const selected = indexes.map((i) => times[i]);
  assert(selected.every((time, i) => !i || time > selected[i - 1]));
  assert.deepEqual(frameDelays([0, 0.11, 0.26]), [110, 150, 100]);
  const clouds: Cloud[] = [
    {
      points: [
        [0, 0, 0],
        [1, 2, 3],
      ],
    },
    {
      points: [
        [1, 2, 3],
        [8, 4, 2],
      ],
    },
  ];
  const camera = fitClouds(clouds);
  assert.deepEqual(
    projectPoint(clouds[0].points[1], camera, 900, 400),
    projectPoint(clouds[1].points[0], camera, 900, 400),
  );
  assert.equal(camera.extent, 8);
  assert.equal(camera.floor, 0);
  assert.equal(camera.height, 3);
});

test("saved image/SVG clips preserve colors, timestamps and hashes; playback loops without controls and disposes", async (t) => {
  const dir = await mkdtemp(join(tmpdir(), "dimcode-clips-"));
  t.after(() => rm(dir, { recursive: true, force: true }));
  const p = paths({ XDG_CACHE_HOME: dir });
  const source = join(dir, "clip.json");
  const frames = ["#ff0000", "#00ff00", "#0000ff"].map((color, i) => ({
    path: `frame-${i}.svg`,
    timestamp: [100, 100.11, 100.26][i],
    color,
  }));
  for (const frame of frames)
    await writeFile(
      join(dir, frame.path),
      `<svg xmlns="http://www.w3.org/2000/svg" width="900" height="448"><rect width="900" height="448" fill="${frame.color}"/></svg>`,
    );
  await writeFile(
    source,
    JSON.stringify({ type: "image", timeOrigin: 99, frames }),
  );
  const result = await renderSaved(p, dir, source, { kind: "points" });
  const clip = result.details.clip!;
  assert.equal(clip.frames.length, 3);
  assert.equal(
    result.content.filter((item) => item.type === "image").length,
    1,
    "model gets a contact sheet, not every playback frame",
  );
  assert.equal(
    result.details.kind,
    "sequence",
    "tagged indexes override a still-view hint",
  );
  for (const [i, frame] of clip.frames.entries()) {
    assert.equal(frame.timestamp, frames[i].timestamp);
    assert.equal(frame.index, i);
    assert.equal(
      frame.sourceSha256,
      createHash("sha256")
        .update(await readFile(frame.source))
        .digest("hex"),
    );
    const { bytes } = await readArtifact(frame.path, frame.sha256);
    const rgb = await sharp(bytes)
      .extract({ left: 450, top: 200, width: 1, height: 1 })
      .raw()
      .toBuffer();
    assert.deepEqual(
      [...rgb],
      [0, 1, 2].map((channel) => (channel === i ? 255 : 0)),
    );
  }
  const gif = await sharp(clip.gif!, { animated: true }).metadata();
  assert.equal(gif.pages, 3);
  assert.deepEqual(gif.delay, [110, 150, 100]);
  const single = await renderSaved(p, dir, source, { frame: 1 });
  assert.equal(single.details.clip?.frames[0].index, 1);
  assert.equal(single.details.clip?.gif, undefined);
  await assert.rejects(
    renderSaved(p, dir, source, { frame: 3 }),
    /source frame index/,
  );
  setCapabilities({ images: null, trueColor: true, hyperlinks: false });
  let updates = 0;
  let onChange = () => {};
  const view = new Playback(clip, () => {
    updates++;
    onChange();
  });
  t.after(() => view.close());
  await view.ready;
  for (const width of [1, 20, 80])
    assert(view.render(width).every((line) => visibleWidth(line) <= width));
  assert.doesNotMatch(view.render(80).join("\n"), /Play|Pause|seek|timeline/);
  const looped = new Promise<void>((resolve, reject) => {
    const timeout = setTimeout(
      () => reject(new Error("Playback did not loop")),
      1500,
    );
    let reachedLast = false;
    onChange = () => {
      reachedLast ||= view.frameIndex === 2;
      if (reachedLast && view.frameIndex === 0) {
        clearTimeout(timeout);
        resolve();
      }
    };
  });
  await looped;
  assert.equal(view.playing, true, "clips loop without controls");
  view.pause();
  assert.equal(view.playing, false);
  view.close();
  const closed = updates;
  view.pause();
  assert.equal(updates, closed, "disposed views cannot notify or restart");
  const pausedWhileLoading = new Playback(clip, () => {}, true);
  pausedWhileLoading.pause();
  await pausedWhileLoading.ready;
  assert.equal(
    pausedWhileLoading.playing,
    false,
    "loading cannot restart a paused or hidden clip",
  );
  pausedWhileLoading.close();
  await writeFile(clip.frames[0].path, "changed");
  const stale = new Playback(clip, () => {});
  await stale.ready;
  assert.match(stale.render(80).join("\n"), /Source changed/);
  assert.equal(stale.playing, false);
  stale.close();
});

test("cloud clips reject changed sources, mismatched frames/times, invalid windows and cancellation", async (t) => {
  const dir = await mkdtemp(join(tmpdir(), "dimcode-clouds-"));
  t.after(() => rm(dir, { recursive: true, force: true }));
  const p = paths({ XDG_CACHE_HOME: dir }),
    source = join(dir, "clouds.json");
  const frames = [
    { path: "a.json", timestamp: 1 },
    { path: "b.json", timestamp: 1.2 },
  ];
  const cloud: Cloud = {
    points: [
      [0, 0, 0],
      [1, 2, 3],
    ],
    frame: "world",
    colors: [
      [255, 0, 0],
      [0, 255, 0],
    ],
    selectedIndices: [1],
  };
  for (const frame of frames)
    await writeFile(
      join(dir, frame.path),
      JSON.stringify({ ...cloud, timestamp: frame.timestamp }),
    );
  const data = { type: "points", frames };
  const original = await readFile(join(dir, frames[0].path));
  const result = await renderClip(p, source, data);
  assert.equal(result.clip.frames.length, 2);
  assert.match(result.summary, /fixed camera and height scale/);
  assert.deepEqual(await readFile(join(dir, frames[0].path)), original);
  await assert.rejects(
    renderClip(p, source, {
      ...data,
      frames: [{ ...frames[0], sha256: "wrong" }],
    }),
    /Source changed/,
  );
  await writeFile(
    join(dir, "b.json"),
    JSON.stringify({ ...cloud, frame: "camera" }),
  );
  await assert.rejects(renderClip(p, source, data), /Mixed coordinate/);
  await writeFile(
    join(dir, "b.json"),
    JSON.stringify({ ...cloud, timestamp: 7 }),
  );
  await assert.rejects(renderClip(p, source, data), /timestamp differs/);
  assert.throws(
    () => clipSourceSchema.parse({ ...data, frames: frames.toReversed() }),
    /strictly increasing/,
  );
  assert.throws(() => clipSourceSchema.parse({ ...data, frames: [] }));
  await assert.rejects(
    renderClip(p, source, {
      ...data,
      frames: [frames[0], { ...frames[1], timestamp: 100 }],
    }),
    /60 seconds/,
  );
  const abort = new AbortController();
  abort.abort();
  await assert.rejects(renderClip(p, source, data, abort.signal), /abort/i);
  await writeFile(source, JSON.stringify({ series: [] }));
  await assert.rejects(renderSaved(p, dir, source), /DimOS or Python as SVG/);
});

test(
  "Go2 memory seconds 1–5 export once and render cloud, camera and native plot",
  {
    timeout: 120000,
    skip: !(process.env.DIMCODE_TEST_PYTHON && process.env.DIMCODE_TEST_GO2_DB),
  },
  async (t) => {
    const dir = await mkdtemp(join(tmpdir(), "dimcode-playback-go2-"));
    t.after(() => rm(dir, { recursive: true, force: true }));
    await promisify(execFile)(
      process.env.DIMCODE_TEST_PYTHON!,
      ["test/fixtures/clip_views.py", dir],
      { env: { ...process.env, PYTHONPATH: resolve("..") }, timeout: 60000 },
    );
    const p = paths({ XDG_CACHE_HOME: dir });
    for (const type of ["points", "image"]) {
      const source = join(dir, type + ".json");
      const index = clipSourceSchema.parse(
        JSON.parse(await readFile(source, "utf8")),
      );
      assert(index.frames.length > 20);
      assert(
        index.frames.every(
          (frame) =>
            frame.timestamp >= index.timeOrigin! + 1 &&
            frame.timestamp < index.timeOrigin! + 5,
        ),
      );
      const result = await renderSaved(p, dir, source);
      const clip = result.details.clip!;
      assert(clip.frames.length > 15);
      assert.equal(clip.sourceCount, index.frames.length);
      for (const frame of clip.frames) {
        assert.equal(frame.sourceSha256, index.frames[frame.index].sha256);
        assert.equal(frame.timestamp, index.frames[frame.index].timestamp);
      }
      assert.equal(
        (await sharp(clip.gif!, { animated: true }).metadata()).pages,
        clip.frames.length,
      );
    }
    const plot = await renderSaved(p, dir, "point-count.svg");
    assert.equal(
      plot.content.filter((item) => item.type === "image").length,
      1,
    );
  },
);
