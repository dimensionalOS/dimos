import assert from "node:assert/strict";
import sharp from "sharp";
import type { RenderProgress } from "#dimos/agents/shared/render/types.ts";
import { FileRenderer } from "#dimos/agents/shared/render/render.ts";
import { createHash } from "node:crypto";
import { join } from "node:path";
import { mkdtemp, readFile, rm, writeFile } from "node:fs/promises";
import { test } from "node:test";
import { tmpdir } from "node:os";

function sha256(bytes: Buffer): string {
  return createHash("sha256").update(bytes).digest("hex");
}

async function scratch(t: {
  after: (fn: () => Promise<void>) => void;
}): Promise<{ dir: string; renderer: FileRenderer }> {
  const dir = await mkdtemp(join(tmpdir(), "render-clip-"));
  t.after(() => rm(dir, { recursive: true, force: true }));
  const renderer = new FileRenderer({
    cacheDir: join(dir, "cache"),
    maxCacheBytes: 128 * 1024 * 1024,
    maxSourceBytes: 64 * 1024 * 1024,
  });
  return { dir, renderer };
}

function svg(color: string): string {
  return `<svg xmlns="http://www.w3.org/2000/svg" width="900" height="448"><rect width="900" height="448" fill="${color}"/></svg>`;
}

test("image clips keep colors, timestamps and hashes; the model gets one contact sheet", async (t) => {
  const { dir, renderer } = await scratch(t);
  const frames = ["#ff0000", "#00ff00", "#0000ff"].map((color, i) => ({
    path: `frame-${i}.svg`,
    timestamp: [100, 100.11, 100.26][i],
    color,
  }));
  for (const frame of frames) await writeFile(join(dir, frame.path), svg(frame.color));
  const index = join(dir, "clip.json");
  await writeFile(index, JSON.stringify({ type: "image", timeOrigin: 99, frames }));
  const originals = await Promise.all(frames.map((frame) => readFile(join(dir, frame.path))));

  const rendering = await renderer.render([{ path: index, sha256: null }], "clip");
  assert(rendering !== null);
  const [view] = rendering.details.views;
  assert(view?.kind === "clip");
  assert.deepEqual(
    view.clip.frames.map((frame) => ({ ...frame, png: null })),
    frames.map((frame, i) => ({
      index: i,
      timestamp: frame.timestamp,
      source: { path: join(dir, frame.path), sha256: sha256(originals[i]) },
      png: null,
    })),
  );
  for (const [i, frame] of view.clip.frames.entries()) {
    const png = await readFile(frame.png.path);
    assert.equal(sha256(png), frame.png.sha256);
    const pixel = await sharp(png).extract({ left: 450, top: 200, width: 1, height: 1 }).raw().toBuffer();
    assert.deepEqual(
      [...pixel],
      [0, 1, 2].map((channel) => (channel === i ? 255 : 0)),
    );
    assert.deepEqual((await sharp(png).metadata()).height, 448 + 28);
  }

  assert.equal(rendering.images.length, 1, "the model gets a contact sheet, not every frame");
  const sheet = await sharp(Buffer.from(rendering.images[0].data, "base64")).metadata();
  assert.deepEqual(
    [rendering.images[0].mimeType, sheet.format, sheet.width, sheet.height],
    ["image/png", "png", 3 * 360, 190],
  );
  assert(view.clip.gif !== null);
  const gif = await sharp(view.clip.gif.path, { animated: true }).metadata();
  assert.equal(gif.pages, 3);
  assert.deepEqual(gif.delay, [110, 150, 100]);

  const [summary, ...rest] = rendering.text.split("\n");
  assert.equal(
    summary,
    `clip.json · ${index} · sha256 ${view.source.sha256} · 3 of 3 image frames, t = 1.000-1.260 s; the image is a contact sheet of 3`,
  );
  assert.deepEqual(rest, [
    `  t = 1.000 s: ${view.clip.frames[0].png.path}`,
    `  t = 1.110 s: ${view.clip.frames[1].png.path}`,
    `  t = 1.260 s: ${view.clip.frames[2].png.path}`,
    `  GIF: ${view.clip.gif.path}`,
  ]);
  for (const [i, frame] of frames.entries()) assert.deepEqual(await readFile(join(dir, frame.path)), originals[i]);
});

test("a clip of camera JPEGs reaches the model as a JPEG contact sheet", async (t) => {
  const { dir, renderer } = await scratch(t);
  const frames = [0, 1, 2, 3].map((i) => ({ path: `frame-${i}.jpg`, timestamp: 10 + i * 0.1 }));
  for (const [i, frame] of frames.entries()) {
    const camera = sharp({
      create: {
        width: 1280,
        height: 720,
        channels: 3,
        background: "#000",
        noise: { type: "gaussian", mean: 60 * i, sigma: 30 },
      },
    });
    await writeFile(join(dir, frame.path), await camera.jpeg().toBuffer());
  }
  const index = join(dir, "camera.json");
  await writeFile(index, JSON.stringify({ type: "image", frames }));
  const rendering = await renderer.render([{ path: index, sha256: null }], "camera");
  assert(rendering !== null);
  const [view] = rendering.details.views;
  assert(view?.kind === "clip");
  const sheet = await sharp(Buffer.from(rendering.images[0].data, "base64")).metadata();
  assert.deepEqual(
    [rendering.images[0].mimeType, sheet.format, sheet.width, sheet.height],
    ["image/jpeg", "jpeg", 1080, 380],
  );
  assert.equal((await sharp(view.sheet.path).metadata()).format, "png", "the terminal's sheet stays a PNG");
});

test("preview frames are at least 0.1 s apart and at most 120, the first and last always kept", async (t) => {
  const { dir, renderer } = await scratch(t);
  await writeFile(join(dir, "frame.svg"), svg("#336699"));
  const dense = join(dir, "dense.json");
  await writeFile(
    dense,
    JSON.stringify({
      type: "image",
      frames: Array.from({ length: 30 }, (_, i) => ({ path: "frame.svg", timestamp: i * 0.03 })),
    }),
  );
  const tenHertz = join(dir, "ten-hertz.json");
  await writeFile(
    tenHertz,
    JSON.stringify({
      type: "image",
      frames: Array.from({ length: 40 }, (_, i) => ({ path: "frame.svg", timestamp: i * 0.1 })),
    }),
  );
  const long = join(dir, "long.json");
  await writeFile(
    long,
    JSON.stringify({
      type: "image",
      frames: Array.from({ length: 6000 }, (_, i) => ({ path: "frame.svg", timestamp: i / 100 })),
    }),
  );

  const rendering = await renderer.render(
    [
      { path: dense, sha256: null },
      { path: tenHertz, sha256: null },
      { path: long, sha256: null },
    ],
    "clips",
  );
  assert(rendering !== null);
  const [denseView, tenHertzView, longView] = rendering.details.views;
  assert(denseView?.kind === "clip" && tenHertzView?.kind === "clip" && longView?.kind === "clip");
  assert.deepEqual(
    denseView.clip.frames.map((frame) => frame.index),
    [0, 4, 8, 12, 16, 20, 24, 28, 29],
  );
  assert.deepEqual(
    tenHertzView.clip.frames.map((frame) => frame.index),
    Array.from({ length: 40 }, (_, i) => i),
    "a 10 Hz export keeps every frame",
  );
  const indexes = longView.clip.frames.map((frame) => frame.index);
  assert.deepEqual([indexes.length, indexes[0], indexes[1], indexes[117], indexes[118]], [119, 0, 51, 5967, 5999]);
});

test("cloud clips share one camera, report progress and reject bad frames", async (t) => {
  const { dir, renderer } = await scratch(t);
  const frames = [
    { path: "a.json", timestamp: 1 },
    { path: "b.json", timestamp: 1.2 },
  ];
  const cloud = {
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
  for (const frame of frames) {
    await writeFile(join(dir, frame.path), JSON.stringify({ ...cloud, timestamp: frame.timestamp }));
  }
  const write = async (name: string, json: unknown): Promise<string> => {
    await writeFile(join(dir, name), JSON.stringify(json));
    return join(dir, name);
  };
  const index = await write("clouds.json", { type: "points", frames });
  const original = await readFile(join(dir, "a.json"));

  const updates: RenderProgress[] = [];
  const rendering = await renderer.render([{ path: index, sha256: null }], "clouds", (update) => updates.push(update));
  assert(rendering !== null);
  const [view] = rendering.details.views;
  assert(view?.kind === "clip");
  assert.deepEqual(updates, [
    { done: 1, total: 2, latest: view.clip.frames[0].png },
    { done: 2, total: 2, latest: view.clip.frames[1].png },
  ]);
  for (const frame of view.clip.frames) {
    const { width, height } = await sharp(frame.png.path).metadata();
    assert.deepEqual([width, height], [900, 476]);
  }
  assert.equal(
    rendering.text.split("\n")[0],
    `clouds.json · ${index} · sha256 ${view.source.sha256} · 2 of 2 points frames, t = 0.000-0.200 s, one camera and height scale; the image is a contact sheet of 2`,
  );
  assert.deepEqual(await readFile(join(dir, "a.json")), original);

  const failure = async (path: string): Promise<string> => {
    const result = await renderer.render([{ path, sha256: null }], "bad clip");
    assert(result !== null);
    assert.deepEqual(result.details.views, []);
    return result.details.failed.map((failed) => failed.error).join("\n");
  };
  const zeros = "0".repeat(64);
  assert.match(
    await failure(await write("changed.json", { type: "points", frames: [{ ...frames[0], sha256: zeros }] })),
    new RegExp(`^frame 0 \\(a\\.json\\): changed since it was exported: sha256 was ${zeros}, is ${sha256(original)}$`),
  );
  assert.equal(
    await failure(await write("not-hex.json", { type: "points", frames: [{ ...frames[0], sha256: "wrong" }] })),
    'not a frame index: /frames/0/sha256 must match pattern "^[0-9a-f]{64}$"',
  );
  assert.equal(
    await failure(await write("backwards.json", { type: "points", frames: frames.toReversed() })),
    "frame timestamps must be strictly increasing; frame 1 is not",
  );
  assert.equal(
    await failure(await write("empty.json", { type: "points", frames: [] })),
    "not a frame index: /frames must not have fewer than 1 items",
  );
  assert.equal(
    await failure(await write("long.json", { type: "points", frames: [frames[0], { ...frames[1], timestamp: 100 }] })),
    "spans 99.000 s, over 60 seconds; export a shorter interval",
  );
  await write("b.json", { ...cloud, frame: "camera", timestamp: 1.2 });
  assert.equal(await failure(index), "frames are in mixed coordinate frames; align them in DimOS before exporting");
  await write("b.json", { ...cloud, timestamp: 7 });
  assert.equal(await failure(index), "frame 1 (b.json): cloud timestamp 7 differs from the index's 1.2");
  assert.equal(
    await renderer.render([{ path: await write("series.json", { series: [] }), sha256: null }], "plot"),
    null,
  );
});
