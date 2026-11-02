import assert from "node:assert/strict";
import sharp from "sharp";
import type { ImageContent } from "@earendil-works/pi-ai";
import type { Sharp } from "sharp";
import { FileRenderer } from "#dimos/agents/shared/render/render.ts";
import { createHash } from "node:crypto";
import { join } from "node:path";
import { mkdtemp, readFile, rm, stat, writeFile } from "node:fs/promises";
import { test } from "node:test";
import { tmpdir } from "node:os";

function sha256(bytes: Buffer): string {
  return createHash("sha256").update(bytes).digest("hex");
}

test("an image group shows six views, gives the model inline formats as they are, and notes the rest by path", async (t) => {
  const dir = await mkdtemp(join(tmpdir(), "render-group-"));
  t.after(() => rm(dir, { recursive: true, force: true }));
  const solid = sharp({ create: { width: 64, height: 48, channels: 3, background: "#c04020" } });
  const images: [string, Buffer][] = [
    ["a.png", await solid.clone().png().toBuffer()],
    ["b.jpg", await solid.clone().jpeg().toBuffer()],
    ["c.webp", await solid.clone().webp().toBuffer()],
    ["d.gif", await solid.clone().gif().toBuffer()],
    ["e.tiff", await solid.clone().tiff().toBuffer()],
    [
      "f.svg",
      Buffer.from(
        '<svg xmlns="http://www.w3.org/2000/svg" width="64" height="48"><rect width="64" height="48"/></svg>',
      ),
    ],
    ["g.avif", await solid.clone().avif().toBuffer()],
    ["h.png", await solid.clone().png({ compressionLevel: 1 }).toBuffer()],
  ];
  for (const [name, bytes] of images) await writeFile(join(dir, name), bytes);
  await writeFile(join(dir, "notes.txt"), "not an image\n");
  await writeFile(join(dir, "data.json"), JSON.stringify({ series: [1, 2, 3] }));
  const paths = ["a.png", "notes.txt", "b.jpg", "c.webp", "data.json", "d.gif", "e.tiff", "f.svg", "g.avif", "h.png"];

  const cacheDir = join(dir, "cache");
  const renderer = new FileRenderer({ cacheDir, maxCacheBytes: 128 * 1024 * 1024, maxSourceBytes: 64 * 1024 * 1024 });
  const rendering = await renderer.render(
    paths.map((name) => ({ path: join(dir, name), sha256: null })),
    "bash: python plots.py",
  );
  assert(rendering !== null);

  const bytes = new Map(images);
  const hash = new Map(images.map(([name, bytes]) => [name, sha256(bytes)]));
  assert.equal(rendering.details.title, "bash: python plots.py");
  assert.deepEqual(
    rendering.details.views.map((view) => ({ ...view, preview: null })),
    ["a.png", "b.jpg", "c.webp", "d.gif", "e.tiff", "f.svg"].map((name) => ({
      kind: "image",
      label: name,
      source: { path: join(dir, name), sha256: hash.get(name) },
      preview: null,
    })),
  );
  assert.deepEqual(rendering.details.listed, [join(dir, "g.avif"), join(dir, "h.png")]);
  assert.deepEqual(rendering.details.failed, []);
  assert.equal(
    rendering.text,
    [
      `a.png · ${join(dir, "a.png")} · sha256 ${hash.get("a.png")} · png 64x48`,
      `b.jpg · ${join(dir, "b.jpg")} · sha256 ${hash.get("b.jpg")} · jpeg 64x48`,
      `c.webp · ${join(dir, "c.webp")} · sha256 ${hash.get("c.webp")} · webp 64x48`,
      `d.gif · ${join(dir, "d.gif")} · sha256 ${hash.get("d.gif")} · gif 64x48`,
      `e.tiff · ${join(dir, "e.tiff")} · sha256 ${hash.get("e.tiff")} · tiff 64x48, sent as png 64x48`,
      `f.svg · ${join(dir, "f.svg")} · sha256 ${hash.get("f.svg")} · svg 128x96, sent as png 128x96`,
      `[image not shown: ${join(dir, "g.avif")} (64x48); read the path to see it]`,
      `[image not shown: ${join(dir, "h.png")} (64x48); read the path to see it]`,
    ].join("\n"),
  );

  assert.deepEqual(
    rendering.images.map((image: ImageContent) => image.mimeType),
    ["image/png", "image/jpeg", "image/webp", "image/gif", "image/png", "image/png"],
  );
  for (const [i, name] of ["a.png", "b.jpg", "c.webp", "d.gif"].entries()) {
    assert.deepEqual(
      Buffer.from(rendering.images[i].data, "base64"),
      bytes.get(name),
      `${name} reaches the model as is`,
    );
  }
  for (const view of rendering.details.views) {
    assert(view.kind === "image");
    const preview = await readFile(view.preview.path);
    assert.equal(sha256(preview), view.preview.sha256);
    assert.equal(view.preview.path, join(cacheDir, `${view.preview.sha256}.png`));
    assert.equal((await sharp(preview).metadata()).format, "png", "the terminal gets a PNG");
    assert.equal((await stat(view.preview.path)).mode & 0o777, 0o600);
  }
  assert.equal((await stat(cacheDir)).mode & 0o777, 0o700);
  for (const [name, bytes] of images) assert.deepEqual(await readFile(join(dir, name)), bytes);
});

test("a photo over 1600x1200 is resized as JPEG, a transparent one as PNG until its base64 fits, a turned JPEG set upright", async (t) => {
  const dir = await mkdtemp(join(tmpdir(), "render-fit-"));
  t.after(() => rm(dir, { recursive: true, force: true }));
  const noise = (width: number, height: number, channels: 3 | 4): Sharp =>
    sharp({
      create: { width, height, channels, background: "#000", noise: { type: "gaussian", mean: 128, sigma: 40 } },
    });
  await writeFile(join(dir, "wide.jpg"), await noise(3200, 1800, 3).jpeg().toBuffer());
  await writeFile(join(dir, "alpha.png"), await noise(2400, 2400, 4).png().toBuffer());
  await writeFile(join(dir, "turned.jpg"), await noise(120, 80, 3).jpeg().withMetadata({ orientation: 6 }).toBuffer());
  const renderer = new FileRenderer({
    cacheDir: join(dir, "cache"),
    maxCacheBytes: 256 * 1024 * 1024,
    maxSourceBytes: 64 * 1024 * 1024,
  });
  const rendering = await renderer.render(
    ["wide.jpg", "alpha.png", "turned.jpg"].map((name) => ({ path: join(dir, name), sha256: null })),
    "fit",
  );
  assert(rendering !== null);
  assert.deepEqual(
    rendering.text.split("\n").map((line) => line.split(" · ")[3]),
    ["jpeg 3200x1800, sent as jpeg 1600x900", "png 2400x2400, sent as png 900x900", "jpeg 80x120"],
  );
  const sent = await Promise.all(
    rendering.images.map(async (image) => {
      const { format, width, height, orientation, hasAlpha } = await sharp(
        Buffer.from(image.data, "base64"),
      ).metadata();
      return { mimeType: image.mimeType, format, width, height, orientation, hasAlpha };
    }),
  );
  assert.deepEqual(sent, [
    { mimeType: "image/jpeg", format: "jpeg", width: 1600, height: 900, orientation: undefined, hasAlpha: false },
    { mimeType: "image/png", format: "png", width: 900, height: 900, orientation: undefined, hasAlpha: true },
    { mimeType: "image/jpeg", format: "jpeg", width: 80, height: 120, orientation: undefined, hasAlpha: false },
  ]);
});

test("an unreadable, oversized or changed file is reported; nothing renderable gives null", async (t) => {
  const dir = await mkdtemp(join(tmpdir(), "render-failed-"));
  t.after(() => rm(dir, { recursive: true, force: true }));
  const renderer = new FileRenderer({ cacheDir: join(dir, "cache"), maxCacheBytes: 1024 * 1024, maxSourceBytes: 4096 });
  const pixel = (background: string): Promise<Buffer> =>
    sharp({ create: { width: 8, height: 8, channels: 3, background } })
      .png()
      .toBuffer();
  const exported = await pixel("#000");
  const changed = await pixel("#fff");
  const small = join(dir, "small.png");
  await writeFile(small, exported);
  const first = await renderer.render([{ path: small, sha256: null }], "export");
  assert(first !== null && first.details.views[0] !== undefined);
  const { sha256: hashAtExport } = first.details.views[0].source;
  assert.equal(hashAtExport, sha256(exported));
  await writeFile(small, changed);
  await writeFile(join(dir, "large.bin"), Buffer.alloc(5000));
  await writeFile(join(dir, "notes.txt"), "plain text\n");

  const missing = join(dir, "missing.png");
  const large = join(dir, "large.bin");
  const rendering = await renderer.render(
    [
      { path: missing, sha256: null },
      { path: large, sha256: null },
      { path: small, sha256: hashAtExport },
    ],
    "exports",
  );
  assert(rendering !== null);
  assert.deepEqual(rendering.details.views, []);
  assert.deepEqual(rendering.images, []);
  const errors = [
    `ENOENT: no such file or directory, stat '${missing}'`,
    "5000 bytes, over the 4096-byte limit",
    `changed since it was exported: sha256 was ${sha256(exported)}, is ${sha256(changed)}`,
  ];
  assert.deepEqual(rendering.details.failed, [
    { path: missing, error: errors[0] },
    { path: large, error: errors[1] },
    { path: small, error: errors[2] },
  ]);
  assert.equal(
    rendering.text,
    [
      `Not rendered: ${missing}: ${errors[0]}`,
      `Not rendered: ${large}: ${errors[1]}`,
      `Not rendered: ${small}: ${errors[2]}`,
    ].join("\n"),
  );
  assert.equal(await renderer.render([{ path: join(dir, "notes.txt"), sha256: null }], "notes"), null);
});
