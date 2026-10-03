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
import { z } from "zod";
import { paths } from "../src/config.js";
import { renderFiles } from "../src/render.js";
import { ResultImages } from "../src/tool-images.js";

test("memory tool keeps all SVG/PNG exports, provenance and selectable panels", async (t) => {
  const dir = await mkdtemp(join(tmpdir(), "dimcode-memory-"));
  t.after(() => rm(dir, { recursive: true, force: true }));
  const p = paths({ XDG_CACHE_HOME: dir });
  const views = [
    { label: "Timeline", path: "timeline.svg" },
    { label: "Spatial", path: "space.svg" },
    { label: "Frames", path: "grid.png" },
  ];
  // Unit fixtures stay independent of the documentation's Git LFS assets.
  const cwd = dir;
  for (const [name, shape] of [
    ["timeline.svg", '<path d="M0 100L200 20L400 150" stroke="green"/>'],
    ["space.svg", '<circle cx="150" cy="100" r="50" fill="blue"/>'],
  ])
    await writeFile(
      join(dir, name),
      `<svg xmlns="http://www.w3.org/2000/svg" width="500" height="300">${shape}</svg>`,
    );
  await sharp({
    create: { width: 300, height: 200, channels: 3, background: "#8b7c35" },
  })
    .png()
    .toFile(join(dir, "grid.png"));
  const result = await renderFiles(p, cwd, views, "Memory · format fixture");
  const images = result.content.filter((item) => item.type === "image");
  assert.equal(images.length, 3, "every view reaches model context");
  assert.deepEqual(
    result.details.views?.map((view) => view.label),
    views.map((view) => view.label),
  );
  for (const [i, view] of views.entries()) {
    const source = resolve(cwd, view.path);
    const bytes = await readFile(source);
    assert.equal(
      result.details.views?.[i].sha256,
      createHash("sha256").update(bytes).digest("hex"),
    );
    const metadata = await sharp(
      Buffer.from(images[i].data, "base64"),
    ).metadata();
    assert.equal(metadata.format, "png");
    assert(metadata.width > 100 && metadata.width <= 1600);
    assert(
      result.content.some(
        (item) => item.type === "text" && item.text.includes(source),
      ),
    );
  }
  setCapabilities({ images: null, trueColor: true, hyperlinks: false });
  let updated!: () => void;
  const ready = new Promise<void>((resolve) => {
    updated = resolve;
  });
  const gallery = new ResultImages(
    images.map((image, i) => ({ ...image, label: views[i].label })),
    updated,
  );
  t.after(() => gallery.close());
  await ready;
  const overview = gallery.render(100).join("\n");
  assert.match(overview, /Overview/);
  assert.match(overview, /3 Frames/);
  assert.doesNotMatch(overview, /Preparing|unavailable/);
  gallery.select(3);
  assert.notEqual(gallery.render(100).join("\n"), overview);
  assert(
    gallery.handleMouse({
      type: "press",
      button: "left",
      x: 2,
      y: 0,
      screenX: 2,
      screenY: 0,
      width: 100,
      height: 30,
      shift: false,
      alt: false,
      ctrl: false,
    })?.handled,
  );
  assert.equal(gallery.render(100).join("\n"), overview);
  for (const width of [20, 40, 100])
    assert(gallery.render(width).every((line) => visibleWidth(line) <= width));
  await assert.rejects(renderFiles(p, cwd, []), /Choose/);
  await assert.rejects(
    renderFiles(p, cwd, [{ label: "Missing", path: "missing.png" }]),
    /ENOENT/,
  );
});

test(
  "recorded Go2 memory query exports its timeline, spatial selection and matching frames",
  {
    timeout: 90000,
    skip: !(process.env.DIMCODE_TEST_PYTHON && process.env.DIMCODE_TEST_GO2_DB),
  },
  async (t) => {
    const dir = await mkdtemp(join(tmpdir(), "dimcode-memory-go2-"));
    t.after(() => rm(dir, { recursive: true, force: true }));
    await promisify(execFile)(
      process.env.DIMCODE_TEST_PYTHON!,
      ["test/fixtures/memory_views.py", dir],
      {
        env: { ...process.env, PYTHONPATH: resolve("..") },
        timeout: 80000,
      },
    );
    const report = z
      .object({
        observations: z.number().gt(10),
        selected: z
          .array(
            z.object({
              timestamp: z.number().positive(),
              similarity: z.number().finite(),
            }),
          )
          .min(1),
        context_cloud_timestamps: z.array(z.number().positive()).min(1),
      })
      .parse(JSON.parse(await readFile(join(dir, "query.json"), "utf8")));
    assert(report.selected.every((row) => row.similarity > 0));
    const result = await renderFiles(
      paths({ XDG_CACHE_HOME: dir }),
      dir,
      [
        { label: "Timeline", path: "timeline.svg" },
        { label: "Spatial", path: "space.svg" },
        { label: "Frames", path: "frames.png" },
      ],
      "Memory · plant",
    );
    assert.equal(
      result.content.filter((item) => item.type === "image").length,
      3,
    );
    assert.equal(result.details.views?.length, 3);
  },
);
