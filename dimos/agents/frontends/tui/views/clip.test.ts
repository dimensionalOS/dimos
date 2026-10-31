import assert from "node:assert/strict";
import sharp from "sharp";
import type { FileRef, RenderedView, Sha256 } from "../../../shared/protocol/media.ts";
import { ClipPanel } from "./clip.ts";
import { createHash } from "node:crypto";
import { join } from "node:path";
import { mkdtemp, rm, writeFile } from "node:fs/promises";
import { setCapabilities, setCellDimensions } from "@earendil-works/pi-tui";
import { setTimeout as delay } from "node:timers/promises";
import { test } from "node:test";
import { tmpdir } from "node:os";

test("clip playback retains its visible cells while the next saved PNG prepares", async (t) => {
  const previous = { TERM: process.env.TERM, NO_COLOR: process.env.NO_COLOR };
  process.env.TERM = "xterm-256color";
  delete process.env.NO_COLOR;
  t.after(() => {
    for (const [name, value] of Object.entries(previous)) {
      if (value === undefined) delete process.env[name];
      else process.env[name] = value;
    }
  });
  setCapabilities({ images: null, trueColor: true, hyperlinks: false });
  setCellDimensions({ widthPx: 10, heightPx: 20 });
  const dir = await mkdtemp(join(tmpdir(), "clip-preview-"));
  t.after(() => rm(dir, { recursive: true, force: true }));
  const ref = async (color: string): Promise<FileRef> => {
    const png = await sharp({ create: { width: 80, height: 40, channels: 3, background: color } })
      .png()
      .toBuffer();
    const path = join(dir, `${color}.png`);
    await writeFile(path, png);
    return { path, sha256: createHash("sha256").update(png).digest("hex") as Sha256 };
  };
  const [red, blue] = await Promise.all([ref("red"), ref("blue")]);
  const view = {
    kind: "clip",
    label: "Saved clip",
    source: red,
    sheet: red,
    clip: {
      frames: [
        { index: 0, timestamp: 0, source: red, png: red },
        { index: 1, timestamp: 0.2, source: blue, png: blue },
      ],
      gif: null,
    },
  } satisfies Extract<RenderedView, { readonly kind: "clip" }>;
  const panel = new ClipPanel(view, () => {});
  t.after(() => panel.dispose());
  const deadline = Date.now() + 3000;
  const picture = () => panel.render(18).filter((line) => line.includes("▀"));
  while (picture().length === 0 && Date.now() < deadline) await delay(10);
  const first = picture();
  assert(first.join("").includes("38;2;255;0;0"));
  let next = first;
  while (!next.join("").includes("38;2;0;0;255") && Date.now() < deadline) {
    assert(next.length > 0, "playback keeps a picture visible between frames");
    await delay(10);
    next = picture();
  }
  assert(next.join("").includes("38;2;0;0;255"));
});

test("warmed Warp clips keep their prepared frames throughout playback", async (t) => {
  const previous = process.env.TERM_PROGRAM;
  process.env.TERM_PROGRAM = "WarpTerminal";
  t.after(() => {
    if (previous === undefined) delete process.env.TERM_PROGRAM;
    else process.env.TERM_PROGRAM = previous;
  });
  setCapabilities({ images: "kitty", trueColor: true, hyperlinks: false });
  setCellDimensions({ widthPx: 10, heightPx: 20 });
  const dir = await mkdtemp(join(tmpdir(), "warp-clip-preview-"));
  t.after(() => rm(dir, { recursive: true, force: true }));
  const refs = await Promise.all(
    ["red", "blue"].map(async (color): Promise<FileRef> => {
      const png = await sharp({ create: { width: 80, height: 40, channels: 3, background: color } })
        .png()
        .toBuffer();
      const path = join(dir, `${color}.png`);
      await writeFile(path, png);
      return { path, sha256: createHash("sha256").update(png).digest("hex") as Sha256 };
    }),
  );
  const view = {
    kind: "clip",
    label: "Saved clip",
    source: refs[0]!,
    sheet: refs[0]!,
    clip: {
      frames: refs.map((ref, index) => ({ index, timestamp: index * 0.15, source: ref, png: ref })),
      gif: null,
    },
  } satisfies Extract<RenderedView, { readonly kind: "clip" }>;
  let recording = false;
  const frames: string[][] = [];
  const panel = new ClipPanel(view, () => {
    const lines = panel.render(18);
    if (recording) frames.push(lines);
  });
  t.after(() => panel.dispose());
  panel.render(18);
  await delay(550);
  recording = true;
  await delay(550);
  assert(frames.length >= 2, "playback continues after both frames prepare");
  assert(
    frames.every((lines) => lines.some((line) => line.includes("\x1b_G"))),
    "prepared frames never blank",
  );
});
