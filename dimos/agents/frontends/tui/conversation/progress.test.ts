import assert from "node:assert/strict";
import sharp from "sharp";
import type { FileRef, Sha256 } from "../../../shared/protocol/media.ts";
import { ProcessTerminal, TuiMainScreen, setCapabilities, setCellDimensions } from "@earendil-works/pi-tui";
import { ProgressPanel } from "./progress.ts";
import { createHash } from "node:crypto";
import { join } from "node:path";
import { mkdtemp, rm, writeFile } from "node:fs/promises";
import { setTimeout as delay } from "node:timers/promises";
import { test } from "node:test";
import { tmpdir } from "node:os";

class SilentTerminal extends ProcessTerminal {
  override write(_data: string): void {}
  override hideCursor(): void {}
  override showCursor(): void {}
}

test("progress preview keeps its current cells while a newer PNG prepares", async (t) => {
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
  const dir = await mkdtemp(join(tmpdir(), "progress-preview-"));
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
  const tui = new TuiMainScreen(new SilentTerminal());
  const panel = new ProgressPanel(tui);
  t.after(() => panel.dispose());
  const picture = () => panel.render(18).filter((line) => line.includes("▀"));
  const deadline = Date.now() + 3000;
  panel.update({ done: 1, total: 2, latest: red });
  while (picture().length === 0 && Date.now() < deadline) await delay(10);
  const first = picture();
  assert(first.length > 0);
  panel.update({ done: 2, total: 2, latest: blue });
  assert.deepEqual(picture(), first);
  let next = picture();
  while (!next.join("").includes("38;2;0;0;255") && Date.now() < deadline) {
    assert(next.length > 0, "a file read and conversion never blank the preview");
    await delay(10);
    next = picture();
  }
  assert(next.join("").includes("38;2;0;0;255"));
});
