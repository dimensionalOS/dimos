import assert from "node:assert/strict";
import sharp from "sharp";
import { StreamCardView } from "./card.ts";
import { setCapabilities, setCellDimensions, visibleWidth } from "@earendil-works/pi-tui";
import { setTimeout as delay } from "node:timers/promises";
import { test } from "node:test";

test("sidebar pictures replace live fallback cells and close pending work", async (t) => {
  const previous = { TERM: process.env.TERM, NO_COLOR: process.env.NO_COLOR };
  process.env.TERM = "xterm-256color";
  delete process.env.NO_COLOR;
  t.after(() => {
    for (const [name, value] of Object.entries(previous)) {
      if (value === undefined) delete process.env[name];
      else process.env[name] = value;
    }
  });
  setCapabilities({ images: null, trueColor: false, hyperlinks: false });
  setCellDimensions({ widthPx: 10, heightPx: 20 });
  let updates = 0;
  const card = new StreamCardView(71, 12, 5, () => updates++);
  t.after(() => card.dispose());
  const frame = (background: string) =>
    sharp({ create: { width: 80, height: 40, channels: 3, background } })
      .png()
      .toBuffer();
  const deadline = Date.now() + 3000;
  const ready = async () => {
    const before = updates;
    card.render(16);
    while (updates === before && Date.now() < deadline) await delay(10);
    assert(updates > before, "prepared cells request a redraw");
    return card.render(16);
  };
  card.setFrame(await frame("red"));
  const red = await ready();
  assert(red.some((line) => line.includes("▀")));
  assert(red.every((line) => visibleWidth(line) <= 16));
  card.setFrame(await frame("blue"));
  assert.deepEqual(card.render(16), red, "the displayed frame stays visible while its replacement prepares");
  const blue = await ready();
  assert.notDeepEqual(blue, red, "the next frame changes picture colors");
  card.setFrame(await frame("green"));
  card.render(16);
  card.dispose();
  const closed = updates;
  await delay(50);
  assert.equal(updates, closed, "a removed card cannot request a late redraw");
});

test("sidebar retains its kitty image id across frames", async (t) => {
  const previous = process.env.TERM_PROGRAM;
  process.env.TERM_PROGRAM = "kitty";
  t.after(() => {
    if (previous === undefined) delete process.env.TERM_PROGRAM;
    else process.env.TERM_PROGRAM = previous;
  });
  setCapabilities({ images: "kitty", trueColor: true, hyperlinks: false });
  setCellDimensions({ widthPx: 10, heightPx: 20 });
  const card = new StreamCardView(71, 12, 5, () => {});
  t.after(() => card.dispose());
  for (const background of ["red", "blue"]) {
    card.setFrame(
      await sharp({ create: { width: 80, height: 40, channels: 3, background } })
        .png()
        .toBuffer(),
    );
    assert.match(card.render(16).join(""), /\x1b_G[^;]*i=71/);
  }
});
