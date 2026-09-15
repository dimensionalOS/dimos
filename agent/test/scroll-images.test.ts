import assert from "node:assert/strict";
import { test } from "node:test";
import { setTimeout as delay } from "node:timers/promises";
import {
  ScrollView,
  setCapabilities,
  setCellDimensions,
} from "@earendil-works/pi-tui";
import { renderLayoutFrame } from "@earendil-works/pi-tui/dist/layout.js";
import sharp from "sharp";
import { TerminalImage } from "../src/terminal-image.js";

// Decode the actual PNG transmissions as a terminal that ignores source-crop controls would.
function transmittedPng(line: string): Buffer {
  const commands = [...line.matchAll(/\x1b_G([^;]*);([^\x1b]*)\x1b\\/g)];
  assert(commands.length);
  assert.match(commands[0][1], /(?:^|,)r=1(?:,|$)/);
  assert.doesNotMatch(commands[0][1], /(?:^|,)[yhw]=/);
  return Buffer.from(commands.map((command) => command[2]).join(""), "base64");
}

test("Warp scrolling clips exact pixel rows instead of squeezing the original image", async (t) => {
  const previous = process.env.TERM_PROGRAM;
  process.env.TERM_PROGRAM = "WarpTerminal";
  t.after(() => {
    if (previous === undefined) delete process.env.TERM_PROGRAM;
    else process.env.TERM_PROGRAM = previous;
  });
  setCapabilities({ images: "kitty", trueColor: true, hyperlinks: false });
  setCellDimensions({ widthPx: 10, heightPx: 20 });
  const png = await sharp(
    Buffer.from(
      '<svg xmlns="http://www.w3.org/2000/svg" width="900" height="500"><rect width="900" height="500" fill="#101b21"/><circle cx="450" cy="250" r="190" fill="#43bda7"/><path d="M0 120H900M450 0V500" stroke="white" stroke-width="8"/></svg>',
    ),
  )
    .removeAlpha()
    .png()
    .toBuffer();
  let updates = 0;
  const image = new TerminalImage(
    png.toString("base64"),
    "image/png",
    { fallbackColor: (text) => text },
    { maxWidthCells: 90, maxHeightCells: 25 },
    () => updates++,
  );
  t.after(() => image.close());
  const ready = async (width: number) => {
    const before = updates;
    image.render(width);
    const deadline = Date.now() + 3000;
    while (updates === before && Date.now() < deadline) await delay(10);
    assert(updates > before, "image preparation redraws the terminal");
  };
  await ready(92);
  const scroll = new ScrollView(image, { follow: "none", scrollbar: "hidden" });
  renderLayoutFrame(scroll, 92, 7, () => {});
  for (const row of [0, 4, 11, 18, 0]) {
    scroll.scrollTo(row);
    const frame = renderLayoutFrame(scroll, 92, 7, () => {});
    assert.equal(frame.lines.length, 7);
    const visible = Buffer.concat(
      await Promise.all(
        frame.lines.map(async (line) => {
          const bytes = transmittedPng(line);
          const metadata = await sharp(bytes).metadata();
          assert.equal(metadata.width, 900);
          assert.equal(metadata.height, 20);
          return sharp(bytes).raw().toBuffer();
        }),
      ),
    );
    const expected = await sharp(png)
      .extract({ left: 0, top: row * 20, width: 900, height: 140 })
      .raw()
      .toBuffer();
    assert.deepEqual(
      visible,
      expected,
      "scrolling changes only which source rows are visible",
    );
  }
  await ready(47);
  const narrow = image.render(47);
  assert.equal(narrow.length, 13);
  assert.equal((await sharp(transmittedPng(narrow[0])).metadata()).width, 450);
  image.render(70);
  image.close();
  const closed = updates;
  await delay(50);
  assert.equal(
    updates,
    closed,
    "resizing then closing cannot redraw a disposed view",
  );
});
