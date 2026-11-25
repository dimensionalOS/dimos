import assert from "node:assert/strict";
import sharp from "sharp";
import {
  ScrollView,
  detectCapabilities,
  setCapabilities,
  setCellDimensions,
  visibleWidth,
} from "@earendil-works/pi-tui";
import { TerminalImage } from "#dimos/agents/frontends/tui/lib/terminal-image.ts";
import { renderLayoutFrame } from "@earendil-works/pi-tui/dist/layout.js";
import { setTimeout as delay } from "node:timers/promises";
import { test } from "node:test";

/** Decodes the PNG transmissions of one line as a terminal that ignores source-crop controls would. */
function transmittedPng(line: string): Buffer {
  const commands = [...line.matchAll(/\x1b_G([^;]*);([^\x1b]*)\x1b\\/g)];
  assert(commands.length > 0, "the line transmits an image");
  assert.match(commands[0]?.[1] ?? "", /(?:^|,)r=1(?:,|$)/);
  assert.doesNotMatch(commands[0]?.[1] ?? "", /(?:^|,)[yhw]=/);
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
  const image = new TerminalImage(png, { maxWidthCells: 90, maxHeightCells: 25 }, () => updates++);
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
    assert.deepEqual(visible, expected, "scrolling changes only which source rows are visible");
  }
  await ready(47);
  const narrow = image.render(47);
  assert.equal(narrow.length, 13);
  assert.equal((await sharp(transmittedPng(narrow[0] ?? "")).metadata()).width, 450);
  image.render(70);
  image.close();
  const closed = updates;
  await delay(50);
  assert.equal(updates, closed, "resizing then closing cannot redraw a disposed view");
});

test("a JPEG the model was sent reaches a kitty terminal as a PNG", async (t) => {
  const previous = process.env.TERM_PROGRAM;
  process.env.TERM_PROGRAM = "kitty";
  t.after(() => {
    if (previous === undefined) delete process.env.TERM_PROGRAM;
    else process.env.TERM_PROGRAM = previous;
  });
  setCapabilities({ images: "kitty", trueColor: true, hyperlinks: false });
  setCellDimensions({ widthPx: 10, heightPx: 20 });
  const jpeg = await sharp({ create: { width: 320, height: 180, channels: 3, background: "#43bda7" } })
    .jpeg()
    .toBuffer();
  let updates = 0;
  const image = new TerminalImage(jpeg, { maxWidthCells: 40, maxHeightCells: 10 }, () => updates++);
  t.after(() => image.close());
  assert.deepEqual(image.render(42), [], "nothing is drawn while the JPEG converts");
  const deadline = Date.now() + 3000;
  while (updates === 0 && Date.now() < deadline) await delay(10);
  const [first = ""] = image.render(42);
  const sent = Buffer.from(
    [...first.matchAll(/\x1b_G[^;]*;([^\x1b]*)\x1b\\/g)].map((command) => command[1]).join(""),
    "base64",
  );
  const { format, width, height } = await sharp(sent).metadata();
  assert.deepEqual([format, width, height], ["png", 320, 180]);
});

test("unsupported graphics terminals draw bounded pictures in their available color mode", async (t) => {
  const previous = {
    TERM: process.env.TERM,
    NO_COLOR: process.env.NO_COLOR,
    PI_IMAGE_PROTOCOL: process.env.PI_IMAGE_PROTOCOL,
    TERM_PROGRAM: process.env.TERM_PROGRAM,
    SSH_CONNECTION: process.env.SSH_CONNECTION,
  };
  t.after(() => {
    for (const [name, value] of Object.entries(previous)) {
      if (value === undefined) delete process.env[name];
      else process.env[name] = value;
    }
  });
  process.env.TERM_PROGRAM = "kitty";
  process.env.PI_IMAGE_PROTOCOL = "none";
  process.env.SSH_CONNECTION = "host 1 client 2";
  setCellDimensions({ widthPx: 10, heightPx: 20 });
  const png = await sharp(
    Buffer.from(
      '<svg xmlns="http://www.w3.org/2000/svg" width="80" height="80"><rect width="40" height="40" fill="red"/><rect x="40" width="40" height="40" fill="white"/><rect y="40" width="40" height="40" fill="blue"/><rect x="40" y="40" width="40" height="40" fill="black"/></svg>',
    ),
  )
    .png()
    .toBuffer();
  const jpeg = await sharp(png).jpeg().toBuffer();
  for (const [term, trueColor, noColor, expected, blocks] of [
    ["xterm-256color", true, false, /\x1b\[38;2;\d+;\d+;\d+m/, true],
    ["xterm-256color", false, false, /\x1b\[38;5;\d+m/, true],
    ["linux", false, false, /\x1b\[3[0-7]m/, true],
    ["linux", true, false, /\x1b\[3[0-7]m/, true],
    ["dumb", false, true, /[.:-=+*#%@]/, false],
    ["vt100", false, false, /[.:-=+*#%@]/, false],
  ] as const) {
    process.env.TERM = term;
    if (noColor) process.env.NO_COLOR = "1";
    else delete process.env.NO_COLOR;
    setCapabilities({ ...detectCapabilities(() => false), trueColor });
    let updates = 0;
    const image = new TerminalImage(jpeg, { maxWidthCells: 12, maxHeightCells: 5 }, () => updates++);
    const deadline = Date.now() + 3000;
    let lines = image.render(14);
    while ((!lines.some((line) => line.length > 0) || updates < 2) && Date.now() < deadline) {
      await delay(10);
      lines = image.render(14);
    }
    assert.equal(lines.length, 5);
    assert(lines.every((line) => visibleWidth(line) <= 12));
    assert(
      lines.some((line) => expected.test(line)),
      `${term} draws picture cells`,
    );
    assert.equal(lines.join("").includes("▀"), blocks);
    assert(!lines.join("").includes("\x1b_G"));
    assert(!lines.join("").includes("\x1b]1337"));
    const narrower = image.render(7);
    assert.equal(narrower.length, 3);
    const beforeResize = updates;
    while (updates === beforeResize && Date.now() < deadline) await delay(10);
    assert(updates > beforeResize);
    assert(image.render(7).every((line) => visibleWidth(line) <= 5));
    image.close();
  }
});

test("cell pictures compensate for terminal cells taller than two pixels wide", async (t) => {
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
  setCellDimensions({ widthPx: 10, heightPx: 30 });
  const png = await sharp({ create: { width: 40, height: 40, channels: 3, background: "red" } })
    .png()
    .toBuffer();
  let updates = 0;
  const image = new TerminalImage(png, { maxWidthCells: 6, maxHeightCells: 2 }, () => updates++);
  t.after(() => image.close());
  image.render(8);
  const deadline = Date.now() + 3000;
  while (updates === 0 && Date.now() < deadline) await delay(10);
  assert(updates > 0);
  const lines = image.render(8);
  assert.equal(lines.length, 2);
  assert(lines.every((line) => visibleWidth(line) === 6));
  assert(lines.join("").includes("▀"));
  for (const color of lines.join("").matchAll(/\x1b\[(?:38|48);2;(\d+);(\d+);(\d+)m/g)) {
    assert.deepEqual(color.slice(1).map(Number), [255, 0, 0], "the square source fills its square physical box");
  }
});

test("cell pictures keep the last frame through rapid aspect changes and discard old layouts", async (t) => {
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
  const frame = (background: string, width = 80, height = 40) =>
    sharp({ create: { width, height, channels: 3, background } })
      .png()
      .toBuffer();
  const [red, blue, green] = await Promise.all([frame("red"), frame("blue", 40, 80), frame("green")]);
  let updates = 0;
  const image = new TerminalImage(red, { maxWidthCells: 12, maxHeightCells: 5 }, () => updates++);
  t.after(() => image.close());
  const waitForUpdate = async (before: number) => {
    const deadline = Date.now() + 3000;
    while (updates === before && Date.now() < deadline) await delay(10);
    assert(updates > before, "completed conversion requests a redraw");
  };
  image.render(14);
  await waitForUpdate(0);
  const first = image.render(14);
  assert(first.join("").includes("▀"));

  image.setPng(blue);
  assert.deepEqual(image.render(14), first);
  image.setPng(green);
  assert.deepEqual(image.render(14), first, "new source does not cancel the active conversion");
  await waitForUpdate(1);
  const second = image.render(14);
  assert.notDeepEqual(second, first, "the pending frame can publish despite a newer source");
  assert(second.join("").includes("38;2;0;0;255"));
  await waitForUpdate(2);
  const third = image.render(14);
  assert(third.join("").includes("38;2;0;128;0"));

  image.setPng(blue);
  assert.deepEqual(image.render(14), third);
  const narrow = image.render(7);
  assert(
    narrow.every((line) => !line.includes("▀")),
    "old geometry is not shown after a resize",
  );
  const beforeNarrow = updates;
  await waitForUpdate(beforeNarrow);
  assert(image.render(7).every((line) => visibleWidth(line) <= 5));
  const beforeClose = updates;
  image.setPng(red);
  image.render(7);
  image.close();
  await delay(50);
  assert.equal(updates, beforeClose, "disposed conversion cannot request a redraw");
});

test("a failed replacement reports its error and a later PNG recovers", async (t) => {
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
  const frame = (background: string) =>
    sharp({ create: { width: 80, height: 40, channels: 3, background } })
      .png()
      .toBuffer();
  const [red, blue] = await Promise.all([frame("red"), frame("blue")]);
  let updates = 0;
  const image = new TerminalImage(red, { maxWidthCells: 12, maxHeightCells: 5 }, () => updates++);
  t.after(() => image.close());
  const waitForUpdate = async (before: number) => {
    const deadline = Date.now() + 3000;
    while (updates === before && Date.now() < deadline) await delay(10);
    assert(updates > before);
  };
  image.render(14);
  await waitForUpdate(0);
  const ready = image.render(14);
  image.setPng(red.subarray(0, 24));
  assert.deepEqual(image.render(14), ready);
  await waitForUpdate(1);
  assert(image.render(14).join("").includes("Image previ"));
  image.setPng(blue);
  assert(image.render(14).join("").includes("Image previ"));
  await waitForUpdate(2);
  assert(image.render(14).join("").includes("38;2;0;0;255"));
});

test("iTerm2 keeps its native image sequence", async (t) => {
  setCapabilities({ images: "iterm2", trueColor: true, hyperlinks: false });
  setCellDimensions({ widthPx: 10, heightPx: 20 });
  const png = await sharp({ create: { width: 20, height: 20, channels: 3, background: "red" } })
    .png()
    .toBuffer();
  const image = new TerminalImage(png, { maxWidthCells: 8, maxHeightCells: 4 }, () => {});
  t.after(() => image.close());
  assert.match(image.render(10).join(""), /\x1b]1337;File=/);
});

test("native Kitty updates keep an allocated image id", async (t) => {
  const previous = process.env.TERM_PROGRAM;
  process.env.TERM_PROGRAM = "kitty";
  t.after(() => {
    if (previous === undefined) delete process.env.TERM_PROGRAM;
    else process.env.TERM_PROGRAM = previous;
  });
  setCapabilities({ images: "kitty", trueColor: true, hyperlinks: false });
  setCellDimensions({ widthPx: 10, heightPx: 20 });
  const frame = (background: string) =>
    sharp({ create: { width: 80, height: 40, channels: 3, background } })
      .png()
      .toBuffer();
  const [red, blue] = await Promise.all([frame("red"), frame("blue")]);
  const image = new TerminalImage(red, { maxWidthCells: 12, maxHeightCells: 5 }, () => {});
  t.after(() => image.close());
  const first = image.render(14).join("");
  const id = first.match(/\x1b_G[^;]*i=(\d+)/)?.[1];
  assert(id);
  image.setPng(blue);
  const second = image.render(14).join("");
  assert.match(second, new RegExp(`\\x1b_G[^;]*i=${id}(?:,|;)`));
  assert.notEqual(second, first, "the replacement sends new image data");
});
