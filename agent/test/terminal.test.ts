import assert from "node:assert/strict";
import { mkdtemp, readFile, rm, writeFile } from "node:fs/promises";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { test } from "node:test";
import { setTimeout as delay } from "node:timers/promises";
import {
  ProcessTerminal,
  TuiMainScreen,
  setCapabilities,
  visibleWidth,
} from "@earendil-works/pi-tui";
import { initTheme } from "@earendil-works/pi-coding-agent";
import { PromptEditor } from "../src/input.js";
import { readCloud, SpatialView } from "../src/spatial.js";

test("Pi editor preserves multiline submissions and never puts credentials into chat history", () => {
  initTheme("dark", false);
  const editor = new PromptEditor(new TuiMainScreen(new ProcessTerminal()));
  let submitted = "";
  editor.onSubmit = (value) => {
    submitted = value;
  };
  editor.handleInput("\x1b[200~line one\nline two\x1b[201~");
  editor.handleInput("\r");
  assert.equal(submitted, "line one\nline two");
  editor.setValue("");
  editor.secret = true;
  editor.setValue("");
  editor.handleInput("private-fixture");
  assert(!editor.render(50).join("").includes("private-fixture"));
  editor.secret = false;
  editor.setValue("");
  for (const key of ["\x1f", "\x19", "\x1b[A"]) editor.handleInput(key);
  assert(!editor.getValue().includes("private-fixture"));
});

test("spatial inspector rotates exact data, preserves source selections and rejects changed provenance", async (t) => {
  const home = await mkdtemp(join(tmpdir(), "dimcode-spatial-"));
  t.after(() => rm(home, { recursive: true, force: true }));
  const source = join(home, "cloud.json");
  const data = {
    points: [
      [1, 4, -0.1],
      [2, 6, 0.4],
      [-1, 8, 0.6],
    ],
    frame: "map",
    timestamp: 123.45,
    selectedIndices: [1],
  };
  await writeFile(source, JSON.stringify(data));
  const { cloud, sha256 } = await readCloud(source);
  setCapabilities({ images: null, trueColor: true, hyperlinks: false });
  let renders = 0;
  const view = new SpatialView(cloud, source, sha256, () => renders++);
  t.after(() => view.close());
  const before = view.render(80).join("\n");
  assert.match(before, /1 source-selected points/);
  const start = renders;
  view.handleInput("r");
  const deadline = Date.now() + 2000;
  while (renders === start && Date.now() < deadline) await delay(20);
  assert(renders > start, "rotation redraws the observation");
  assert.notEqual(view.render(80).join("\n"), before);
  for (const width of [25, 40, 100])
    assert(view.render(width).every((line) => visibleWidth(line) <= width));
  assert.deepEqual(cloud, data);
  assert.equal(await readFile(source, "utf8"), JSON.stringify(data));
  await writeFile(source, JSON.stringify({ ...data, timestamp: 124 }));
  await assert.rejects(readCloud(source, sha256), /Source changed/);
  view.close();
  const closed = renders;
  await delay(80);
  assert.equal(renders, closed, "no redraws after disposal");
});
