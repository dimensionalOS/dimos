import assert from "node:assert/strict";
import { test } from "node:test";
import sharp from "sharp";
import { plot } from "../src/render.js";
import { ChatInput } from "../src/terminal.js";

test("point projection and measured series produce real images without mutating inputs", async () => {
  const input = {
    points: [
      [0, 0, 0],
      [1, 0, 2],
      [1, 2, 3],
    ],
    frame: "map",
  };
  const before = JSON.stringify(input);
  const result = await plot(input, "points");
  assert.equal(JSON.stringify(input), before);
  assert.match(result.summary, /3 source samples/);
  assert.equal((await sharp(result.png).metadata()).width, 800);
  const compare = await plot(
    {
      series: [
        {
          name: "duration_ms",
          values: [
            [1, 12],
            [2, 17],
            [3, 4],
          ],
        },
      ],
    },
    "series",
  );
  assert.match(compare.summary, /duration_ms/);
  await assert.rejects(plot({ points: [[NaN, 1, 2]] }, "points"));
});
test("credentials cannot return through input undo after auth", () => {
  const input = new ChatInput();
  input.secret = true;
  input.handleInput("fixture-secret");
  assert(!input.render(80).join("").includes("fixture-secret"));
  input.secret = false;
  input.handleInput("\x1f");
  input.handleInput("\x19");
  assert.equal(input.getValue(), "");
});
