import assert from "node:assert/strict";
import { test } from "node:test";
import sharp from "sharp";
import { cloudSchema, paintCloud } from "../src/points.js";
import { ChatInput } from "../src/input.js";

test("point presentation validates geometry without mutating source", async () => {
  const input = cloudSchema.parse({
    points: [
      [0, 0, 0],
      [1, 0, 2],
      [1, 2, 3],
    ],
    frame: "map",
  });
  const before = JSON.stringify(input);
  const png = await paintCloud(input);
  assert.equal(JSON.stringify(input), before);
  assert.equal((await sharp(png).metadata()).width, 900);
  assert.throws(() => cloudSchema.parse({ points: [[NaN, 1, 2]] }));
  assert.throws(() =>
    cloudSchema.parse({ points: [[0, 1, 2]], selectedIndices: [1] }),
  );
  assert.throws(() => cloudSchema.parse({ points: [[0, 1, 2]], colors: [] }));
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
