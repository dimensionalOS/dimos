import { expect, test } from "vitest";
import { oneInFlight } from "./live.ts";

test("asked again while one runs, it runs once more after it, never twice at once", async () => {
  let running = 0;
  let most = 0;
  let runs = 0;
  const releases: (() => void)[] = [];
  const go = oneInFlight(async () => {
    runs++;
    running++;
    most = Math.max(most, running);
    await new Promise<void>((resolve) => releases.push(resolve));
    running--;
  });
  go();
  go();
  go();
  releases.shift()!();
  await new Promise((resolve) => setTimeout(resolve));
  releases.shift()!();
  await new Promise((resolve) => setTimeout(resolve));
  expect(runs).toBe(2);
  expect(most).toBe(1);
});
