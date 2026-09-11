import { expect, it } from "vitest";
import { FrameRate } from "./frameRate.ts";
it("counts completed draws over elapsed time", () => {
  const rate = new FrameRate();
  expect(rate.draw(0)).toBeNull();
  for (let i = 1; i < 60; i++) expect(rate.draw(i * 1000 / 60)).toBeNull();
  expect(rate.draw(1000)).toBe(60);
});
it("excludes inactive and hidden time after reset", () => {
  const rate = new FrameRate();
  rate.draw(0);
  rate.draw(500);
  rate.reset();
  expect(rate.draw(10000)).toBeNull();
  for (let i = 1; i < 12; i++) rate.draw(10000 + i * 1000 / 12);
  expect(rate.draw(11000)).toBe(12);
});
