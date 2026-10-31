import { expect, test } from "vitest";
import { failedWhy, type Launch } from "./run.ts";

const launch = (fields: Partial<Launch>): Launch => ({
  blueprint: "nav",
  phase: "failed",
  pid: 1,
  runId: null,
  ...fields,
});

test("failedWhy: what the gateway said, when it said anything", () => {
  expect(
    failedWhy(
      launch({ error: "Error: Invalid blueprint configuration", output: "$ dimos run nav\n" }),
    ),
  )
    .toBe("Error: Invalid blueprint configuration");
});

test("failedWhy: the last thing printed, when it didn't", () => {
  expect(failedWhy(launch({ output: "$ dimos run nav\nStarting DimOS\nKilled\n\n" }))).toBe(
    "Killed",
  );
});

test("failedWhy: a launch that said and printed nothing still reads as a sentence", () => {
  expect(failedWhy(launch({}))).toBe("it printed nothing");
  expect(failedWhy(launch({ error: "   ", output: "  \n \n" }))).toBe("it printed nothing");
  expect(failedWhy(null)).toBe("it printed nothing");
});

// dimos prints tracebacks in rich's boxes: the gateway's own `error` ends up being the bottom edge of one.
const BOXED = [
  "$ dimos run r1pro-nav",
  "╭───────────────────── Traceback (most recent call last) ──────────────────────╮",
  '│ │   File "/home/nvidia/.dimos/dimos/dimos/protocol/service/zenohservice.py",  │ │',
  "│ │ line 427, in start                                                         │ │",
  "│ │     raise RuntimeError(                                                    │ │",
  "│ │ RuntimeError: zenoh client mode needs a reachable router, none at any      │ │",
  "│ │ scouted locator                                                            │ │",
  "│ │                                                                            │ │",
  "│ ╰────────────────────────────────────────────────────────────────────────────╯ │",
  "╰──────────────────────────────────────────────────────────────────────────────╯",
].join("\n");

test("failedWhy: a boxed traceback reads as its exception, not the box's bottom edge", () => {
  expect(failedWhy(launch({ error: "╰────────────────────╯", output: BOXED })))
    .toBe("RuntimeError: zenoh client mode needs a reachable router, none at any scouted locator");
});

test("failedWhy: the exception wins over the last line, and its frames are left out", () => {
  const output = [
    "ValueError: nothing to plan from",
    'File "/x/plan.py", line 9, in plan',
    "  raise ValueError",
  ].join("\n");
  expect(failedWhy(launch({ output }))).toBe("ValueError: nothing to plan from");
});

test("failedWhy: the last exception is the one shown", () => {
  const output = "RuntimeError: first\nsome chatter\nOSError: the one that ended it\n";
  expect(failedWhy(launch({ output }))).toBe("OSError: the one that ended it");
});
