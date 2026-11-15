import assert from "node:assert/strict";
import { CronExpr, parseTimeZone } from "#dimos/agents/harness/jobs/cron.ts";
import { epochMs } from "#dimos/agents/shared/protocol/time.ts";
import { test } from "node:test";

const LA = parseTimeZone("America/Los_Angeles");

const UTC = parseTimeZone("UTC");

/** The next occurrence as an ISO instant, so expectations are literal UTC times independent of the code under test. */
function next(expression: string, after: string, zone = LA): string {
  return new Date(CronExpr.parse(expression).nextAfter(epochMs(Date.parse(after)), zone)).toISOString();
}

test("every five minutes is strictly after", () => {
  // 2026-10-13 15:20 in Los Angeles (PDT, UTC-7).
  assert.equal(next("*/5 * * * *", "2026-10-13T22:20:00Z"), "2026-10-13T22:25:00.000Z");
  assert.equal(next("*/5 * * * *", "2026-10-13T22:20:01Z"), "2026-10-13T22:25:00.000Z");
  assert.equal(next("*/5 * * * *", "2026-10-13T22:21:59Z"), "2026-10-13T22:25:00.000Z");
});

test("ranges, lists and rollover", () => {
  // 9:00 and 17:30 on weekdays; Friday evening rolls to Monday morning.
  const friday = "2026-10-17T01:00:00Z";
  assert.equal(next("0,30 9,17 * * 1-5", friday), "2026-10-19T16:00:00.000Z");
  assert.equal(next("0 0 1 1 *", friday), "2027-01-01T08:00:00.000Z");
});

test("sunday is zero or seven", () => {
  const saturday = "2026-10-17T19:00:00Z";
  assert.equal(next("0 8 * * 0", saturday), "2026-10-18T15:00:00.000Z");
  assert.equal(next("0 8 * * 7", saturday), "2026-10-18T15:00:00.000Z");
});

test("both day fields match either", () => {
  // The 20th (a Tuesday) or any Friday, whichever comes first.
  const start = "2026-10-13T19:00:00Z";
  assert.equal(next("0 6 20 * 5", start), "2026-10-16T13:00:00.000Z");
  assert.equal(next("0 6 20 * 5", "2026-10-17T07:00:00Z"), "2026-10-20T13:00:00.000Z");
  // A stepped "*" field is unrestricted, so both fields must match.
  assert.equal(next("0 6 */2 * 5", start), "2026-10-23T13:00:00.000Z");
});

test("timezone and utc input", () => {
  const moment = "2026-10-13T22:00:00Z";
  assert.equal(next("30 15 * * *", moment), "2026-10-13T22:30:00.000Z");
  assert.equal(next("30 15 * * *", moment, UTC), "2026-10-14T15:30:00.000Z");
});

test("daylight saving gap and repeat", () => {
  // 02:30 does not exist on 2026-03-08 in Los Angeles.
  assert.equal(next("30 2 * * *", "2026-03-08T08:00:00Z"), "2026-03-09T09:30:00.000Z");
  // 01:30 occurs twice on 2026-11-01; only the first instance runs.
  const first = next("30 1 * * *", "2026-11-01T07:00:00Z");
  assert.equal(first, "2026-11-01T08:30:00.000Z");
  assert.equal(next("30 1 * * *", first), "2026-11-02T09:30:00.000Z");
});

test("invalid expressions", () => {
  for (const expression of [
    "* * * *",
    "60 * * * *",
    "* 24 * * *",
    "* * 0 * *",
    "*/0 * * * *",
    "a * * * *",
    "5-1 * * * *",
  ]) {
    assert.throws(() => CronExpr.parse(expression), Error, expression);
  }
});

test("impossible date is an error", () => {
  assert.throws(() => next("0 0 30 2 *", "2026-01-01T08:00:00Z"), /no occurrence/);
});

test("unknown zone is an error", () => {
  assert.throws(() => parseTimeZone("Mars/Olympus"), /Unknown time zone "Mars\/Olympus"/);
});
