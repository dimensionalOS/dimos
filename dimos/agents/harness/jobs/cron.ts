import type { EpochMs } from "../../shared/protocol/time.ts";
import type { TimeZone } from "../../shared/protocol/jobs/state.ts";
import { epochMs } from "../../shared/protocol/time.ts";
import { readFileSync, realpathSync } from "node:fs";

const DAY_MS = 86_400_000;

/** A day-of-month that never occurs (e.g. February 30) must not loop forever. */
const SEARCH_DAYS = 366 * 5;

const FIELDS = [
  ["minute", 0, 59],
  ["hour", 0, 23],
  ["day of month", 1, 31],
  ["month", 1, 12],
  ["day of week", 0, 7],
] as const;

export function parseTimeZone(name: string): TimeZone {
  try {
    return new Intl.DateTimeFormat("en-US", { timeZone: name }).resolvedOptions().timeZone as TimeZone;
  } catch (error) {
    if (error instanceof RangeError) throw new Error(`Unknown time zone ${JSON.stringify(name)}`);
    throw error;
  }
}

/** This machine's zone from TZ, the Intl default, /etc/timezone or the /etc/localtime link; UTC when none is valid. */
export function localTimeZone(): TimeZone {
  const candidates = [
    process.env["TZ"]?.replace(/^:/, ""),
    Intl.DateTimeFormat().resolvedOptions().timeZone,
    readOrNull("/etc/timezone")?.trim(),
    zoneinfoName("/etc/localtime"),
  ];
  for (const name of candidates) {
    if (!name) continue;
    try {
      return parseTimeZone(name);
    } catch {
      continue;
    }
  }
  return parseTimeZone("UTC");
}

function readOrNull(path: string): string | null {
  try {
    return readFileSync(path, "utf8");
  } catch {
    return null;
  }
}

function zoneinfoName(link: string): string | null {
  try {
    const target = realpathSync(link);
    const index = target.indexOf("zoneinfo/");
    return index < 0 ? null : target.slice(index + "zoneinfo/".length);
  } catch {
    return null;
  }
}

function number(text: string, field: string): number {
  if (!/^\d+$/.test(text)) throw new Error(`Invalid ${field} value ${JSON.stringify(text)}`);
  return Number(text);
}

function parseField(text: string, field: string, low: number, high: number): readonly number[] {
  const values = new Set<number>();
  for (const part of text.split(",")) {
    const slash = part.indexOf("/");
    const base = slash < 0 ? part : part.slice(0, slash);
    const step = slash < 0 ? 1 : number(part.slice(slash + 1), field);
    if (step < 1) throw new Error(`Invalid ${field} step ${JSON.stringify(part.slice(slash + 1))}`);
    let start: number;
    let end: number;
    if (base === "*") {
      start = low;
      end = high;
    } else if (base.includes("-")) {
      const dash = base.indexOf("-");
      start = number(base.slice(0, dash), field);
      end = number(base.slice(dash + 1), field);
    } else {
      start = number(base, field);
      end = slash < 0 ? start : high;
    }
    if (!(low <= start && start <= end && end <= high))
      throw new Error(`${field} ${JSON.stringify(part)} is outside ${low}-${high}`);
    for (let value = start; value <= end; value += step) values.add(value);
  }
  return [...values].sort((a, b) => a - b);
}

const formatters = new Map<TimeZone, Intl.DateTimeFormat>();

/** The zone's wall clock at `instant`, as the Date.UTC value of its fields. */
function wallAt(instant: number, zone: TimeZone): number {
  let formatter = formatters.get(zone);
  if (formatter === undefined) {
    formatter = new Intl.DateTimeFormat("en-US", {
      timeZone: zone,
      hourCycle: "h23",
      year: "numeric",
      month: "numeric",
      day: "numeric",
      hour: "numeric",
      minute: "numeric",
      second: "numeric",
    });
    formatters.set(zone, formatter);
  }
  const fields: Record<string, number> = {};
  for (const part of formatter.formatToParts(new Date(instant))) {
    if (part.type !== "literal") fields[part.type] = Number(part.value);
  }
  return Date.UTC(
    fields["year"] ?? 1970,
    (fields["month"] ?? 1) - 1,
    fields["day"] ?? 1,
    fields["hour"] ?? 0,
    fields["minute"] ?? 0,
    fields["second"] ?? 0,
  );
}

/**
 * The instant at which the zone's wall clock reads `wall` (a Date.UTC value), or null when a daylight-saving gap
 * skips it. A wall time that occurs twice gives its first instance.
 */
function instantOf(wall: number, zone: TimeZone): number | null {
  const offsets = [wall - DAY_MS, wall + DAY_MS].map((probe) => wallAt(probe, zone) - probe);
  const instants = offsets.map((offset) => wall - offset).filter((instant) => wallAt(instant, zone) === wall);
  return instants.length === 0 ? null : Math.min(...instants);
}

type CronFields = {
  readonly minutes: readonly number[];
  readonly hours: readonly number[];
  readonly days: readonly number[];
  readonly months: readonly number[];
  readonly weekdays: readonly number[];
  /** As in Vixie cron, a field that starts with "*" is unrestricted. */
  readonly daysRestricted: boolean;
  readonly weekdaysRestricted: boolean;
};

/** A standard five-field cron expression: minute hour day-of-month month day-of-week. Ported from dimos/agents/harness/cron.py. */
export class CronExpr {
  readonly expression: string;
  private readonly fields: CronFields;

  private constructor(expression: string, fields: CronFields) {
    this.expression = expression;
    this.fields = fields;
  }

  /** Throws with the field and value at fault. */
  static parse(expression: string): CronExpr {
    const parts = expression.trim().split(/\s+/).filter(Boolean);
    if (parts.length !== 5)
      throw new Error("A cron expression has 5 fields: minute hour day-of-month month day-of-week");
    const [minutes, hours, days, months, weekdays] = FIELDS.map(([field, low, high], index) =>
      parseField(parts[index] ?? "", field, low, high),
    );
    return new CronExpr(parts.join(" "), {
      minutes: minutes ?? [],
      hours: hours ?? [],
      days: days ?? [],
      months: months ?? [],
      weekdays: [...new Set((weekdays ?? []).map((day) => day % 7))].sort((a, b) => a - b),
      daysRestricted: !(parts[2] ?? "").startsWith("*"),
      weekdaysRestricted: !(parts[4] ?? "").startsWith("*"),
    });
  }

  private matchesDay(month: number, dayOfMonth: number, weekday: number): boolean {
    const { months, days, weekdays, daysRestricted, weekdaysRestricted } = this.fields;
    if (!months.includes(month)) return false;
    const inMonth = days.includes(dayOfMonth);
    const inWeek = weekdays.includes(weekday);
    return daysRestricted && weekdaysRestricted ? inMonth || inWeek : inMonth && inWeek;
  }

  /**
   * The first matching minute strictly after `after`, in `zone`. A local time skipped by a daylight-saving change
   * does not occur; a repeated one occurs once, at its first instance. Throws when nothing matches in five years.
   */
  nextAfter(after: EpochMs, zone: TimeZone): EpochMs {
    const start = new Date(wallAt(after, zone));
    let day = Date.UTC(start.getUTCFullYear(), start.getUTCMonth(), start.getUTCDate());
    for (let step = 0; step < SEARCH_DAYS; step += 1, day += DAY_MS) {
      const date = new Date(day);
      if (!this.matchesDay(date.getUTCMonth() + 1, date.getUTCDate(), date.getUTCDay())) continue;
      for (const hour of this.fields.hours) {
        for (const minute of this.fields.minutes) {
          const instant = instantOf(day + hour * 3_600_000 + minute * 60_000, zone);
          if (instant !== null && instant > after) return epochMs(instant);
        }
      }
    }
    throw new Error(`${JSON.stringify(this.expression)} has no occurrence in the next five years`);
  }
}
