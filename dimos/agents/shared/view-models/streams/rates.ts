import type { Channel, RateSample } from "#dimos/agents/shared/protocol/streams.ts";
import type { EpochMs } from "#dimos/agents/shared/protocol/time.ts";
import type { Level } from "#dimos/agents/shared/view-models/levels.ts";
import { epochMs } from "#dimos/agents/shared/protocol/time.ts";

export type Rate = { readonly hz: number; readonly lastAt: EpochMs | null };

/** Received rate and freshness reported by the relay SDK. */
export type RateTable = ReadonlyMap<Channel, Rate>;

/** Each SDK sample describes the currently subscribed channels. */
export function ratesOf(sample: RateSample): RateTable {
  return new Map(
    sample.counts.map((count) => [
      count.channel,
      { hz: count.hz, lastAt: count.ageS === null ? null : epochMs(sample.at - count.ageS * 1000) },
    ]),
  );
}

/** Unknown freshness has no level; silence warns after 3 s and becomes critical after 30 s. */
export function rateLevel(rate: Rate | undefined, now: EpochMs): Level | null {
  if (rate === undefined || rate.lastAt === null) return null;
  const silent = now - rate.lastAt;
  if (silent >= 30_000) return "critical";
  if (silent >= 3_000) return "warn";
  return "ok";
}
