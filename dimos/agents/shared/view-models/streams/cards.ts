import type { Channel, StreamRef } from "#dimos/agents/shared/protocol/streams.ts";
import type { EpochMs } from "#dimos/agents/shared/protocol/time.ts";
import type { Level } from "#dimos/agents/shared/view-models/levels.ts";
import type { ModuleInstance, ModuleRow } from "#dimos/agents/shared/protocol/runtime/module.ts";
import type { OwnedStream } from "#dimos/agents/shared/protocol/conversation/state.ts";
import type { Picture } from "#dimos/agents/shared/view-models/streams/kind.ts";
import type { Rate, RateTable } from "#dimos/agents/shared/view-models/streams/rates.ts";
import { pictureOf } from "#dimos/agents/shared/view-models/streams/kind.ts";
import { rateLevel } from "#dimos/agents/shared/view-models/streams/rates.ts";

/** One stream card: its picture kind decides whether the client paints its frames or shows its newest value. */
export type StreamCard = {
  readonly channel: Channel;
  readonly title: string;
  readonly direction: "out" | "inout" | null;
  readonly type: string;
  readonly picture: Picture | null;
  readonly rate: Rate | null;
  readonly level: Level | null;
};

/** A module heading and its stream cards. */
export type CardGroup = {
  readonly title: string;
  readonly module: ModuleRow | null;
  readonly cards: readonly StreamCard[];
};

function cardOf(ref: StreamRef, rates: RateTable, now: EpochMs): StreamCard {
  const rate = rates.get(ref.channel);
  return {
    channel: ref.channel,
    title: ref.source?.name ?? ref.channel,
    direction: ref.source?.direction ?? null,
    type: ref.source?.type ?? ref.encoding,
    picture: pictureOf(ref.encoding),
    rate: rate ?? null,
    level: rateLevel(rate, now),
  };
}

/** One group per module this conversation owns. */
export function sidebarGroups(
  streams: readonly OwnedStream[],
  modules: readonly ModuleRow[],
  rates: RateTable,
  now: EpochMs,
): readonly CardGroup[] {
  const groups: CardGroup[] = [];
  for (const module of modules) {
    const own = streams.filter((stream) => stream.via.module === module.name).map((stream) => stream.ref);
    groups.push({
      title: module.name,
      module,
      cards: own.map((ref) => cardOf(ref, rates, now)),
    });
  }
  return groups;
}

/** Each readable relay channel once, with unannotated channels under Relay. */
export function allStreamGroups(
  streams: readonly StreamRef[],
  modules: readonly ModuleRow[],
  rates: RateTable,
  now: EpochMs,
): readonly CardGroup[] {
  const shown = new Map(streams.map((ref) => [ref.channel, ref]));
  const byInstance = new Map<ModuleInstance | null, StreamRef[]>();
  for (const ref of shown.values()) {
    const instance = ref.source?.module ?? null;
    const refs = byInstance.get(instance);
    if (refs === undefined) byInstance.set(instance, [ref]);
    else refs.push(ref);
  }
  return [...byInstance].map(([instance, refs]) => {
    const module = instance === null ? undefined : modules.find((candidate) => candidate.instance === instance);
    return {
      title: instance ?? "Relay",
      module: module === undefined ? null : module,
      cards: refs.map((ref) => cardOf(ref, rates, now)),
    };
  });
}

/**
 * Picture channels the conversation does not already show, grouped by annotated module or under Relay.
 * Streams without a picture are counted in `others`, which `/menu` lists.
 */
export function systemGroups(
  streams: readonly StreamRef[],
  modules: readonly ModuleRow[],
  shown: ReadonlySet<Channel>,
  rates: RateTable,
  now: EpochMs,
): { readonly groups: readonly CardGroup[]; readonly others: number } {
  const all = allStreamGroups(
    streams.filter((ref) => !shown.has(ref.channel)),
    modules,
    rates,
    now,
  );
  const cards = all.flatMap((group) => group.cards);
  const groups = all
    .map((group) => ({
      ...group,
      cards: group.cards.filter((card) => card.picture !== null),
    }))
    .filter((group) => group.cards.length > 0);
  return { groups, others: cards.filter((card) => card.picture === null).length };
}
