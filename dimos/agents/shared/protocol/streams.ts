import type { EpochMs } from "#dimos/agents/shared/protocol/time.ts";
import type { ModuleInstance } from "#dimos/agents/shared/protocol/runtime/module.ts";
import type { Static } from "typebox";
import { Type } from "typebox";
import { nullable } from "#dimos/agents/shared/protocol/schema.ts";

/** A DimOS message type by its class name, as inventory reports it and the model reads it, e.g. `Image`. */
export type MsgType = string & { readonly __brand: "MsgType" };

/** A channel ID from the relay manifest, used verbatim with Session.subscribe. */
export type Channel = string & { readonly __brand: "Channel" };

export const ChannelSchema = Type.Unsafe<Channel>(Type.String({ minLength: 1 }));

export const StreamRefSchema = Type.Object({
  channel: ChannelSchema,
  encoding: Type.String(),
  source: nullable(
    Type.Object({
      module: Type.Unsafe<ModuleInstance>(Type.String()),
      name: Type.String(),
      direction: Type.Union([Type.Literal("out"), Type.Literal("inout")]),
      type: Type.Unsafe<MsgType>(Type.String()),
    }),
  ),
});

/** A readable relay channel, optionally annotated by an exact module.port inventory match. */
export type StreamRef = Static<typeof StreamRefSchema>;

/** SDK received rate and source age for each requested preview. */
export type RateSample = {
  readonly at: EpochMs;
  readonly counts: readonly { readonly channel: Channel; readonly hz: number; readonly ageS: number | null }[];
};
