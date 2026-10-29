import type { ConversationId, JsonObject } from "@earendil-works/pi-durable";
import type { Static } from "typebox";
import { ExportSchema } from "#dimos/agents/shared/protocol/runtime/export.ts";
import { Type } from "typebox";

/** The class name an agent module runs under. */
export type AgentModuleName = string & { readonly __brand: "AgentModuleName" };

/** A live module notice delivered through event intake. */
export const EventSchema = Type.Object({
  id: Type.String({ pattern: "^[0-9a-fA-F]{8}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-9a-fA-F]{12}$" }),
  module: Type.Unsafe<AgentModuleName>(Type.String()),
  /** The conversation that created the module; null for modules created outside any conversation. */
  conversation: Type.Union([Type.Unsafe<ConversationId>(Type.Integer({ minimum: 1 })), Type.Null()]),
  /** Unix seconds. */
  at: Type.Number(),
  text: Type.String(),
  data: Type.Unsafe<JsonObject>(Type.Record(Type.String(), Type.Unknown())),
  /** Images and point clouds `notify` saved, already replaced by their paths in `data`. */
  exports: Type.Array(ExportSchema),
  /** Notices the rate limit dropped since this module's previous event. */
  suppressed: Type.Integer({ minimum: 0 }),
});

export type HarnessEvent = Static<typeof EventSchema>;
