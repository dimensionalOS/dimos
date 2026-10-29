import type { AgentState, ConversationRecord, InboxState, LiveState, UsageState } from "@earendil-works/pi-durable";
import type { Detachable } from "./calls.ts";
import type { InputOutcome, OwnedStream, Transcript } from "./state.ts";
import type { JobRow } from "../jobs/state.ts";
import type { ModuleRow } from "../runtime/module.ts";
import { CONVERSATION_ACTIONS } from "../actions/conversation.ts";
import { ConversationIdSchema } from "../args.ts";
import { Type } from "typebox";
import { carried } from "../schema.ts";
import { hostNamed, remote } from "@earendil-works/chord";
import { methodsOf } from "../actions/methods.ts";

/**
 * Everything one conversation shows, one record used by the gateway's ConversationRoom and by both variants of the
 * served view. The pi-durable members are its documents as pi-durable keeps them, fed from `watch()`; the DimOS members
 * are the gateway's shared rows filtered to what this conversation owns.
 */
export const ConversationState = {
  record: carried<ConversationRecord>(),
  /** The active range projected, image bytes stripped, within the transfer budget. */
  transcript: carried<Transcript>(),
  /** `pi.live`: run control, the streamed partial, running tool calls with their output. */
  live: carried<LiveState>(),
  /** `pi.inbox`: queued items in submission order, user and notice alike. */
  queue: carried<InboxState>(),
  usage: carried<UsageState>(),
  /** `pi.agent` as stored; unset fields follow the gateway's settings. */
  agent: carried<AgentState>(),
  outcomes: carried<readonly InputOutcome[]>(),
  detachable: carried<readonly Detachable[]>(),
  modules: carried<readonly ModuleRow[]>(),
  jobs: carried<readonly JobRow[]>(),
  /** Its modules' readable relay channels, shown in the sidebar. */
  streams: carried<readonly OwnedStream[]>(),
} as const;

/**
 * One conversation for one client. The requested role is limited by the principal's grant. Every authorized owner
 * can send input; viewers read. The methods come from the catalog.
 */
export const ConversationContract = remote("dimos.conversation", {
  version: 1,
  input: Type.Object({
    conversation: ConversationIdSchema,
    role: Type.Union([Type.Literal("owner"), Type.Literal("viewer")]),
  }),
  host: hostNamed("gateway"),
  variants: {
    owner: { state: ConversationState, methods: methodsOf(CONVERSATION_ACTIONS, "owner") },
    viewer: { state: ConversationState, methods: methodsOf(CONVERSATION_ACTIONS, "viewer") },
  },
});
