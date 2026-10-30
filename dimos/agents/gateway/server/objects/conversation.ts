import type { Handlers } from "./lib/bind.ts";
import type { ServeObject } from "./base.ts";
import type { Standing } from "../../../shared/protocol/actions/standing.ts";
import type { TranscriptItem } from "../../../shared/protocol/conversation/state.ts";
import { BACKGROUND_CONTEXT, withAbortSignal } from "@earendil-works/chord/context";
import { CONVERSATION_ACTIONS } from "../../../shared/protocol/actions/conversation.ts";
import { ConversationContract, ConversationState } from "../../../shared/protocol/conversation/contract.ts";
import { ConversationRoom } from "./room.ts";
import { GatewayStore } from "./store.ts";
import { bind } from "./lib/bind.ts";
import { conversationStanding } from "../../../shared/protocol/actions/standing.ts";
import { done, refused } from "../../../shared/protocol/outcome.ts";
import { forward } from "./lib/forward.ts";
import { fromBranch, fromSubmit } from "./lib/outcomes.ts";
import { imageAt } from "./lib/items.ts";
import { grantOf } from "../../auth/principal.ts";

const SHARED = Object.keys(ConversationState) as (keyof typeof ConversationState)[];

/** A notice input: anything a module, a job or background work placed, not the user and not the runtime steer. */
function isNotice(item: TranscriptItem): boolean {
  return item.kind === "input" && item.input.source.kind !== "user" && item.input.source.kind !== "runtime";
}

/** One conversation for one principal, with methods limited by the requested role and the principal's grant. */
export const serveConversation: ServeObject = (host, deps) =>
  host.serve(ConversationContract, ({ conversation, role }, principal, self) => {
    const room = self.get(ConversationRoom, { conversation });
    const store = self.get(GatewayStore, {});
    const state = forward(room, SHARED);
    const owner = role === "owner" && grantOf(principal) === "owner";
    self.on("seen", room.transcript, (transcript, ctx) => {
      const newest = transcript.items.findLast(isNotice);
      if (newest === undefined) return;
      return deps.agent.conversations.markSeen(
        conversation,
        newest.entry,
        withAbortSignal(ctx.signal, BACKGROUND_CONTEXT),
      );
    });
    const standing = (): Standing => ({
      role: owner ? "owner" : "viewer",
      link: store.link.value,
      conversation: conversationStanding(conversation, {
        transcript: room.transcript.value,
        live: room.live.value,
        queue: room.queue.value,
        detachable: room.detachable.value,
      }),
    });
    const { agent } = deps;
    const handlers: Handlers<typeof CONVERSATION_ACTIONS> = {
      send: async ({ request, text, images, whenBusy }, call) =>
        fromSubmit(
          await agent.inputs.submit(
            conversation,
            { source: { kind: "user", request }, text, images, render: null, whenBusy },
            call.context,
          ),
          conversation,
        ),
      withdraw: async ({ request }, call) =>
        done((await agent.inputs.withdrawUser(conversation, request, "keep", call.context)).texts),
      abort: async ({ request }, call) => done(await agent.inputs.abortUser(conversation, request, call.context)),
      configure: async ({ model, thinkingLevel }, call) => {
        const handle = await agent.harness.conversation(conversation, call.context);
        if (handle === undefined)
          return refused({ code: "not_found", what: "conversation", name: String(conversation) });
        await handle.configure({ model, thinkingLevel }, call.context);
        return done(null);
      },
      edit: async ({ request, message, text, images }, call) =>
        fromBranch(
          await agent.history.edit(conversation, message, text, images, request, call.context),
          conversation,
          message,
        ),
      summarizeUpTo: async ({ request, message }, call) =>
        fromBranch(
          await agent.history.summarizeUpTo(conversation, message, request, call.context),
          conversation,
          message,
        ),
      summarizeFrom: async ({ request, message }, call) =>
        fromBranch(
          await agent.history.summarizeFrom(conversation, message, request, call.context),
          conversation,
          message,
        ),
      background: async ({ call: target }, call) => {
        const result = await agent.detacher.detach(conversation, target, call.context);
        return result.kind === "detached"
          ? done({ task: result.task })
          : refused({ code: "not_running", call: target });
      },
      image: async ({ at }, call) => imageAt(await agent.harness.commit((tx) => tx.entry(at.entry), call.context), at),
    };
    if (!owner)
      return {
        variant: "viewer",
        ...state,
        ...bind("conversation", CONVERSATION_ACTIONS, handlers, standing, deps.replies, "viewer"),
      };
    return {
      variant: "owner",
      ...state,
      ...bind("conversation", CONVERSATION_ACTIONS, handlers, standing, deps.replies, "owner"),
    };
  });
