# Shared protocol

The protocol is the agreement between the gateway and its clients: the Chord contracts the gateway serves, the
catalog of actions that become their methods, the `Outcome` every method returns, and the records that cross the wire.
The gateway serves the contracts and gives each action an executor, the SDK, CLI and terminal hold the contracts and
read the records, and the harness builds the records. You come here to add an action or a state member, or to read the
exact shape of what a client sends and receives. Start at a `contract.ts`, such as
[gateway/contract.ts](/dimos/agents/shared/protocol/gateway/contract.ts), and at [actions/catalog.ts](/dimos/agents/shared/protocol/actions/catalog.ts).

Source: [dimos/agents/shared/protocol/](/dimos/agents/shared/protocol/). TypeScript examples use imports relative to this source directory.
Save temporary example files there and run them from `dimos/agents` with `node --import=./test/web-sdk.mjs shared/protocol/demo_example.ts`.
Remove the temporary file with `rm shared/protocol/demo_example.ts` before running `npm run typecheck` or `npm test`.
Those checks treat saved examples as package code and may reject their imports.

## Built on Chord and TypeBox

[Chord](https://github.com/earendil-works/pi/tree/main/packages/chord) serves objects across processes, and
[TypeBox](https://github.com/sinclairzx81/typebox) writes their schemas. This folder uses:

- A contract is what an object exposes, declared with `remote(name, { version, input, host, variants })`. A client holds
  an object only when its copy of the contract carries the gateway's version.
- A variant, `owner` or `viewer`, is one set of state members and methods. The gateway picks the variant per client.
- A state member is declared with `carried<T>()`. The gateway produces it and clients trust it, so its TypeScript type
  is its schema.
- A method comes from an action. `methodsOf(group, variant)` turns a catalog group into the methods of a variant.
- Method arguments get a full TypeBox schema, because clients produce them and Chord checks every call against it.

## Quickstart

Declare an action and the contract that serves it, as [actions/lifecycle.ts](/dimos/agents/shared/protocol/actions/lifecycle.ts) and
[lifecycle/contract.ts](/dimos/agents/shared/protocol/lifecycle/contract.ts) do, then call it from a reference. On the gateway, the object that serves
this contract gives `bind` in [gateway/server/objects/lib/bind.ts](/dimos/agents/gateway/server/objects/lib/bind.ts) a
`Handlers<typeof NOTE_ACTIONS>`, one executor per action that returns `done(value)` or `refused(refusal)`, and the
gateway fails to compile until the executor exists. The snippets under Usage continue from these declarations.

```ts
import type { ActionGroup } from "./actions/base.ts";
import type { EpochMs } from "./time.ts";
import type { Ref } from "@earendil-works/chord";
import { Type } from "typebox";
import { action } from "./actions/lib/action.ts";
import { carried } from "./schema.ts";
import { hostNamed, remote } from "@earendil-works/chord";
import { methodsOf } from "./actions/methods.ts";
import { rules } from "./actions/standing.ts";

type Note = { readonly text: string; readonly at: EpochMs };

export const NOTE_ACTIONS = {
    write: action({
        args: Type.Object({ text: Type.String({ minLength: 1, maxLength: 2000 }) }),
        value: carried<Note>(),
        executor: "gateway",
        access: "owner",
        retry: "unknown",
        rule: rules.open,
    }),
} as const satisfies ActionGroup;

export const NoteState = { note: carried<Note | null>() } as const;

export const NoteContract = remote("dimos.note", {
    version: 1,
    input: Type.Object({}),
    host: hostNamed("gateway"),
    variants: {
        owner: { state: NoteState, methods: methodsOf(NOTE_ACTIONS, "owner") },
        viewer: { state: NoteState },
    },
});

export async function saveNote(note: Ref<typeof NoteContract>, text: string): Promise<string> {
    if (note.variant !== "owner") return "viewing";
    const outcome = await note.write({ text });
    return outcome.ok ? `saved at ${outcome.value.at}` : outcome.refusal.code;
}
```

## Usage

**Refuse an action while dimOS is down.** Every action names a `rule`, a function of the arguments and the caller's
`Standing` that returns a `Refusal` or null. The gateway builds the standing for each call from the caller's role, the
dimOS link and the conversation's state, and runs the rule before the executor. Here the link is down, so the rule
answers with `dimos_unavailable` and the executor is skipped.

```ts
import type { Standing } from "./actions/standing.ts";
import { epochMs } from "./time.ts";

const guarded = action({ ...NOTE_ACTIONS.write, rule: rules.dimosUp });
const standing: Standing = {
    role: "owner",
    link: { state: "down", error: "connection refused", retryAt: epochMs(Date.now() + 5_000) },
    conversation: null,
};
console.log(guarded.rule({ text: "robot, where are you?" }, standing));
```

**Make an action safe to retry.** Give the arguments a `request` id and set `retry: "by_request_id"`. The client makes
the id with `newActionRequest()` before the first try and sends the same arguments again after a dropped link, and the
gateway answers the retry with the first call's outcome, which it keeps for five minutes. The type allows
`by_request_id` on an action whose arguments carry `request`.

```ts
import { ActionRequestIdSchema } from "./args.ts";

const retried = action({
    args: Type.Object({ request: ActionRequestIdSchema, text: Type.String({ minLength: 1 }) }),
    value: carried<Note>(),
    executor: "gateway",
    access: "owner",
    retry: "by_request_id",
    rule: rules.open,
});
console.log(retried.retry);
```

**Read a state member's type.** A member's TypeScript type is `Static` of its schema, so a view model takes the same
type the gateway produces.

```ts
import type { Static } from "typebox";
import { ConversationState } from "./conversation/contract.ts";

type Queue = Static<typeof ConversationState.queue>;
const empty: Queue = { items: [] };
console.log(empty.items.length);
```

## API

| Export                                                                                                   | File                                                                                                                                                                                         | What it does                                                                                                                                                                                                        |
| -------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `LifecycleContract`, `GatewayContract`, `ConversationContract`, `InventoryContract` | `*/contract.ts` | The served objects, with their state records beside them. Their members are in [Agent gateway](/docs/usage/agent/gateway.md). |
| `CATALOG`, `LIFECYCLE_ACTIONS`, `GATEWAY_ACTIONS`, `CONVERSATION_ACTIONS`                                | [actions/](/dimos/agents/shared/protocol/actions/catalog.ts)                                                                                                                                                               | The action groups. Each entry has `args`, `value`, `executor`, `access`, `retry` and `rule`. Every action is `owner` except `image`, which is `any` and so on both variants.                                        |
| `methodsOf(group, variant)`, `OwnerMethods`, `ViewerMethods`                                             | [actions/methods.ts](/dimos/agents/shared/protocol/actions/methods.ts)                                                                                                                                                     | A group as the Chord methods of one variant: all of them for `owner`, the `any` ones for `viewer`.                                                                                                                  |
| `rules`, `Standing`, `conversationStanding`                                                              | [actions/standing.ts](/dimos/agents/shared/protocol/actions/standing.ts)                                                                                                                                                   | The rules `open`, `dimosUp`, `idle`, `queued`, `userMessage`, `inRange`, `detachable` and `moved`, and what they read: the dimOS link and the conversation's transcript, run, queue and detachable calls. |
| `newUserRequest()`, `newActionRequest()`                                                                 | [actions/request.ts](/dimos/agents/shared/protocol/actions/request.ts)                                                                                                                                                     | A `UserRequestId` for `send` and `edit`, and an `ActionRequestId` for every other retried action.                                                                                                                   |
| `carried<T>()`, `nullable(schema)`                                                                       | [schema.ts](/dimos/agents/shared/protocol/schema.ts)                                                                                                                                                                       | A gateway-produced value with its type as the schema, and a nullable argument field.                                                                                                                                |
| `ConversationIdSchema`, `EntryIdSchema`, `CallIdSchema`, `ModelRefSchema`, `JobNameSchema` and the rest  | [args.ts](/dimos/agents/shared/protocol/args.ts)                                                                                                                                                                           | The argument schemas the actions share: ids, model refs and thinking levels.                                                                                                                                        |
| `Outcome<T>`, `Refusal`, `done`, `refused`                                                               | [outcome.ts](/dimos/agents/shared/protocol/outcome.ts)                                                                                                                                                                     | Every method's result, and every reason an action was refused, as data. `refusalText` in shared/view-models turns a refusal into the sentence every frontend shows.                                                 |
| `Live<T>`, `Status<T>`                                                                                   | [status.ts](/dimos/agents/shared/protocol/status.ts)                                                                                                                                                                       | A changing value as `value` and `subscribe`, which the gateway feeds into cells and the harness owns as a `Status`.                                                                                                 |
| `EpochMs`, `epochMs`                                                                                     | [time.ts](/dimos/agents/shared/protocol/time.ts)                                                                                                                                                                           | Milliseconds since the epoch, branded.                                                                                                                                                                              |
| `LinkStatus`, `describeLink`                                                                             | [gateway/link.ts](/dimos/agents/shared/protocol/gateway/link.ts)                                                                                                                                                           | The dimOS link as the gateway reports it, and its one-line description.                                                                                                                                             |
| The records                                                                                              | [conversation/](/dimos/agents/shared/protocol/conversation/state.ts), [runtime/](/dimos/agents/shared/protocol/runtime/event.ts), [jobs/](/dimos/agents/shared/protocol/jobs/state.ts), [auth.ts](/dimos/agents/shared/protocol/auth.ts), [streams.ts](/dimos/agents/shared/protocol/streams.ts), [media.ts](/dimos/agents/shared/protocol/media.ts), [tools/](/dimos/agents/shared/protocol/tools/details.ts) | What state members and inputs carry: the transcript and queue, inputs and images, events, modules, the inventory, runs and loads, jobs, accounts, streams, media and tool details.                                  |

Two facts a change here depends on:

- Adding a method or a state member keeps a contract's `version`. Removing, renaming or retyping a member bumps it. The
  gateway serves one version, so a client built on another fails to hold the object with `contract_mismatch`, which the
  SDK reports as the refusal `other_build`. `LifecycleContract` stays at version 1, so `dimos agent status` and `stop`
  work against any build.
- A new action is complete when its entry is in a group, its executor is in the served object's `Handlers`, its rule has
  a test in [actions/catalog.test.ts](/dimos/agents/shared/protocol/actions/catalog.test.ts), and any new `Refusal` variant has its sentence in
  `refusalText`. The compiler enforces the second and the fourth.
