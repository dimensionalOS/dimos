# Chord objects the gateway serves

This folder holds every Chord object a client can hold: the gateway's shared state, one conversation, the gateway
process and the inventory. `GatewayServer.serve` in [host.ts](../host.ts) calls each `serve…` function in its `OBJECTS`
list once, on the gateway's one Chord host. Each function registers a factory with `host.serve`. The four served
objects implement contracts from [shared/protocol](../../../shared/protocol/README.md), and store.ts and room.ts declare
local types of their own. The factories read the harness and the gateway's
stores through `ObjectDeps` and turn catalog actions into methods with `bind`. [base.ts](base.ts) defines `ServeObject`,
the type every file here implements.

## Built on Chord

[Chord](https://github.com/earendil-works/pi/tree/main/packages/chord) runs objects that hold state and methods at an
address and carries their state and calls between hosts over links. The gateway is one Chord host. Each terminal runs a
client host that connects to it over the Unix socket. Chord's usage guide is `docs/guide.md` in the package. This
section covers the parts this folder uses.

### Contracts, factories and variants

A contract names a type, its input, the host it lives on and its variants. `remote()` declares one in shared/protocol,
such as `ConversationContract` in [conversation/contract.ts](../../../shared/protocol/conversation/contract.ts). The
input is the object's address, for example `{ conversation, role }`. Every contract here says `hostNamed("gateway")`, so
a client never picks the host. A variant is the shape one principal gets, its state members and its methods. Our
contracts have an `owner` and a `viewer` variant, or `viewer` alone.

`host.serve(Contract, factory, options)` registers the code for a type. Chord calls the factory with `input`,
`principal` and `self` the first time a principal asks for an address. The factory runs synchronously. It declares what
the object needs through `self` and returns the implementation, which is the `variant` it picked, the state members and
the methods. Chord makes one object per type, input and principal. `principalKey` in
[principal.ts](../../auth/principal.ts) maps a terminal to its terminal id, so a terminal that reconnects reaches the
same objects.

A factory picks its variant from `grantOf(principal)`. A terminal on the 0600 socket is a `terminal` principal, and its
grant is `owner`. A `page` principal carries an `owner` or `viewer` grant from its token, but no listener creates one
yet. The conversation object also lowers the variant to the role the client asks for. `dimos agent --view` attaches
read-only that way.

### What a factory declares

The factory's `self` declares everything the object keeps:

- `self.state(name, schema, initial)` makes a cell, one observable state member. Pass `{ init }` in place of a value to
  read the first value from a source before the object goes live. `cell.replace(ctx, value)` publishes a new value. A
  client gets a snapshot, then deltas. `replace` sends a structural diff when the new value shares containers with the
  old one.
- `self.derive(sources, fn)` makes a readable that `fn` computes synchronously from other readables.
- `self.resource(name, { version, open, close })` opens a watch or a subscription before the object goes live and closes
  it after the object is disposed.
- `self.on(name, source, handler)` reacts to a source once the object is live. A readable delivers its current value,
  then each new value. An event source delivers events only. With a `select` argument, Chord subscribes to
  `select(value)`. This folder uses `select` to turn a resource's value into events.
- `self.task(fn)` runs background work while the object is live. Chord aborts its `ctx.signal` when the object is
  disposed.
- `self.get(Type, input)` holds a reference to another object for as long as this object lives.

Chord activates an object in a fixed order. It runs the factory, opens the resources, waits for the references, runs
each cell's `init` and then makes the object live. Listeners and tasks start only at that last step. A change that
arrives between a cell's `init` and the start of its listener would be lost. [feed.ts](lib/feed.ts) and
[commits.ts](lib/commits.ts) close that gap. Inside the factory body you cannot read what you declared. Read cells and
references in methods, listeners, tasks and `init`.

### Lifetime and local objects

A client's reference keeps an object alive until the client releases it or disconnects. When the last reference goes,
Chord disposes the object, after the `linger` its `serve` options name. It aborts the tasks, removes the listeners and
closes the resources. `lifetime: "host"` instead makes one object when the type is served and keeps it until the host
closes. `host.ready()` waits until that object is live.

`local()` declares a type that only code inside the host can reach. A local type has no variants. Code inside an object
acts as the host, so when the objects of two clients call `self.get(GatewayStore, {})`, both reach one instance. This
folder shares state that way. Each served object is a view for one principal. State that every principal shares lives in
a local object that every view holds.

### Who is calling

Inside an object's methods, listeners and tasks, `ctx.principal` is the gateway's own principal, `{ kind: "gateway" }`.
It is never the client. Chord's guide states the rule under "Principals and variants". Code inside a host acts as its
`self`. The caller is the factory's `principal` argument. A method that needs the caller closes over that argument.
[gateway.ts](gateway.ts) does so when it passes `principal` to `deps.logins.start`.

Chord gives us objects, variants, lifetime and state sync. This folder adds the `ServeObject` family, so host.ts serves
every type with one loop. It adds `bind`, which turns a catalog group into Chord methods with a rule check, retries and
refusals. It adds helpers that feed cells from harness state without losing a change during activation.

## The API

### ServeObject and ObjectDeps

[base.ts](base.ts) exports both types. `ServeObject` is `(host: Host<Principal>, deps: ObjectDeps) => void`. Each file
in this folder exports one, and it calls `host.serve` for its types. `ObjectDeps` is everything a factory and its
methods may use. `runGateway` in [gateway/main.ts](../../main.ts) builds it once per process.

| Field      | Type                                                     | What the objects use it for                                                                                                                               |
| ---------- | -------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `agent`    | `Agent` from [harness/main.ts](../../../harness/main.ts) | The harness, with pi-durable's `Harness`, the input gate, conversations, history, the runtime client, jobs, the detacher, the DimOS link, the loader, the run watcher and the state directory. |
| `models`   | pi-ai `Models`                                           | `updateSettings` refuses a model that `getModel` does not know.                                                                                           |
| `catalog`  | `ModelCatalog`                                           | The `accounts` cell and `logout`.                                                                                                                         |
| `logins`   | `LoginFlows`                                             | The `loginFlow` cell of each principal, `login` and `answerLogin`.                                                                                        |
| `config`   | `ConfigStore`                                            | The `settings` cell and `updateSettings`.                                                                                                                 |
| `process`  | `GatewayProcess`                                         | The lifecycle object's `process` cell.                                                                                                                    |
| `replies`  | `Replies`                                                | The first outcome of each `by_request_id` call.                                                                                                           |
| `report`   | `(error: unknown) => void`                               | Writes a failure to the gateway log.                                                                                                                      |
| `shutdown` | `() => void`                                             | Stops the gateway.                                                                                                                                        |

### The OBJECTS list

`OBJECTS` in [host.ts](../host.ts) is the family's one consumer. It lists `serveStore`, `serveRoom`, `serveLifecycle`,
`serveGateway`, `serveConversation` and `serveInventory`, the local types first. `GatewayServer.serve`
creates the host, calls each function and awaits `host.ready()` before it accepts a connection. The store has
`lifetime: "host"`, so it has scanned every conversation before the first client connects. `GatewayServer.close` closes
the host, which disposes every object and closes its resources.

### Served objects

Each served object implements one contract from shared/protocol. Clients hold them through the SDK.

| File                               | Function            | Contract               | Input                    | Variants      | State                                          | Methods                                        |
| ---------------------------------- | ------------------- | ---------------------- | ------------------------ | ------------- | ---------------------------------------------- | ---------------------------------------------- |
| [lifecycle.ts](lifecycle.ts)       | `serveLifecycle`    | `LifecycleContract`    | `{}`                     | owner, viewer | `process`                                      | `LIFECYCLE_ACTIONS` on owner                   |
| [gateway.ts](gateway.ts)           | `serveGateway`      | `GatewayContract`      | `{}`                     | owner, viewer | `GatewayState` from the store, and `loginFlow` | `GATEWAY_ACTIONS` on owner                     |
| [conversation.ts](conversation.ts) | `serveConversation` | `ConversationContract` | `{ conversation, role }` | owner, viewer | `ConversationState` from the room              | `CONVERSATION_ACTIONS`, only `image` on viewer |
| [inventory.ts](inventory.ts)       | `serveInventory`    | `InventoryContract`    | `{}`                     | viewer        | `inventory` from the store                     | none                                           |

`serveLifecycle` serves the gateway process's identity and its one action, `stop`. `LifecycleContract` is frozen at
version 1, so `dimos agent status` and `stopGateway` in cli/process.ts work against a gateway of any build.

`serveGateway` forwards every `GatewayState` member from the store. It adds `loginFlow`, fed from
`deps.logins.of(principal)`, so only the terminal that started a login sees its prompts. The object lingers 30 s after
its last reference, so a terminal that reconnects keeps its login. Its disposal cancels the principal's running login,
because nobody can answer that prompt any more. A prompt left unanswered for 10 minutes cancels its login too. Its
executors return the action's result. Clients see the effect through the store's cells. A restarted module arrives
through the runtime view, and a new job through the jobs document.

`serveConversation` holds the conversation's room and the store. Its variant is `owner` only when the client asks for
`role: "owner"` and its grant is `owner`. Every owner of a conversation can send input to it. Every served conversation
object, owner or viewer, listens to the room's transcript and marks the newest notice input seen with
`Conversations.markSeen`. That clears the conversation's unread count in the switcher rows.

`serveInventory` forwards the store's `inventory` member. It is its own object because RPC docstrings make the inventory
the largest gateway value. The terminal reads it for its relay endpoint and shares it with the `/runtime` tabs.

### Local objects

Two local types hold the state the served objects show.

`GatewayStore` in [store.ts](store.ts) has `lifetime: "host"`, so there is one per gateway, and it keeps collecting
while no client is attached. Its members are `GatewayState` plus `inventory`:

| Member                                                    | Source                                                                                         |
| --------------------------------------------------------- | ---------------------------------------------------------------------------------------------- |
| `link`, `load`, `runs`                         | `feed` of the `status` of `agent.connection`, `agent.loader`, `agent.runs` |
| `settings`, `accounts`                                    | `feed` of `deps.config.status` and `deps.catalog.status`                                       |
| `conversations`                                           | `conversationRows`, the switcher rows                                                          |
| `jobs`                                                    | `jobRows`                                                                                      |
| `modules`, `streams`, `inventory` | derived from the one cell `runtimeView` keeps                                                  |

`ConversationRoom` in [room.ts](room.ts) has one object per conversation, with input `{ conversation }` and the members
of `ConversationState`. It is referenced, with a 30 s linger, so a client that switches away and back, or reconnects,
does not mount pi-durable's view again. It holds:

- A `watch` resource over pi-durable's `watch()` of the conversation. Each frame replaces `record`, `transcript`,
  `live`, `queue`, `usage` and `agent`. `live`, `queue`, `usage` and `agent` are the `pi.live`, `pi.inbox`, `pi.usage`
  and `pi.agent` documents as pi-durable keeps them. `transcript` is `projectTranscript` of the view, replaced with a
  structural diff, so an appended entry reaches clients as a splice.
- A `commits` resource from `commitEvents`. A listener folds the settled input submissions of this conversation into
  `outcomes` and keeps the newest 50.
- `detachable`, a `feed` of `agent.detacher.of(conversation)`.
- `moved`, a `feed` of `agent.detacher.moved(conversation)` with `{ diff: true }`, so a moved call's growing output
  reaches clients as appends.
- `modules`, `jobs` and `streams`, derived from the store's cells and filtered to what this
  conversation owns.

Preview frames use the terminal's Web SDK session. The gateway exposes the relay endpoint through inventory.

### bind and Handlers

[lib/bind.ts](lib/bind.ts) connects a catalog group from shared/protocol/actions to executors.

`Handlers<G>` is a record with one executor per entry of the group `G`. The compiler checks it is exhaustive, so a new
catalog entry does not compile until its executor exists. An executor takes the action's arguments and a `HandlerCall`.
It returns a promise of an `Outcome` of the action's value. `HandlerCall.context` is a `Context` from
`@earendil-works/chord/context` that carries the call's abort signal, or, for a `by_request_id` action, the gateway's,
which only shutdown aborts. Pass it to every harness and pi-durable call.

`bind(object, group, handlers, standing, replies, variant)` returns the Chord methods for one variant. `object` is the
group's name in `CATALOG`, one of `"conversation"`, `"gateway"` and `"lifecycle"`. `standing` is a function that reads
the server state the rules need, the `Standing` from [standing.ts](../../../shared/protocol/actions/standing.ts). The
`owner` variant gets every action. The `viewer` variant gets only the actions whose `access` is `"any"`. The result has
the shape `methodsOf` puts in the contract, so the factory spreads it into its implementation.

Each bound method runs these steps in order:

1. If the entry's `retry` is `by_request_id`, it answers through `replies.once` under `<object>.<name>:<request>`. A
   repeated `request` gets the first call's outcome, a refusal too, before the rule sees the state that call left. A
   call still running is joined. The call runs in the gateway's context, so the caller's dropped link does not abort
   it.
2. It runs the entry's `rule` over the arguments and `standing()`. A refusal returns at once, and the executor never
   runs.
3. It runs the executor.
4. If the executor throws `DimosUnavailable`, it returns a `dimos_unavailable` refusal worded by `describeLink`. Any
   other throw propagates to Chord, and the client's call rejects.

`Replies` in [replies.ts](../replies.ts) is one cache per gateway process. `runGateway` keeps each outcome for
`REPLIES_KEPT_MS`, five minutes, and gives it the context its calls run in, which shutdown aborts. The cache lives
outside the objects because Chord can dispose a served object and make it again between a dropped call and its retry.
A call that threw is forgotten, so its retry runs again.

### Helpers in lib/

These files are private to this folder. The boundaries test refuses an import of a lib/ file from outside its parent.

- [commits.ts](lib/commits.ts) exports `commitEvents(harness)`. It subscribes to pi-durable's commit publications when
  called and keeps them in `pending` until a listener subscribes. Every caller opens it as a resource, so the buffer
  covers the gap between a cell's `init` and the start of its listener. On a reload, Chord subscribes the new listener
  before it unsubscribes the old one, so an unsubscribe clears only its own listener.
- [conversations.ts](lib/conversations.ts) exports `conversationRows(self, harness, conversations)`, which declares the
  store's `conversations` cell. Its `init` reads the `dimos.seen` cursors, scans every conversation record and its
  entries in one transaction, and snapshots each conversation's documents. It then drops the buffered publications that
  arrived before that transaction began, because the scan already includes them. A listener folds each later publication
  and counts notice entries past the seen cursor as unread. When the `dimos.seen` document changes, it re-reads the
  cursors after the commit returns. `foldPublication(rows, publication, seen, now)` is the pure fold. It applies new
  conversation records first, then entries and the `pi.live`, `pi.inbox`, `pi.usage` and `pi.agent` documents. A row
  that did not change keeps its object, so the `pi.live` partials that arrive every 100 ms send an empty diff.
- [feed.ts](lib/feed.ts) exports `feed(self, name, schema, source)`. It makes a cell that holds `source.value` and
  replaces it on every change. `source` is a `Live`, the read side of a `Status` from
  [shared/protocol/status.ts](../../../shared/protocol/status.ts). `DimosConnection`, `RuntimeLoader`, `RunWatcher`,
  `ConfigStore` and `ModelCatalog` expose one as `status`. `LoginFlows.of` returns one per principal and
  `Detacher.of` one per conversation. A `Live` calls a new subscriber at once with its current value, so a change
  between `init` and the start of the listener is not lost.
- [forward.ts](lib/forward.ts) exports `forward(ref, keys)`. It returns the named state members of a held reference as
  the object's own, the same readables with no copy. A reference cannot be spread, so the keys come from the contract's
  state record, as in `Object.keys(GatewayState)`.
- [items.ts](lib/items.ts) turns stored entries into the transcript clients see. `projectTranscript(view, memo, budget)`
  walks the entries newest first and keeps items until one does not fit `TRANSCRIPT_BUDGET`, 2,000 items or
  `8 * 2 ** 20` characters of JSON. The kept items are contiguous and come back oldest first. `omitted` counts the older
  rest. `transcriptItem` maps one entry, strips the `data` of every image block and turns an input's images into
  `ImageRef` addresses. A `TranscriptMemo` keeps one item per entry record, so successive frames share item objects and
  Chord sends ops. `imageAt(entry, at)` returns the stored image block at an address, or a `not_found` refusal, for the
  conversation's `image` action. `toolResult` parses a stored tool result's details with `parseToolDetails`.
- [jobs.ts](lib/jobs.ts) exports `jobRows(self, harness)`, which declares the store's `jobs` cell from the jobs document
  and the checkpoint of each job's task, which holds its next due time. A task change updates that job's due time. A
  change to the jobs document lists the jobs again after the commit returns. `jobRow(job, nextDue)` builds one row with
  the next three due times from the cron expression. The job actions return it too.
- [outcomes.ts](lib/outcomes.ts) converts harness results into an `Outcome` with `fromModuleOutcome`, `fromJobChange`,
  `fromSubmit` and `fromBranch`. Each maps every result kind to `done` or one refusal.
- [runtime.ts](lib/runtime.ts) exports `runtimeView(self, link, options)`, which declares one cell with the store's
  DimOS rows. Modules and the inventory are parsed from HarnessRuntime's inventory, and streams follow
  `RuntimeClient.streams`. Each read also connects the relay, or drops it when the inventory names none. It reads the
  inventory when the link changes, when
  HarnessRuntime reports a status change, and every `pollMs` while the link is up. The store passes 30 s. A read that is
  asked for while one runs makes one more read after it. While the link is down, it lists the agent modules from their
  records on disk with state `unknown` and marks the last inventory `current: false`.

## Examples

### Serve the lifecycle object

`serveLifecycle` is the smallest object with an action. A client that holds `LifecycleContract` reads the `process`
cell, and an owner can call `stop`. This is [lifecycle.ts](lifecycle.ts) as it stands.

```ts
import type { GatewayProcess } from "../../../shared/protocol/lifecycle/contract.ts";
import type { Handlers } from "./lib/bind.ts";
import type { ServeObject } from "./base.ts";
import type { Standing } from "../../../shared/protocol/actions/standing.ts";
import { LIFECYCLE_ACTIONS } from "../../../shared/protocol/actions/lifecycle.ts";
import { LifecycleContract } from "../../../shared/protocol/lifecycle/contract.ts";
import { bind } from "./lib/bind.ts";
import { carried } from "../../../shared/protocol/schema.ts";
import { done } from "../../../shared/protocol/outcome.ts";
import { grantOf } from "../../auth/principal.ts";

const STOP_AFTER_MS = 100;

export const serveLifecycle: ServeObject = (host, deps) =>
    host.serve(LifecycleContract, (_input, principal, self) => {
        const process = self.state("process", carried<GatewayProcess>(), deps.process);
        if (grantOf(principal) === "viewer") return { variant: "viewer", process };
        const standing = (): Standing => ({
            role: "owner",
            link: deps.agent.connection.status.value,
            conversation: null,
        });
        const handlers: Handlers<typeof LIFECYCLE_ACTIONS> = {
            stop: async () => {
                // A tick later than the result: closing the links at once can beat the result frame onto the wire when other
                // clients are attached, and the caller then sees `unknown_outcome` for a stop that happened.
                setTimeout(deps.shutdown, STOP_AFTER_MS);
                return done(null);
            },
        };
        return {
            variant: "owner",
            process,
            ...bind("lifecycle", LIFECYCLE_ACTIONS, handlers, standing, deps.replies, "owner"),
        };
    });
```

Line by line:

- `host.serve(LifecycleContract, …)` registers the factory. The input is always `{}`, so Chord makes one object per
  principal.
- `self.state("process", …, deps.process)` declares a cell with a fixed first value. `carried()` is a schema that trusts
  the value the gateway produces.
- A `viewer` grant returns the cell alone, because the viewer variant has no methods.
- `standing` reads the state the rules need at call time. `stop`'s rule is `rules.open`, so nothing refuses it, but
  `bind` takes a standing for every group.
- `handlers` has one executor because `LIFECYCLE_ACTIONS` has one entry. `stop` schedules `deps.shutdown` 100 ms later
  and returns `done(null)`.
- `bind(…, "owner")` returns `{ stop }` with the rule check and the `DimosUnavailable` refusal. `stop`'s `retry` is
  `unknown`, so it skips the replies cache.

### Feed cells from the harness

This object combines a resource, a `feed` and a derived cell. A client sees the DimOS link, the tool calls of one
conversation that can move to the background, their count, and the sequence number of the newest commit that added an
entry to the conversation. A real contract belongs in shared/protocol. This one sits in the module so that the block is
complete. [room.ts](room.ts) combines the same three declarations.

```ts
import type { Seq } from "@earendil-works/pi-durable";
import type { ServeObject } from "./base.ts";
import { ConversationIdSchema } from "../../../shared/protocol/args.ts";
import { ConversationState } from "../../../shared/protocol/conversation/contract.ts";
import { GatewayState } from "../../../shared/protocol/gateway/contract.ts";
import { Type } from "typebox";
import { carried } from "../../../shared/protocol/schema.ts";
import { commitEvents } from "./lib/commits.ts";
import { feed } from "./lib/feed.ts";
import { hostNamed, remote } from "@earendil-works/chord";

const ActivityState = {
    link: GatewayState.link,
    detachable: ConversationState.detachable,
    backgroundable: carried<number>(),
    lastCommit: carried<Seq | null>(),
} as const;

export const ActivityContract = remote("dimos.activity", {
    version: 1,
    input: Type.Object({ conversation: ConversationIdSchema }),
    host: hostNamed("gateway"),
    variants: { viewer: { state: ActivityState } },
});

export const serveActivity: ServeObject = (host, deps) =>
    host.serve(ActivityContract, ({ conversation }, _principal, self) => {
        const { agent } = deps;
        const commits = self.resource("commits", {
            version: 1,
            open: () => commitEvents(agent.harness),
            close: (events) => events.close(),
        });
        const lastCommit = self.state("lastCommit", ActivityState.lastCommit, null);
        self.on(
            "commits",
            commits,
            (events) => events,
            (publication, ctx) => {
                const touched = publication.changes.some(
                    (change) => change.type === "entry" && change.value.conversationId === conversation,
                );
                if (touched) lastCommit.replace(ctx, publication.seq);
            },
        );
        const detachable = feed(self, "detachable", ActivityState.detachable, agent.detacher.of(conversation));
        return {
            variant: "viewer",
            link: feed(self, "link", ActivityState.link, agent.connection.status),
            detachable,
            backgroundable: self.derive([detachable], (calls) => calls.length),
            lastCommit,
        };
    });
```

The `commits` resource opens before the object goes live. `commitEvents` keeps what arrives until the `commits` listener
starts, so a commit made during activation still reaches the handler. Each `feed` call declares a cell and a listener
named after it. `backgroundable` has no name and is recomputed whenever `detachable` changes. To serve the object, add
its `ServeObject` to `OBJECTS`.

### Share a room between clients

This is [conversation.ts](conversation.ts) without the listener that marks notices seen. It gets the conversation's room
and the store, forwards the room's members, and binds the conversation actions for the variant it picks. An owner gets
every method. A viewer gets the same state and only `image`.

```ts
import type { Handlers } from "./lib/bind.ts";
import type { ServeObject } from "./base.ts";
import type { Standing } from "../../../shared/protocol/actions/standing.ts";
import { CONVERSATION_ACTIONS } from "../../../shared/protocol/actions/conversation.ts";
import { ConversationContract, ConversationState } from "../../../shared/protocol/conversation/contract.ts";
import { ConversationRoom } from "./room.ts";
import { GatewayStore } from "./store.ts";
import { bind } from "./lib/bind.ts";
import { conversationStanding } from "../../../shared/protocol/actions/standing.ts";
import { done, refused } from "../../../shared/protocol/outcome.ts";
import { forward } from "./lib/forward.ts";
import { fromBranch, fromSubmit } from "./lib/outcomes.ts";
import { grantOf } from "../../auth/principal.ts";
import { imageAt } from "./lib/items.ts";

const SHARED = Object.keys(ConversationState) as (keyof typeof ConversationState)[];

export const serveConversation: ServeObject = (host, deps) =>
    host.serve(ConversationContract, ({ conversation, role }, principal, self) => {
        const room = self.get(ConversationRoom, { conversation });
        const store = self.get(GatewayStore, {});
        const state = forward(room, SHARED);
        const owner = role === "owner" && grantOf(principal) === "owner";
        const standing = (): Standing => ({
            role: owner ? "owner" : "viewer",
            link: store.link.value,
            conversation: conversationStanding(conversation, {
                transcript: room.transcript.value,
                live: room.live.value,
                queue: room.queue.value,
                detachable: room.detachable.value,
                moved: room.moved.value,
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
            abort: async ({ request }, call) => {
                const withdrawn = await agent.inputs.withdrawUser(conversation, request, "abort", call.context);
                if (withdrawn.aborts !== null) await agent.harness.abortTask(withdrawn.aborts, call.context);
                return done(withdrawn.texts);
            },
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
            stopBackground: async ({ call: target }) =>
                agent.detacher.stop(conversation, target) ? done(null) : refused({ code: "not_running", call: target }),
            image: async ({ at }, call) =>
                imageAt(await agent.harness.commit((tx) => tx.entry(at.entry), call.context), at),
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
```

`self.get(ConversationRoom, { conversation })` acts as the host, so every client of one conversation shares one room and
one pi-durable watch. `forward(room, SHARED)` returns the room's cells, so the served object keeps no state of its own.
`standing` reads the room's values each time a method runs, so `rules.idle` sees the current `pi.live` run. `Handlers`
makes you write all ten executors, even though the viewer variant exposes only `image`. Each executor passes
`call.context` on, so the call's abort signal, or the gateway's for a `by_request_id` action, reaches pi-durable.

### Add a served object

1. Declare the contract in `shared/protocol/<name>/contract.ts` with `remote()` and `hostNamed("gateway")`. Build its
   state record with `carried()`. If it has actions, add a group in shared/protocol/actions, add the group to `CATALOG`
   in [catalog.ts](../../../shared/protocol/actions/catalog.ts) and take the variant's methods from `methodsOf`. `bind`
   accepts only a group named in `CATALOG`.
2. Write `gateway/server/objects/<name>.ts`. Export a `ServeObject` typed from base.ts that calls `host.serve` with the
   contract. Keep shared state in a local type that the served object gets with `self.get`, as store.ts and room.ts do.
   Put a helper that serves nothing in lib/.
3. Import the function in [host.ts](../host.ts) and add it to `OBJECTS`, after the local types.
4. Run `node --import ./test/web-sdk.mjs --test test/boundaries.test.ts` from dimos/agents. The `FAMILIES` entry for `gateway/server/objects` in
   [boundaries.test.ts](../../../test/boundaries.test.ts) already names host.ts as the consumer. The test fails when a
   file in this folder does not import base.ts, or when host.ts does not import it as a value. Add the contract to the
   list in the test "no served contract names a state member and a method alike".
5. Hold the object from a client with `host.get(Contract, input)` through the SDK.

## Files

The folder is a family. [base.ts](base.ts) defines its type, and every other `.ts` file here serves one or more Chord
types and is listed in `OBJECTS`.

- [base.ts](base.ts): `ServeObject` and `ObjectDeps`.
- [store.ts](store.ts): `GatewayStore`, the local type with the gateway's shared cells, and `serveStore`.
- [room.ts](room.ts): `ConversationRoom`, the local type with one conversation's cells, and `serveRoom`.
- [lifecycle.ts](lifecycle.ts): `serveLifecycle`, the gateway process and `stop`.
- [gateway.ts](gateway.ts): `serveGateway`, the store's members, the login flow and the gateway actions.
- [conversation.ts](conversation.ts): `serveConversation`, one conversation for one principal.
- [inventory.ts](inventory.ts): `serveInventory`, the inventory for the `/runtime` tabs.

lib/ holds helpers private to this folder:

- [lib/bind.ts](lib/bind.ts): `bind`, `Handlers`, `HandlerCall`, `OwnerBound` and `ViewerBound`.
- [lib/commits.ts](lib/commits.ts): `commitEvents`, commit publications buffered until a listener starts.
- [lib/conversations.ts](lib/conversations.ts): `conversationRows` and `foldPublication`.
- [lib/feed.ts](lib/feed.ts): `feed`, a cell bound to a `Live`.
- [lib/forward.ts](lib/forward.ts): `forward`, a held reference's members as an object's own.
- [lib/items.ts](lib/items.ts): `projectTranscript`, `TRANSCRIPT_BUDGET`, `transcriptItem`, `toolResult` and `imageAt`.
- [lib/jobs.ts](lib/jobs.ts): `jobRows` and `jobRow`.
- [lib/outcomes.ts](lib/outcomes.ts): `fromModuleOutcome`, `fromJobChange`, `fromSubmit` and `fromBranch`.
- [lib/runtime.ts](lib/runtime.ts): `runtimeView`, the DimOS rows from HarnessRuntime's inventory or from disk.

[lib/bind.test.ts](lib/bind.test.ts) covers the rule check, the replies cache, the `DimosUnavailable` refusal and the
viewer's methods. [lib/conversations.test.ts](lib/conversations.test.ts) covers `foldPublication`.
[lib/items.test.ts](lib/items.test.ts) covers `projectTranscript` and `imageAt`.

## See also

- [gateway/README.md](../../README.md) for the process that serves these objects, principals and `Replies`.
- [shared/protocol/README.md](../../../shared/protocol/README.md) for contracts, actions, rules and refusals.
- [sdk/README.md](../../../sdk/README.md) for the client side that holds these objects.
- [harness/README.md](../../../harness/README.md) for the `Agent` the executors call.
- [Attach to a conversation](../../../../../docs/usage/agent.md#attach-to-a-conversation) for `--view`.
