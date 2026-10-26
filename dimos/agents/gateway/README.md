# Gateway

The gateway is the agent daemon and the server every client talks to. It is one Node process that holds every
conversation, runs the model turns, tool calls, cron jobs and event-driven turns, and keeps going when every terminal
has closed and when DimOS restarts. A client connects to its Unix socket and holds Chord objects: the gateway's shared
state, one conversation, the process itself, a module's log, a stream preview and the inventory. This README is what you need to write
such a client. The entry for running the daemon is `runGateway` in [main.ts](main.ts).

## Built on Chord

[Chord](https://github.com/earendil-works/pi/tree/main/packages/chord) serves objects across processes. A client of the
gateway uses these concepts:

- A host owns objects. The gateway is one host, named `gateway`. A client runs its own host, which holds references.
- A link carries Chord's frames between two hosts. A client wraps its WebSocket in one with `webSocketLink`.
- A principal is who a link acts as. The gateway stamps it on each link from the socket URL.
- A reference is a client's handle on one object, taken with `host.get(Contract, input)` and ended with `release()`.
- A cell is one state member of a reference, read with `value` and followed with `subscribe`.
- A method is a call on a reference. Every gateway method resolves to an `Outcome`, done with a value or refused with a
  `Refusal`.

The contracts and the actions behind the methods are declared in [shared/protocol/](../shared/protocol/README.md).

## Quickstart

This connects to a running gateway with Chord and the `ws` package. It prints the gateway's pid and its link to DimOS,
sends one message to conversation `process.argv[2]` as owner, waits for its outcome, and disconnects.

```ts
import type { ConversationId } from "@earendil-works/pi-durable";
import { ConversationContract } from "../shared/protocol/conversation/contract.ts";
import { GatewayContract } from "../shared/protocol/gateway/contract.ts";
import { LifecycleContract } from "../shared/protocol/lifecycle/contract.ts";
import { WebSocket } from "ws";
import { createHost, webSocketLink } from "@earendil-works/chord";
import { describeLink } from "../shared/protocol/gateway/link.ts";
import { newUserRequest } from "../shared/protocol/actions/request.ts";
import { randomUUID } from "node:crypto";

const socket = new WebSocket(`ws+unix:/run/user/1000/dimos-agent.sock:/?terminal=${randomUUID()}&label=example`);
await new Promise<void>((resolve) => socket.addEventListener("open", () => resolve()));
const host = createHost({ name: "client", self: { kind: "gateway" }, kind: "client" });
host.connect(webSocketLink(socket), { principal: { kind: "gateway" } });

const lifecycle = await host.get(LifecycleContract, {});
console.log(`gateway pid ${lifecycle.process.value.pid}`);

const gateway = await host.get(GatewayContract, {});
console.log(describeLink(gateway.link.value));

const id = Number(process.argv[2]) as ConversationId;
const conversation = await host.get(ConversationContract, { conversation: id, role: "owner" });
if (conversation.variant !== "owner") throw new Error("attached as a viewer");
const sent = await conversation.send({
    request: newUserRequest(),
    text: "Where is the robot?",
    images: [],
    whenBusy: "followUp",
});
if (!sent.ok) throw new Error(sent.refusal.code);
await new Promise<void>((resolve) =>
    conversation.outcomes.subscribe((rows) => {
        if (rows.some((row) => row.submission === sent.value.submission)) resolve();
    }),
);

conversation.release();
gateway.release();
lifecycle.release();
await host.close();
```

## API

### The socket

The gateway listens on `$XDG_RUNTIME_DIR/dimos-agent.sock`, or `dimos-agent.sock` in `~/.local/state/dimos/agent/`
without that variable. The file is mode 0600, so every process that can open it is trusted and gets the `owner` grant.
The WebSocket URL carries two query parameters: `terminal`, a UUID that stays the same for the life of a client process,
and `label`, 1 to 32 of `[A-Za-z0-9_-]` naming the kind of client. The gateway keys objects by `terminal`, so a client
that reconnects with the same UUID reaches the same objects. A missing or malformed parameter closes the link with the
reason `unauthorized`. An upgrade before the objects are ready gets HTTP 503, so dial with backoff.

### The objects

| Contract               | Input                    | Variants                                        | State                                                                                                                                         | Actions                                                                                                                                                                                                           |
| ---------------------- | ------------------------ | ----------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `LifecycleContract`    | `{}`                     | `owner`, `viewer`                               | `process`: build, pid, start time                                                                                                             | `stop`                                                                                                                                                                                                            |
| `GatewayContract`      | `{}`                     | `owner`, `viewer`                               | `link`, `load`, `runs`, `conversations`, `modules`, `streams`, `jobs`, `settings`, `accounts`, `loginFlow` | `createConversation`, `login`, `answerLogin`, `logout`, `updateSettings`, `createModule`, `restartModule`, `unloadModule`, `deleteModule`, `createJob`, `updateJob`, `deleteJob`, `runJob` |
| `ConversationContract` | `{ conversation, role }` | `owner` when you ask for it, `viewer` otherwise | `record`, `transcript`, `live`, `queue`, `usage`, `agent`, `outcomes`, `detachable`, `modules`, `jobs`, `streams` | `send`, `withdraw`, `abort`, `configure`, `edit`, `summarizeUpTo`, `summarizeFrom`, `background`, and `image` on both variants                                                                                    |
| `InventoryContract`    | `{}`                     | `viewer`                                        | `inventory`                                                                                                                                   | none                                                                                                                                                                                                              |

Each contract file in [shared/protocol/](../shared/protocol/README.md) types every member and argument. A viewer variant
has the state and no owner methods. `loginFlow` is per principal, so only the client that started a login sees its
prompts. `InventoryContract` carries the full inventory, including the nullable relay endpoint. The terminal holds
it for its lifetime and uses that endpoint for its own Web SDK session. Preview frames travel directly from the relay
to the terminal.

### Semantics a client relies on

- A method resolves to `{ ok: true, value }` or `{ ok: false, refusal }`. It rejects only when Chord itself fails, with
  a `ChordError` whose `code` names the failure.
- An action whose catalog entry says `retry: "by_request_id"` takes a `request` id. Make the id once, before the first
  try. If the link drops mid-call, send the same arguments again once the link is live, and the gateway answers with the
  first call's outcome; it keeps each one for five minutes, and the dropped link does not abort the call. The other
  actions run once per call.
- A cell delivers its current value to a new subscriber at once, then every change. After a reconnect the gateway sends
  a fresh snapshot of each cell and replays nothing.
- `LifecycleContract` stays at version 1 forever, so any client can read `process.build` and call `stop` on a gateway of
  another build. Compare `process.build.build` with your own build to detect a mismatch before you hold
  `GatewayContract`, which may have changed.
- When the gateway shuts down it closes every link with the reason `stopping`. That close is final.

### Running it

`dimos agent gateway` runs `runGateway(launch)` from [main.ts](main.ts) in the foreground, and the systemd user service
runs that command on a robot. It exits 1 when serving fails or the process crashes, after writing the error to
`gateway.log`, so the service restarts it. `dimos agent`, `run` and `conversations` spawn it detached when nothing
answers the socket. `agentPaths(env)` in [config/paths.ts](config/paths.ts) gives the socket path and every other file
the gateway owns. `buildInfo()` in [config/build.ts](config/build.ts) is the build a client compares against. The rest
of this folder is the daemon's own machinery: the socket server and the retry cache in [server/](server/), who a
connection is and the model logins in [auth/](auth/), and the objects clients hold in
[server/objects/](server/objects/README.md).
