# Client SDK

The SDK is how a program talks to the gateway. Connecting with Chord alone gets you the gateway's objects over its
socket, as [Agent gateway](/docs/usage/agent/gateway.md) shows. The SDK adds what a long-lived client needs on top:
redialing with backoff under one terminal identity, a connection state to show, an `ended` signal for the end of the
client, one mapping from every failure to a `Refusal`, a safe retry for deduplicated actions, waits on cells, and live
stream frames. The `dimos agent` commands and the terminal are built on it. The entry is `connect` in
[main.ts](/dimos/agents/sdk/main.ts). Everything except [node.ts](/dimos/agents/sdk/node.ts), [relay.ts](/dimos/agents/sdk/relay.ts), and [streams/](/dimos/agents/sdk/streams/) runs in a browser too.

Source: [dimos/agents/sdk/](/dimos/agents/sdk/). TypeScript examples use imports relative to this source directory.
Save temporary example files there and run them from `dimos/agents` with `node --import=./test/web-sdk.mjs sdk/demo_example.ts`.
Remove the temporary file with `rm sdk/demo_example.ts` before running `npm run typecheck` or `npm test`.
Those checks treat saved examples as package code and may reject their imports.

## Built on Chord

[Chord](https://github.com/earendil-works/pi/tree/main/packages/chord) serves objects across processes. The SDK uses
four of its concepts:

- A host owns objects. The gateway is one host. A client is another that holds references.
- A reference is a client's handle on one object, taken with `host.get` and ended with `release()`.
- A cell is one observable state member of a reference, read with `value` and followed with `subscribe`.
- A method is a call on a reference. Ours resolve to an `Outcome`, done with a value or refused with a `Refusal`.

The objects you can hold, with their state and actions, are in [Agent gateway](/docs/usage/agent/gateway.md).

## Quickstart

Connect to a running gateway, hold the gateway object and print its link to dimOS. The snippets under Usage continue
from this `client` and `gateway`.

```ts
import { GatewayContract } from "../shared/protocol/gateway/contract.ts";
import { agentPaths } from "../gateway/config/paths.ts";
import { connect } from "./main.ts";
import { describeLink } from "../shared/protocol/gateway/link.ts";
import { unixSocket } from "./node.ts";

const client = await connect(unixSocket({ path: agentPaths(process.env).socket, label: "example" }));
const gateway = await client.host.get(GatewayContract, {});
console.log(describeLink(gateway.link.value));
```

## Usage

**Send a message and wait for the answer.** Hold the conversation as owner, call `send`, and wait for its outcome in the
`outcomes` cell with `until`. `send` is deduplicated by its request id, so `retryByRequestId` may send the same
arguments again after a dropped link and gets the first outcome. `dimos agent run` in
[cli/commands/run.ts](/dimos/agents/cli/commands/run.ts) does this and streams the answer.

```ts
import type { ConversationId } from "@earendil-works/pi-durable";
import { ConversationContract } from "../shared/protocol/conversation/contract.ts";
import { newUserRequest } from "../shared/protocol/actions/request.ts";
import { owner } from "./failures.ts";
import { retryByRequestId } from "./retry.ts";
import { until } from "./until.ts";

const id = Number(process.argv[2]) as ConversationId;
const conversation = await client.host.get(ConversationContract, { conversation: id, role: "owner" });
const sent = await retryByRequestId(
    client,
    { request: newUserRequest(), text: "Where is the robot?", images: [], whenBusy: "followUp" as const },
    (args) => owner(conversation).send(args),
);
if (!sent.ok) throw new Error(sent.refusal.code);
const answered = await until(conversation.outcomes, (rows) =>
    rows.find((row) => row.submission === sent.value.submission),
);
console.log(answered.status);
conversation.release();
```

**Show the connection and wait for dimOS.** `listen` subscribes one callback to several cells, and the terminal requests
a render in it. `until` with a signal waits for a condition with a timeout. The terminal's setup in
[frontends/tui/setup.ts](/dimos/agents/frontends/tui/setup.ts) waits for `accounts` this way.

```ts
import { listen } from "./state.ts";

const stop = listen([client.connection, gateway.link], () => {
    console.log(`${client.connection.value.state}: ${describeLink(gateway.link.value)}`);
});
await until(gateway.link, (link) => (link.state === "up" ? link : undefined), AbortSignal.timeout(60_000));
stop();
```

**Read live frames.** `RemoteFrames` owns a Web SDK `Session`. `configure(endpoint)` opens or replaces it when the
nullable `{url, robot}` endpoint changes. `want(channels)` selects exact relay manifest IDs. `channelFor` selects a
readable channel by its advertised ID. `toFrame` converts decoded SDK slots into images, voxel centers, grids,
and values for the existing terminal renderers. Rates use the SDK's received frequency and age.

```ts
import { InventoryContract } from "../shared/protocol/inventory/contract.ts";
import { RemoteFrames } from "./streams/remote.ts";

const inventory = await client.host.get(InventoryContract, {});
const frames = new RemoteFrames((error) => console.error(error));
const configure = () => {
	const view = inventory.inventory.value;
	frames.configure(view?.current ? view.inventory.environment.relay : null);
};
const unsubscribe = inventory.inventory.subscribe(configure);
configure();
frames.onRecord((record) => {
	if (record.kind === "image") console.log(`${record.channel} ${record.width}x${record.height}`);
});
try {
	frames.want(new Set(gateway.streams.value.filter((stream) => stream.encoding === "jpeg.v1").map((stream) => stream.channel)));
	await new Promise((resolve) => setTimeout(resolve, 10_000));
} finally {
	unsubscribe();
	frames.close();
	inventory.release();
}
```

`openRelay(endpoint)` in [relay.ts](/dimos/agents/sdk/relay.ts) injects the Node WebTransport provider and returns the SDK `Session`.
The harness and TUI each own a session and close it on shutdown. Relay configuration belongs to `RelayBridgeModule`.
The loader starts it when absent and reuses an existing configured bridge.

Automatically loaded bridges use `rtc=False`, `open_browser=False`, and `web_build=False`. An existing authored bridge using RTC with an SFU must set `rtc=False` to provide Node image previews. Existing bridge configuration stays under its owner's control. The SDK reports an explanatory error for an RTC source.

**Release and close.** Release each reference when its view closes, and close the client at the end.

```ts
gateway.release();
await client.close();
```

## API

| Export                                    | File                               | What it does                                                                                                                                                                                                                                                                             |
| ----------------------------------------- | ---------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `connect(open, options?)`                 | [main.ts](/dimos/agents/sdk/main.ts)                 | Dials with `open` until a socket opens, then redials with backoff after every drop. Returns a `Client`: `host`, `connection` as a `Live<Connection>`, `ended`, `whenLive()` and `close()`. It keeps dialing until a socket opens, so check `gatewayState` first when nothing may answer. |
| `unixSocket({ path, label, terminal? })`  | [node.ts](/dimos/agents/sdk/node.ts)                 | The `open` factory for the gateway's socket. One terminal id across reconnects, so a reconnecting client reaches the same objects. `label` is 1 to 32 of `[A-Za-z0-9_-]`. Node only.                                                                                                     |
| `owner(ref)`                              | [failures.ts](/dimos/agents/sdk/failures.ts)         | Narrows a reference to its `owner` variant or throws a `RefusalError` with `viewing`. Call it at each use, because the variant can change.                                                                                                                                               |
| `retryByRequestId(client, args, invoke)`  | [retry.ts](/dimos/agents/sdk/retry.ts)               | Calls `invoke(args)`, and once more with the same `args` after a reconnect when the link dropped mid-call. Only for actions whose catalog entry says `by_request_id`. Make the id first with `newUserRequest()` or `newActionRequest()`.                                                 |
| `refusalOfFailure(error, since)`          | [failures.ts](/dimos/agents/sdk/failures.ts)         | Any failure as a `Refusal`: `not_in_variant` or `forbidden` becomes `viewing`, Chord's `invalid` (arguments or input the gateway rejected) becomes `invalid`, a contract mismatch becomes `other_build`, anything else `unknown_outcome`. Print it with `refusalText`.                   |
| `until(live, pick, signal?)`              | [until.ts](/dimos/agents/sdk/until.ts)               | Resolves with the first non-undefined `pick` of a cell's values, now or later. Race it against `client.ended` or pass a signal.                                                                                                                                                          |
| `listen(members, changed)`                | [state.ts](/dimos/agents/sdk/state.ts)               | Subscribes `changed` to every cell given and returns one unsubscribe. The terminal requests a render this way.                                                                                                                                                                           |
| `attachRefusal(conversation, asked, got)` | [conversation.ts](/dimos/agents/sdk/conversation.ts) | The `viewing` refusal when you asked for `owner` and got `viewer`, or null. `dimos agent --view` asks for `viewer`.                                                                                                                                                                      |
| `RemoteFrames(report?)` | [streams/remote.ts](/dimos/agents/sdk/streams/remote.ts) | `configure(endpoint)` selects the relay. `want(channels)` selects manifest channel IDs, and `onRecord` delivers frames and SDK rates. |
| `openRelay(endpoint)`, `channelFor(session, channel)` | [relay.ts](/dimos/agents/sdk/relay.ts) | Opens a Node SDK session and selects readable manifest channels by ID. |
| `toFrame(channel, spec, slot, at)` | [streams/remote.ts](/dimos/agents/sdk/streams/remote.ts) | Converts an SDK slot for the existing frame renderers. |

A client's life, as `dimos agent run` in [cli/commands/run.ts](/dimos/agents/cli/commands/run.ts) lives it:

- `connect` resolves at the first open socket and redials after every drop. `client.connection` moves through
  `connecting`, `live`, `reconnecting` and `refused`.
- Hold `LifecycleContract` first and compare its `process.build.build` with your own `buildInfo().build`. A gateway of
  another build answers that object and refuses the rest.
- Hold `GatewayContract`, then each conversation you show, as `owner` to send or `viewer` to watch.
- Race every wait against `client.ended`. It rejects with `stopping` when the gateway shuts down and with `other_build`
  after a protocol mismatch, so every wait ends when the gateway is gone.
- Release each reference when its view closes, and `close()` the client at the end.
