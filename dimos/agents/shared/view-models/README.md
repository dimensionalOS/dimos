# View models

A view model is a pure function from the values a frontend holds, the cells of its gateway and conversation references
and the time, to what it shows: chat rows, footer fields, switcher lines, status lines, job lanes and stream cards. The
terminal calls these functions each time it draws, and `dimos agent conversations` and `dimos agent status` print the
same rows, so a person reads the same words in every frontend and a sentence or a threshold changes in one place. A view
model takes its values and the time as arguments and returns plain rows, so the terminal recomputes on every draw and a
CLI command computes once. Each function is imported from the file that defines it. The folder is browser code and
imports values from `shared/protocol/` and itself.

## Built on pi-durable documents

The conversation reference serves pi-durable's per-conversation
[documents](https://github.com/earendil-works/pi/tree/main/packages/durable#watching-a-conversation) as cells of the
same name, and [Busy Conversations](https://github.com/earendil-works/pi/tree/main/packages/durable#busy-conversations)
there explains the run and the queue. The view models read four of them beside the transcript:

- `live` is `pi.live`. `run` exists exactly while the conversation is busy, `generation.message` is the partial answer
  being streamed, and `tools` is the current tool round.
- `queue` is `pi.inbox`, the submissions waiting for a boundary: a `steer` joins the run after the current tool round, a
  `followUp` starts the next run.
- `usage` is `pi.usage`, token and cost totals per model, and `agent` is `pi.agent`, the conversation's model and
  thinking level.
- `transcript` holds `TranscriptItem` values, the gateway's projection of the entries with image bytes stripped.

## Quickstart

Build the chat rows and the footer from a conversation reference, as the terminal's `rows()` and `footer()` in
[frontends/tui/main.ts](../../frontends/tui/main.ts) do on every draw. The terminal passes its pending sends where this
passes none, and reads `Date.now()` once per draw. The snippets under Usage continue from these references.

```ts
import type { ConversationContract } from "../protocol/conversation/contract.ts";
import type { EpochMs } from "../protocol/time.ts";
import type { GatewayContract } from "../protocol/gateway/contract.ts";
import type { Ref } from "@earendil-works/chord";
import { footerOf } from "./conversation/footer.ts";
import { timelineOf } from "./conversation/timeline.ts";

export function draw(gateway: Ref<typeof GatewayContract>, state: Ref<typeof ConversationContract>, now: EpochMs) {
    const rows = timelineOf(
        {
            transcript: state.transcript.value,
            live: state.live.value,
            queue: state.queue.value,
            outcomes: state.outcomes.value,
            detachable: state.detachable.value,
        },
        [],
    );
    for (const row of rows) console.log(row.kind, row.key);
    const footer = footerOf({
        transcript: state.transcript.value,
        live: state.live.value,
        queue: state.queue.value,
        usage: state.usage.value,
        agent: state.agent.value,
        notices: state.notices.value,
        settings: gateway.settings.value,
        accounts: gateway.accounts.value,
    });
    console.log(footer.running ? "running" : "idle", `$${footer.costUsd.toFixed(2)}`, footer.context?.level, now);
}
```

## Usage

**List the conversations.** `switcherRows` orders roots newest first with their forks under them and gives each a state,
its unread and waiting counts and a `Level`. `dimos agent conversations` prints `switcherLine` of each row, and the
terminal's Ctrl+O switcher draws the same rows.

```ts
import { switcherLine, switcherRows } from "./gateway/switcher.ts";

export function listConversations(gateway: Ref<typeof GatewayContract>, now: EpochMs): void {
    for (const row of switcherRows(gateway.conversations.value, null, now))
        console.log(switcherLine(row));
}
```

**Say why an action was refused.** Every `Refusal` a person reads goes through `refusalText`, and every span through
`duration`. This prints `No job patrol.` and `41s 3m12s 6h12m 2d4h`.

```ts
import { duration } from "./time.ts";
import { refusalText } from "./outcome.ts";

console.log(refusalText({ code: "not_found", what: "job", name: "patrol" }));
console.log([41_000, 192_000, 22_320_000, 187_200_000].map(duration).join(" "));
```

**Rate a stream.** `ratesOf` converts each `RateSample` into the current `RateTable`. Counts contain `channel`, `hz`,
and nullable `ageS`, as reported by the SDK. `rateLevel` grades freshness as `ok`, `warn` after 3 seconds, or `critical`
after 30 seconds. A channel with no received value has no freshness level.

```ts
import type { Channel } from "../protocol/streams.ts";
import { epochMs } from "../protocol/time.ts";
import { ratesOf, rateLevel } from "./streams/rates.ts";

const camera = "dimos/color_image/sensor_msgs.Image" as Channel;
const rates = ratesOf({ at: epochMs(2_000), counts: [{ channel: camera, hz: 30, ageS: 0.1 }] });
console.log(rates.get(camera)?.hz, rateLevel(rates.get(camera), epochMs(32_000)));
```

## API

| Export                                                                   | File                                                                                                 | What it does                                                                                                                                                                                   |
| ------------------------------------------------------------------------ | ---------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `timelineOf(input, sending)`, `isRunning(live)`                          | [conversation/timeline.ts](conversation/timeline.ts)                                                 | The chat as keyed rows: each item with its outcome, the streamed partial, the running calls, the queue, live notices and the sends still in flight. `isRunning` is the one definition of busy. |
| `runningCalls`, `backgroundTarget`, `movedToBackground`, `movedRows`     | [conversation/tools.ts](conversation/tools.ts)                                                       | The running calls with their ages, the one Ctrl+B sends to the background, whether a tool result says its call moved there, and the calls running in the background as rows.                                                                |
| `queuedRows`, `userQueued`, `editorTextOf`                               | [conversation/queue.ts](conversation/queue.ts)                                                       | The queue as rows with who queued each, the user's texts among them, and those texts joined back into the editor after a withdraw.                                                             |
| `selectables`, `editText`                                                | [conversation/history.ts](conversation/history.ts)                                                   | The user and assistant items the arrow keys select for an edit or a summary, and the text an edit starts from.                                                                                 |
| `noticesOf`, `footerOf`                                                  | [conversation/notices.ts](conversation/notices.ts), [conversation/footer.ts](conversation/footer.ts) | Delivered notices and notices in the ordinary input queue, and the footer: model, thinking, running, queued counts, cost and the context left with its level, warn under 25 % and critical under 10 %.               |
| `switcherRows`, `switcherLine`                                           | [gateway/switcher.ts](gateway/switcher.ts)                                                           | The conversations as the switcher and the CLI list them.                                                                                                                                       |
| `linkSegments`, `runSegment`, `connectionSegment`, `statusText`          | [gateway/link.ts](gateway/link.ts), [connection.ts](connection.ts), [status.ts](status.ts)           | The DimOS link, the run file and this client's own link as status segments, and `dimos agent status` as lines. `statusText` is the one view model that takes references.                       |
| `jobLanes(jobs, from, to)`                                               | [jobs/timeline.ts](jobs/timeline.ts)                                                                 | One lane per job with its past runs and next due times inside the window.                                                                                                                      |
| `sidebarGroups`, `allStreamGroups`, `systemGroups`, `pictureOf`          | [streams/cards.ts](streams/cards.ts), [streams/kind.ts](streams/kind.ts)                             | Stream cards grouped by the modules a conversation owns, by every module, or by the system's own streams, and whether a type is an image, a cloud or a grid.        |
| `ratesOf`, `rateLevel`                                                 | [streams/rates.ts](streams/rates.ts)                                                                 | The SDK's received rate and age, and the resulting freshness level.                                                                                                |
| `actionLine`, `pendingSends`                                             | [actions.ts](actions.ts)                                                                             | One line per action the client made, pending, done or refused, and the sends still unanswered for `timelineOf`.                                                                                |
| `refusalText`, `Level`, `worst`, `Segment`, `clean`, `clock`, `duration` | [outcome.ts](outcome.ts), [levels.ts](levels.ts), [text.ts](text.ts), [time.ts](time.ts)             | A refusal as its sentence, how much a datum needs a person and the worst of several, text with control characters removed, and times as `14:06` and `3m12s`.                                   |

A `Level` is `ok`, `warn` or `critical`, and a renderer maps it to a colour.
[test/boundaries.test.ts](../../test/boundaries.test.ts) keeps this folder browser-safe: value imports from
`shared/protocol/` and itself, type imports from `sdk/`, and platform modules left out.
