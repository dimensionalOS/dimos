# Agent harness

The harness is the agent. It runs each conversation's model turns and tool calls, starts runs from DimOS events and cron
jobs, and keeps every step in one SQLite store, so it works with no terminal attached and finishes an interrupted turn
after a restart. `openAgent` returns an `Agent` whose members start conversations, submit input, fork history, operate
modules and jobs, and report the DimOS link. The gateway opens it once per process and serves those members as the
objects in [gateway/README.md](../gateway/README.md). The entry is `openAgent` in [main.ts](main.ts).
[runtime/](runtime/harness_runtime.py) is the Python half, HarnessRuntime, which `dimos agent load` puts into the running DimOS
system and which the harness reaches over MCP.

## Built on pi-durable

[pi-durable](https://github.com/earendil-works/pi/tree/main/packages/durable) runs the agent loop and stores every step
as a commit. One `Harness` owns the store, and the `Agent` exposes it as `harness`. A caller meets these concepts:

- A conversation is one transcript, made with `conversations.start` and configured through `harness.conversation(id)`.
- An entry is one immutable transcript record, read inside `harness.commit((tx) => tx.entry(...))`. The harness adds
  `dimos.input` for every input and the summary and origin entries that `history` writes.
- A submission is one input handed to a conversation. `inputs.submit` returns its id, and `harness.submission(id)` waits
  for it to settle as `done` with an answer entry or `unanswered` with a reason.
- A task is a durable step: a model request, a tool call, a sleeping job or a backgrounded call. `harness.abortTask`
  ends one.
- A document is typed JSON stored beside the transcript, such as `dimos.jobs` and each conversation's
  `dimos.runtime_baseline`.
- An extension bundles tools, prompt sections, hooks and task kinds. `openAgent` installs the harness's extensions, then
  calls `resume()` to start the scheduler on whatever work the store holds.

## Quickstart

Start a DimOS system first, such as `dimos --replay run unitree-go2 --daemon`, and set `OPENAI_API_KEY` in the
environment or in the checkout's `.env`. The Quickstart opens the agent on this checkout with `gpt-5.6-luna`. The agent
finds the running system, runs `dimos agent load`, and connects to McpServer. Then the Quickstart asks what is running
and prints the answer. The gateway's [main.ts](../gateway/main.ts) opens the agent the same way, with the model and
credentials from its config. The snippets under Usage continue from this `agent` and `conversation`.

```ts
import type { UserRequestId } from "../shared/protocol/conversation/input.ts";
import { AssistantEntry } from "@earendil-works/pi-durable";
import { BACKGROUND_CONTEXT } from "@earendil-works/chord/context";
import { ModelRuntime } from "@earendil-works/pi-coding-agent";
import { existsSync, mkdtempSync } from "node:fs";
import { fileURLToPath } from "node:url";
import { homedir, tmpdir } from "node:os";
import { join } from "node:path";
import { openAgent } from "./main.ts";
import { until } from "../sdk/until.ts";

const context = BACKGROUND_CONTEXT;
const root = fileURLToPath(new URL("../../../", import.meta.url));
if (existsSync(join(root, ".env"))) process.loadEnvFile(join(root, ".env"));
const state = join(process.env["XDG_STATE_HOME"] ?? join(homedir(), ".local", "state"), "dimos");
const dir = mkdtempSync(join(tmpdir(), "dimos-agent-"));
const models = await ModelRuntime.create({ authPath: join(dir, "auth.json"), modelsPath: null });

const agent = await openAgent(
    {
        install: {
            root,
            python: join(root, ".venv", "bin", "python"),
            cli: join(root, ".venv", "bin", "dimos"),
            mcpUrl: new URL("http://127.0.0.1:9990/mcp"),
            runsDir: join(state, "runs"),
            stateDir: state,
        },
        models,
        storePath: join(dir, "agent.sqlite"),
        toolResultsDir: join(dir, "tool-results"),
        renderCacheDir: join(dir, "render"),
        skillRoots: [join(root, "dimos", "agents", "harness", "skills")],
        defaults: () => ({
            model: { provider: "openai", modelId: "gpt-5.6-luna" },
            thinkingLevel: null,
            workspace: root,
        }),
        gatewayEnv: process.env,
        report: (error) => console.error(error),
    },
    context,
);

const load = await until(agent.loader.status, (status) =>
    status.state === "idle" || status.state === "running" ? undefined : status,
);
if (load.state !== "loaded") throw new Error(`the runtime did not load: ${JSON.stringify(load)}`);
await until(agent.connection.status, (link) => (link.state === "up" ? link : undefined));

const conversation = await agent.conversations.start(null, "readme-start", context);
const sent = await agent.inputs.submit(
    conversation,
    {
        source: { kind: "user", request: "r1" as UserRequestId },
        text: "what is running?",
        images: [],
        render: null,
        whenBusy: "reject",
    },
    context,
);
if (sent.kind !== "submitted") throw new Error(sent.kind);
const handle = await agent.harness.submission(sent.submission, context);
if (handle === undefined) throw new Error("submission vanished");
const settled = await handle.wait(context);
if (settled.answer === undefined) throw new Error(`unanswered: ${settled.reason}`);
const answer = await agent.harness.commit((tx) => tx.entry(AssistantEntry, settled.answer), context);
const message = answer?.model?.[0];
if (message?.role === "assistant") {
    for (const part of message.content) if (part.type === "text") console.log(part.text);
}
```

## Usage

**Edit a message.** `history` forks the conversation before the message and submits the new text there, and the source
stays as it was. `summarizeUpTo` and `summarizeFrom` fork the same way around a model-written summary. A repeated
request returns the fork it already made. The gateway's conversation object calls all three.

```ts
const branch = await agent.history.edit(conversation, settled.entry, "and now?", [], "r2" as UserRequestId, context);
if (branch.kind !== "fork" && branch.kind !== "new") throw new Error(branch.kind);
console.log(`forked into conversation ${branch.conversation}`);
```

**Show the DimOS link.** `connection`, `loader`, and `runs` each carry a `status` with `value` and
`subscribe`, the `Live` in [shared/protocol/status.ts](../shared/protocol/status.ts). The gateway feeds each one into a
cell of its gateway object.

```ts
import { describeLink } from "../shared/protocol/gateway/link.ts";

const stop = agent.connection.status.subscribe((link) => console.log(describeLink(link)));
console.log(agent.loader.status.value.state, agent.runs.status.value.length);
stop();
```

**Create a job.** `jobs` writes the definition and its sleeping task in one commit. When the job is due, a `notify`
action submits its text through `inputs` as a turn in the job's conversation, and a `skill` action calls a skill through
the link directly. `runtime` is the other side of the link: module operations and the inventory.

```ts
import type { JobName, TimeZone } from "../shared/protocol/jobs/state.ts";

const job = await agent.jobs.create(
    {
        name: "morning-check" as JobName,
        conversation,
        cron: "0 9 * * *",
        timeZone: "UTC" as TimeZone,
        action: { kind: "notify", text: "look around and report what changed overnight" },
    },
    context,
);
if (job.kind !== "done") throw new Error(job.kind);
console.log(new Date(job.firstDue).toISOString());
```

**Close.** Stops the services, then closes the store and the MCP link.

```ts
await agent.close(context);
```

## API

| Member                                   | File                                                               | What it does                                                                                                                                                                                                                                                                                                                                                                |
| ---------------------------------------- | ------------------------------------------------------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `openAgent(options, context)`            | [main.ts](main.ts)                                                 | Opens the store, installs the extensions, resumes pending work, starts the background services and returns an `Agent`. `options` is the install, pi-ai's `Models`, the store, tool-result and render paths, the skill roots, the per-conversation `defaults`, the gateway's environment, a `report` sink for errors raised in background work, and an optional `now` clock. |
| `agent.harness`                          | pi-durable                                                         | `conversation(id)` for a handle to configure, `submission(id)` to wait on an input, `commit((tx) => ...)` to read entries and documents, `abortTask(id)` to end a run.                                                                                                                                                                                                      |
| `agent.inputs`                           | [inputs/gate.ts](inputs/gate.ts)                                   | The one door into a conversation. `submit(conversation, draft, context)` for user, event, job and runtime inputs, with `whenBusy` deciding what happens during a run. `withdrawUser` takes queued text back.                                                                                                                                                                |
| `agent.conversations`                    | [conversations.ts](conversations.ts)                               | `start(cwd, request, context)` creates a conversation with the defaults and a runtime baseline, once per request. `markSeen` and `seen` keep the newest entry shown per conversation.                                                                                                                                                                                       |
| `agent.history`                          | [history/forks.ts](history/forks.ts)                               | `edit`, `summarizeUpTo` and `summarizeFrom`, each a fork recorded under its request and returning a `Branch`.                                                                                                                                                                                                                                                               |
| `agent.runtime`                          | [dimos/client.ts](dimos/client.ts)                                 | Typed MCP calls for `snapshot`, `inventory`, `createModule`, and `moduleOp`. `relay(inventory?)` opens or reuses the process's Web SDK session from the inventory endpoint.                                                                                                                                                                                                 |
| `agent.jobs`                             | [jobs/ops.ts](jobs/ops.ts)                                         | `create`, `update`, `remove`, `run` and `nextDue`, shared with the model's `job` tool.                                                                                                                                                                                                                                                                                      |
| `agent.detacher`                         | [background/detacher.ts](background/detacher.ts)                   | `of(conversation)` lists the calls that can move now, and `detach` moves one.                                                                                                                                                                                                                                                                                               |
| `agent.connection`                       | [dimos/connection.ts](dimos/connection.ts)                         | The MCP link to McpServer. `status` is the link, `onEvent` and `onHarnessStatus` are its notifications, `retryNow` ends a backoff early.                                                                                                                                                                                                                                    |
| `agent.loader`, `agent.runs`             | [dimos/loader.ts](dimos/loader.ts), [dimos/runs.ts](dimos/runs.ts) | Status for the automatic runtime load and the DimOS runs directory. The loader starts missing runtime modules and reuses the relay bridge.                                                                                                                                                                                                                                  |
| `agent.stateDir`, `agent.close(context)` |                                                                    | Where saved module records are read while DimOS is down, and the stop that closes the services, the `Harness` and both links.                                                                                                                                                                                                                                               |

The gateway's folds also import the entry kinds in [inputs/source.ts](inputs/source.ts) and
[history/entries.ts](history/entries.ts), `SEEN_DOC_KIND` from [conversations.ts](conversations.ts), `listJobs` from
[jobs/registry.ts](jobs/registry.ts) and `DimosInstall` from [dimos/install.ts](dimos/install.ts).
The rest is the agent's own machinery: the model's tools in [tools/](tools/extension.ts), the runtime facts the model
is told in [context/](context/extension.ts), events in [events/](events/intake.ts), the cut on large text in
[limit/](limit/limit.ts), skills in [skills/](skills/catalog.ts) and the trajectory a recording keeps in
[trajectory/](trajectory/sender.ts).
