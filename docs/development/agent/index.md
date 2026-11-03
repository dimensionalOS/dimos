# Agent development

`dimos/agents/` holds two things. The Python agent tooling that dimOS blueprints use: the `@skill` decorator,
`SkillResult`, the capability registry, `McpServer` and `McpClient`, the skill containers in `skills/`, and
`HarnessRuntime` in `harness/runtime/`. And the on-robot agent, written in TypeScript and run as `dimos agent`: a
gateway process that hosts every conversation, a terminal that attaches to it, and the harness that runs the model with
its tools. This guide is about the TypeScript agent and names the Python parts at the end.
[The on-robot agent](/docs/usage/agent.md) says how to run it.

## How the on-robot agent runs

Two processes matter. The gateway is one Node process that stays up when no terminal is attached and when dimOS
restarts. dimOS is the robot's own process, started with `dimos run`. The gateway reaches it through `McpServer`, which
serves `HarnessRuntime`, a dimOS module that holds the Python half of the agent.

```text
dimos agent (Python)  execs  node dist/cli.js
                                  |
      terminal (frontends/tui)    |    CLI commands (cli/commands)
                   \______ sdk __/
                            |  Chord over a WebSocket on a Unix socket
                     gateway process
         server objects --> harness --> pi-durable --> agent.sqlite
                                |
                                |  MCP over HTTP, through pi-mcp
                          dimOS process
                  McpServer --> HarnessRuntime, agent modules
```

One message takes this path:

1. You type in the terminal. `frontends/tui` holds a `ConversationContract` reference through the SDK and calls its
   `send` method.
2. Chord carries the call over the socket to the gateway's conversation object in
   [gateway/server/objects/conversation.ts](/dimos/agents/gateway/server/objects/conversation.ts). `bind` runs the action's rule, then
   its executor.
3. The executor calls `InputGate.submit` in [harness/inputs/gate.ts](/dimos/agents/harness/inputs/gate.ts), the one door into a
   conversation. The gate derives a request id, prepends any change to the running system that the model has not seen,
   cuts the text to its limit, and calls pi-durable's `submit`.
4. pi-durable commits the input and runs the model through pi-ai. Each tool call is a durable task. `bash` runs in the
   conversation's execution environment. `inventory` goes through `DimosConnection` in
   [harness/dimos/connection.ts](/dimos/agents/harness/dimos/connection.ts) to `McpServer` and `HarnessRuntime`. The `afterTool` hook
   in [harness/tools/extension.ts](/dimos/agents/harness/tools/extension.ts) cuts large results and renders images.
5. Every commit updates the conversation's view. The `ConversationRoom` in
   [gateway/server/objects/room.ts](/dimos/agents/gateway/server/objects/room.ts) folds each frame into Chord cells, and Chord sends
   the deltas to every attached client.
6. The terminal turns the cells into rows with `shared/view-models` and draws them with pi-tui.

Nothing in that path needs a terminal. An event from an agent module, or a cron job, enters at step 3 and runs the same
way.

## The libraries and what each does for us

[pi-durable](https://github.com/earendil-works/pi/tree/main/packages/durable) is the agent engine. It owns one SQLite
store and runs conversations in it: transcripts of immutable entries, typed documents beside them, durable tasks with
checkpoints, and submissions you can wait for. It commits every change before anyone sees it, so a gateway killed in the
middle of a turn finishes the turn when it starts again. Everything in `harness/` is built on it.

[Chord](https://github.com/earendil-works/pi/tree/main/packages/chord) builds applications out of objects that have
observable state and callable methods and live at an address. A client holds a reference, never the object, and the
reference works the same in one process or across a WebSocket. pi-durable is itself built on Chord. We use it for the
gateway's client-facing objects in `gateway/server/objects/`, for the SDK in `sdk/`, and its `Context` for cancellation
everywhere. Contracts between the two sides are declared with Chord and TypeBox in `shared/protocol/`.

[pi-ai](https://github.com/earendil-works/pi/tree/main/packages/ai) talks to model providers. `gateway/auth/` opens its
`Models` and credential store, and the harness hands them to pi-durable. Its message and content types are the ones the
transcript stores.

[pi-mcp](https://github.com/earendil-works/pi/tree/main/packages/mcp) is the MCP client. One file imports it,
[harness/dimos/connection.ts](/dimos/agents/harness/dimos/connection.ts), and the test suite refuses a second importer.

[pi-tui](https://github.com/earendil-works/pi/tree/main/packages/tui) supplies the terminal components and keyboard
handling in `frontends/tui/`. [pi-coding-agent](https://github.com/earendil-works/pi/tree/main/packages/coding-agent) is
used for its message and tool components and themes, for the login flows, and for one summarizer. The agent loop itself
does not come from it.

[TypeBox](https://github.com/sinclairzx81/typebox) declares the schemas of contracts, action arguments and the records
that cross the process boundary from `HarnessRuntime`. [sharp](https://sharp.pixelplumbing.com) decodes and draws images
in `shared/render/`. `ws` is the WebSocket server in the gateway and the client socket in [sdk/node.ts](/dimos/agents/sdk/node.ts).
`agent.sqlite` is opened through `node:sqlite` with `synchronous = FULL`, because pi-durable's own setting can lose the
newest commit on power loss.

## Components and the direction they depend in

Each component has one entry file, and each may import only the components above it in this table.
[test/boundaries.test.ts](/dimos/agents/test/boundaries.test.ts) fails on any other import.

| Folder                                              | Role                                                                  | Entry                                | May import                                      |
| --------------------------------------------------- | --------------------------------------------------------------------- | ------------------------------------ | ----------------------------------------------- |
| [shared/protocol/](/docs/development/agent/protocol.md)       | The contracts, actions and types both sides agree on. Browser-safe.   | one `contract.ts` per object         | itself                                          |
| [sdk/](/docs/usage/agent/sdk.md)                               | Connects a client to the gateway and holds its objects. Browser-safe. | [main.ts](/dimos/agents/sdk/main.ts)               | protocol                                        |
| [shared/view-models/](/docs/development/agent/view-models.md) | Turns state into rows, labels and status text. Browser-safe.          | one function per file                | protocol, SDK types                             |
| [shared/render/](/docs/development/agent/render.md)           | Turns image files, point clouds and clips into images. Node only.     | [render.ts](/dimos/agents/shared/render/render.ts) | protocol                                        |
| [harness/](/docs/usage/agent/harness.md)                       | The agent: tools, inputs, context, events, jobs, the link to dimOS.   | [main.ts](/dimos/agents/harness/main.ts)           | protocol, render                                |
| [gateway/](/docs/usage/agent/gateway.md)                       | Hosts the harness and serves it to clients.                           | [main.ts](/dimos/agents/gateway/main.ts)           | harness, protocol, `shared/socket.ts`           |
| [frontends/tui/](/docs/development/agent/tui.md)           | The terminal.                                                         | [main.ts](/dimos/agents/frontends/tui/main.ts)     | sdk, protocol, view-models, render              |
| [cli/](/docs/usage/agent/cli.md)                               | The `dimos agent` commands and the gateway process.                   | [main.ts](/dimos/agents/cli/main.ts)               | gateway config and main, sdk, frontends, shared |
| test/                                               | The runner, the boundary test and the shared helpers.                 | [run.mjs](/dimos/agents/test/run.mjs)              |                                                 |

Three rules shape the table. The protocol depends on nothing, so a browser and the gateway can share it. The harness
does not know the gateway exists, so it can be tested without a socket. A renderer reaches the gateway only through the
SDK and reads only view models, so a web frontend would be a sibling of `frontends/tui/` with the same imports.

## Conventions the tests enforce

- Local imports use Node package subpaths, such as `#dimos/agents/harness/main.ts`, mapped by `imports` in `package.json`.
- `main.ts` is the entry of each application. Supporting APIs are imported from the file that defines them. There are no
  `index.ts` files and no re-exports.
- A folder with a `base.ts` is a family. `base.ts` defines the interface, each sibling implements it, and one consumer
  imports every member. The families are listed in `FAMILIES` in `test/boundaries.test.ts`.
- `lib/` is private to its parent folder.
- Tests sit beside the code they exercise, as `*.test.ts`.
- One file calls pi-durable's `submit`: [harness/inputs/gate.ts](/dimos/agents/harness/inputs/gate.ts). One file imports pi-mcp.
  `frontends/` and `shared/render/` import pi-durable's types only.
- `shared/protocol/`, `shared/view-models/` and `sdk/` except `node.ts` and `streams/` import no Node module, so they
  run in a browser.
- A served contract never names a state member and a method alike, because Chord rejects such a contract.

## Build, test and run

The package needs Node 24 or 26. Run these from `dimos/agents`:

```sh skip
npm ci                # install at the locked versions
npm run build         # esbuild bundles cli/main.ts to dist/cli.js; dependencies stay external
npm run typecheck     # tsc --noEmit
npm test              # every *.test.ts with Node's test runner
npm run check         # typecheck, test, build
npm run format        # prettier at 120 columns
```

CI runs install, typecheck, test and build. `dimos agent`, implemented in
[dimos/cli/commands/agent.py](/dimos/cli/commands/agent.py), checks the Node version, runs `npm ci` and `npm run build` when
their inputs changed (see [the CLI guide](/docs/usage/agent/cli.md)), then replaces itself with `node dist/cli.js`. It passes
`DIMOS_ROOT`, `DIMOS_PYTHON`, `DIMOS_CLI`, `DIMOS_MCP_URL`, `DIMOS_RUNS_DIR` and `DIMOS_STATE_DIR`, which tell the
gateway where dimOS is.

The loader starts or reuses `RelayBridgeModule`, which owns stream discovery, codecs, and relay configuration.
The harness and terminal each open a Web SDK session using the nullable `inventory.environment.relay` endpoint.
The terminal receives previews directly from the relay. Module diagnostics use the normal rotating run logs,
read with `dimos log`. Meaningful module alerts use `notify()` and arrive live over MCP.
The build resolves web SDK sources through [web-aliases.mjs](/dimos/agents/web-aliases.mjs), and the test runner loads those sources
through [test/web-sdk.mjs](/dimos/agents/test/web-sdk.mjs). See [agent setup](/docs/usage/agent.md#set-up-and-start-the-agent).

## Where to start reading

To run the agent, read [The on-robot agent](/docs/usage/agent.md). To understand the code, read the guides in this
order. Each one explains the library it rests on, lists its API, and ends with examples from the most basic use to a
composed one.

1. [sdk/](/docs/usage/agent/sdk.md): how a client connects and what it can hold and call. The smallest complete program is here.
2. [shared/protocol/](/docs/development/agent/protocol.md): the contracts and actions that client holds, and how an action is
   defined, checked and answered.
3. [gateway/server/objects/](/docs/development/agent/gateway-objects.md): how the gateway serves those contracts as Chord objects.
4. [harness/](/docs/usage/agent/harness.md): the agent on pi-durable and its subsystems.
5. [gateway/](/docs/usage/agent/gateway.md) and [cli/](/docs/usage/agent/cli.md): the process around the harness and the commands that reach
   it.
6. [frontends/tui/](/docs/development/agent/tui.md), [shared/view-models/](/docs/development/agent/view-models.md) and
   [shared/render/](/docs/development/agent/render.md): how state becomes a screen.

## Python in `dimos/agents/`

These parts predate the TypeScript agent and serve dimOS blueprints and Claude Code.
[Agents](/docs/capabilities/agents/index.md) documents them.

- [annotation.py](/dimos/agents/annotation.py) defines `@skill`, which marks a module method as an RPC and as a tool for a model.
  [skill_result.py](/dimos/agents/skill_result.py) is its return type. [capabilities.py](/dimos/agents/capabilities.py) keeps two skills that need
  the same capability, such as movement, from running at once.
- [mcp/](/dimos/agents/mcp/README.md) holds `McpServer`, which serves every `@skill` of a running system as an MCP tool on port 9990,
  `McpClient`, the LangGraph agent that calls them, and `McpAdapter`.
- [system_prompt.py](/dimos/agents/system_prompt.py) is the Go2 system prompt for that agent. [skills/](/dimos/agents/skills/) holds the skill
  containers: navigation, observation, speech, GPS, person following and more.
- [Python agent runtime](/docs/usage/agent/runtime.md) is `HarnessRuntime`, the dimOS module the gateway talks to: inventory, stream
  reads, agent modules, events, exports, and SDK preview publication over Zenoh.
- [agent_spec.py](/dimos/agents/agent_spec.py), [vlm_agent.py](/dimos/agents/vlm_agent.py), [vlm_agent_spec.py](/dimos/agents/vlm_agent_spec.py) and
  [ollama_agent.py](/dimos/agents/ollama_agent.py) are the Spec and modules of the vision agent. [typesafe/](/dimos/agents/typesafe/) is an agent
  whose input streams become a JSON state that one model request answers typed questions about.
- [voice_input.py](/dimos/agents/voice_input.py) and [web_human_input.py](/dimos/agents/web_human_input.py) turn speech and web input into agent
  text. [llm_trace.py](/dimos/agents/llm_trace.py) captures every provider request and response byte for byte.
- [testing/](/dimos/agents/testing/), [demos/](/dimos/agents/demos/), [fixtures/](/dimos/agents/fixtures/) and [demo_agent.py](/dimos/agents/demo_agent.py) support the Python
  tests and demos.
