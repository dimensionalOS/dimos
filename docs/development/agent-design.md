# On-robot agent design

The on-robot agent uses pi-durable for conversations, MCP for runtime discovery and live notices, and the DimOS Web SDK for relay streams. This document records the accepted relay migration design. See [agent usage](/docs/usage/agent.md) for setup and commands.

## How the pieces run

The gateway is one Node process that hosts conversations and opens the pi-durable SQLite store. Terminals attach through Chord over a Unix socket. The gateway can continue running while DimOS restarts or no terminal is attached.

DimOS runs the robot blueprint, McpServer, HarnessRuntime, RelayBridgeModule, and any deployed agent modules. The loader runs `dimos agent load` to add missing runtime modules. It starts RelayBridgeModule when absent and reuses an existing bridge with its configuration intact.

An automatically loaded bridge uses `rtc=False`, `open_browser=False`, and `web_build=False`. Node consumers need JPEG image channels. An authored bridge using RTC with an SFU must set `rtc=False` to supply agent image previews. The SDK adapter reports an explanatory error for an RTC source. There is no additional video adapter.

RelayBridgeModule resolves relay configuration, discovers native stream sources, owns their subscriptions, chooses codecs, and manages its relay child process. Its `relay_endpoint()` RPC returns null or `{url, robot}`. HarnessRuntime includes that value in `inventory.environment.relay`. Agents do not select a fixed relay address, and setup needs no separate Zenoh bridge.

## Connections and ownership

The gateway owns one `DimosConnection` to McpServer. `inventory` and `agent_module` remain MCP tools, alongside the robot's normal skills and module lifecycle operations. MCP also carries `dimos/harness_event` and `dimos/harness_status` notifications.

The harness and TUI each own a Web SDK `Session`. `openRelay(endpoint)` in `dimos/agents/sdk/relay.ts` injects the Node WebTransport provider and returns that session. Each owner replaces its session when the endpoint changes and closes it on shutdown. Harness code can import this Node factory and the stream conversion helper.

The gateway serves conversation state and inventory through Chord. The TUI holds inventory for its lifetime, uses its current relay endpoint, and shares the reference with the runtime tabs. Preview frames travel directly from the relay to the TUI.

## The agent's tools are few on purpose

| Tool | Implementation | Purpose |
|---|---|---|
| `read`, `bash`, `edit`, `write` | pi-durable coding tools | Files, commands, and Python scripts |
| `inventory` | HarnessRuntime through MCP | Runtime discovery |
| `agent_module` | HarnessRuntime through MCP | Create, restart, unload, delete, or list agent modules |
| `stream_get` | Local harness tool using the SDK | Read one relay value and optionally export its preview |
| `job` | Gateway | Manage cron jobs |

The harness registers the remote `inventory` and `agent_module` schemas from McpServer. `stream_get` has a local schema. The gateway does not expose an arbitrary RPC tunnel. Scripts and agent modules use the existing DimOS Python API and typed Specs for RPC calls.

## Stream reads and previews

`stream_get op=list` lists readable relay channel IDs and encodings. `op=read channel=ID` reads an exact advertised ID with `mode=latest|next`, `timeout_s`, and `export`. `timeout_s` defaults to 5 seconds.

The tool discovers channels through the SDK manifest and subscribes with `Session.subscribe(ch)`. The gateway catalog follows `Session.status.subscribe`, including runtime additions and removals on the same connection. Inventory supplies optional module annotations and the relay endpoint. Authored channels need no module annotation to appear or be read. `latest` returns the SDK's current value or waits if none exists. `next` waits for a value newer than the one present at the start of the read. Each call closes its SDK subscription in `finally`. A timeout or unavailable channel returns an error.

`export=true` saves an image, point-cloud, or grid preview at the relay's preview resolution. These exports do not preserve the original native message resolution. Use a module's native input or the Python API when original message data is required.

RelayBridgeModule reuses the existing JPEG, voxel, and costmap encoders. Compressed images use the existing decode-to-JPEG path. `toFrame(channel, spec, slot, at)` converts a decoded SDK slot to the existing `FrameRecord`, with SDK voxel and costmap inflation.

`RemoteFrames(report?)` in `dimos/agents/sdk/streams/remote.ts` owns the TUI session. `configure(endpoint|null)` selects the endpoint, `want(nativeChannels)` selects previews, and `onRecord` delivers frames. The existing image, cloud, and grid renderers remain responsible for terminal output.

Rate samples contain `at` and counts shaped as `{channel, hz, ageS}`. The SDK reports received rate and nullable age. `ratesOf` produces the current display table, and `rateLevel` grades freshness. There is no local source-rate or bandwidth estimator.

## Inventory

`inventory` supports whole-system discovery, `module=NAME`, `module=NAME rpc=NAME`, and `compact=true`.

The whole-system result includes:

- `environment`, including Python, checkout, transport, robot settings, the Python API guide, and nullable `relay: {url, robot}`.
- `runtime.modules`, with instance names, classes, paths, documentation, and RPC signatures.
- `runtime.streams`, with module, name, direction, type, and native channel.
- `agent_modules`, with source path, ownership, state, revision, and last error.

Compact inventory supplies keyed facts for the prompt. Stream reads do not create persistent subscription rows in inventory or the UI.

## Discovery goes from summary to detail

A conversation starts with a compact runtime snapshot. Later observations enter the transcript as runtime-change messages through InputGate. The agent can request module or RPC detail with `inventory`, or use `Dimos.connect()` from Python.

The context observer compares runtime facts with what the conversation has already seen. Tool results and notice inputs share the harness's output limits. Large text is cut and saved to a file that the model can read.

## Agent modules

An agent module is a Python file defining a subclass of `AgentModule`. HarnessRuntime owns its source path, revision, lifecycle, and conversation ownership. `In` and `Out` ports join compatible streams, and attributes typed with a module's Spec call its RPCs.

Use `setup_logger()` from `dimos.utils.logging_config` for diagnostics. `dimos log` reads the normal rotating run logs, including callback failures with module and stream fields. Use `notify(text, **data)` for meaningful alerts. Its rate limit and exported attachments remain part of the agent-module API.

## Events wake the agent

A module notice takes this route:

1. `notify` exports supported image and point-cloud attachments and publishes an `AgentNotice`.
2. HarnessRuntime associates the notice with the conversation that created the module and sends a live `dimos/harness_event` notification through McpServer.
3. `DimosConnection.onEvent` validates the notice. Its fields are `{id, module, conversation, at, text, data, exports, suppressed}`, with a UUID string `id`.
4. EventIntake passes the notice through EventDrafts, which renders attachments, and InputGate, which submits it to the owning conversation.

The request ID is `dimos-event:<uuid>`. InputGate uses that stable ID to identify repeated submissions. An idle conversation starts a turn; a busy conversation receives a normal follow-up input. This works without a terminal attached.

Notices are live. A notice missed while disconnected is not recovered. Once InputGate admits an input, pi-durable stores it with the conversation and retains the ordinary input queue. A notice with no owner or an unknown conversation is reported and dropped.

## Renderable output is shown automatically

FileRenderer renders files produced by coding tools, exported relay previews, and `notify` attachments. The model receives images with the input or tool result. The terminal uses its existing image, point-cloud, and clip views. Source paths and hashes remain attached to rendered files.

The stream tool exports SDK previews locally. AgentModule's notice exporter remains on the Python side. These are separate paths with different source resolutions.

## Motion safety

DimOS controls motion in several layers below `McpServer`:

| Layer | What it does | In `unitree-go2`? |
|---|---|---|
| `GO2Connection` | Every motion command reaches the robot through its `cmd_vel` input or its `move()` RPC. A 0.2 s command timeout stops the robot when commands stop. `stop_movement()` stops it at once. | Yes |
| `MovementManager` | Combines two velocity inputs. Teleop (`tele_cmd_vel`) beats navigation (`nav_cmd_vel`), cancels the navigation goal, and blocks navigation for 1 s after the last teleop command. | Yes |
| `ControlCoordinator` (`dimos/control/`) | A 100 Hz loop with per-joint priorities: the highest-priority task wins and the loser hears that it was preempted. The Go2 base is modelled as twist joints. | No. `unitree-go2-coordinator` and some controller blueprints use it. |
| `McpServer` capability registry | Keeps two skills marked `uses=[CAP_MOVEMENT]` from running at once. | Only for calls through MCP |

In the default `unitree-go2` blueprint, two gaps remain:

- Calling `GO2Connection.move()`, or publishing to `cmd_vel`, skips `MovementManager`, so teleop no longer wins. An agent module that declares `cmd_vel: Out[Twist]` does that, because its ports join running streams.
- Two autonomous behaviours, for example an agent module following a person and the patrol module, both enter at navigation priority, and `MovementManager` forwards both.

The harness adds no motion rules of its own. An agent module uses DimOS's motion stack like any other module. Closing these gaps and separating behaviours belongs in the control layer, in `ControlCoordinator` or a reworked `MovementManager`. A note in `MovementManager` already plans that rework.

## Large output is cut and saved, on every path

Tool results pass through the shared output limiter before entering the conversation. Long text is saved under `tool-results/`, and the result includes a bounded excerpt and its file path. `max_chars` requests a different limit within the remaining context budget. Renderable attachments use the shared file renderer.

## Trajectory

The pi-durable store records conversation entries. TrajectorySender converts committed entries to steps and sends batches through the existing MCP connection to `HarnessRuntime.publish_steps`. The runtime validates the batch and publishes `AgentStep` messages on its ordinary `agent_step` output. The system recorder captures that topic like any other stream, according to recording settings and topic selection.

The sender keeps its conversation position in the store and advances it after publication succeeds. Publication does not acknowledge durable recording. This position belongs to trajectory publication and does not provide notice recovery. Trajectories use the harness connection independently of the relay.

## Cron jobs

The gateway owns cron jobs as durable tasks. A job has a five-field cron expression, a timezone, and an action. A notify action submits an input through InputGate; a skill action calls McpServer. Job inputs use their own request IDs and the normal conversation queue.

## Files

| Component | Files |
|---|---|
| Harness composition | `dimos/agents/harness/main.ts` |
| MCP connection and typed client | `dimos/agents/harness/dimos/connection.ts`, `client.ts` |
| Local stream tool | `dimos/agents/harness/tools/lib/stream.ts` |
| Live notice intake | `dimos/agents/harness/events/intake.ts`, `message.ts` |
| Input persistence | `dimos/agents/harness/inputs/gate.ts`, `source.ts` |
| Trajectory publication | `dimos/agents/harness/trajectory/sender.ts`, `steps.ts` |
| Node relay factory and channel lookup | `dimos/agents/sdk/relay.ts` |
| Terminal frame adapter | `dimos/agents/sdk/streams/remote.ts`, `base.ts` |
| Runtime discovery and module lifecycle | `dimos/agents/harness/runtime/` |
| Relay discovery, codecs, and publication | `dimos/web/relay_bridge/` |
| Gateway contracts and presenters | `dimos/agents/shared/protocol/`, `shared/view-models/` |
| Media rendering | `dimos/agents/shared/render/`, `frontends/tui/` |

The protocol and view-model code stay separate from Node media processing. The Node relay provider workaround stays in the factory. The migration does not require changes to the browser SDK library.

## State on disk

| Path | Contents |
|---|---|
| `~/.config/dimos/agent/` | Provider credentials and gateway settings |
| `~/.local/state/dimos/agent/agent.sqlite` | Conversations, admitted inputs, tasks, jobs, and trajectory positions |
| `~/.local/state/dimos/agent/tool-results/` | Full tool text and local stream exports |
| `~/.cache/dimos/agent/render/` | Rendered file previews |
| `~/.local/state/dimos/harness/` | Agent module metadata and notice exports |
| `~/.local/state/dimos/modules/dimos_agent/` | Agent module source files and revisions |
| `~/.local/state/dimos/logs/<run-id>/main.jsonl` | Normal DimOS run logs |
| `~/.local/state/dimos/runs/` | DimOS run registry |

Only the gateway opens the conversation database. Clients read and act through gateway contracts.

## Validation

TypeScript tests cover live notice IDs, delivery ordering, attachment rendering, SDK channel mapping, stream reads, and terminal presenters. Python tests cover runtime and relay behavior. The real relay SDK test runs automatically when the checkout has a `.venv`; `DIMOS_RELAY_TEST_PYTHON` can select another Python executable. Its fixture is `dimos/agents/sdk/streams/demo_relay.py`.

`npm test` in `dimos/agents` runs the TypeScript tests. Passing local tests does not establish behavior on every robot or relay configuration.
