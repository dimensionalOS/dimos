---
name: dimos-runtime
description: Work with the running DimOS system. Discover modules, RPCs and streams, read a stream, write agent modules that watch streams or drive the robot, and schedule cron jobs.
---

HarnessRuntime is a module inside the running DimOS system. It serves `inventory` and `agent_module` through McpServer. The harness implements `stream_get` locally through the DimOS Web SDK relay session. The gateway that hosts this conversation runs your `job` tool and delivers events, so there is nothing to poll. `docs/usage/agent/harness.md` in the DimOS checkout describes the parts.

The `dimos_runtime` section of the system prompt is a snapshot of the system when this conversation started. Later changes arrive inside messages, as a `<dimos_runtime_change>` block that lists facts added, changed and removed, or as a `<dimos_runtime>` block that states the whole runtime again. The gateway writes them. They are not user input.

Discovery goes from summary to detail. Read only what the task needs:

1. The snapshot lists the modules, their RPC signatures with the first line of each doc, and the streams.
2. `inventory module=NAME` gives the module's documentation, its RPC signatures with the first paragraph of each doc, and its streams with the import of each message type.
3. `inventory module=NAME rpc=NAME` gives one RPC's full documentation.

`inventory` with no arguments describes the whole system, its agent modules, and `environment.relay`, which is null or `{url, robot}`. Discovery is live on every call.

An RPC is a module's Python API, and a `@skill` is an RPC too. A `Dimos.connect()` client calls it as `app.get_module(MODULE).RPC(...)`, and a module calls it through an attribute typed with that module's Spec. A stream is read once with `stream_get`, or continuously by a module with an `In` port of the same name and type.

`stream_get op=list` lists the relay manifest's readable channels and encodings. Read one with `stream_get op=read channel=ID`, using the exact returned ID. Authored channels can have arbitrary names. Newly discovered outputs use `module.port`. Native topics from `inventory` are for Python ports, not relay subscriptions. `mode=latest` returns the SDK's current value or waits if none is available. `mode=next` waits for a value newer than the one present when the read starts. `timeout_s` defaults to 5. A timeout or unavailable channel is an error, not a new observation.

Each call closes its SDK subscription when the read ends. `export=true` saves and renders an image, point-cloud, or grid preview at the relay's preview resolution. Use a module's native `In` port or the Python API when you need the original message data.

The loader starts `RelayBridgeModule` when needed and reuses an existing bridge. The bridge resolves its own configuration and advertises the endpoint through inventory. No separate Zenoh bridge setup is needed.

Automatically loaded bridges use `rtc=False`, `open_browser=False`, and `web_build=False`. An existing authored bridge using RTC with an SFU must set `rtc=False` to provide Node image previews. Existing bridge configuration stays under its owner's control. The SDK reports an explanatory error for an RTC source.

Work that continues without you, such as a watcher, a follower or a control loop, is an agent module. It is one Python file that defines one subclass of `dimos.agents.harness.runtime.agent_module.AgentModule`, which is a `dimos.core.module.Module`. It runs on its own worker under its class name, which no running module may already use, and it stops when the system stops.

- An `In[T]` or `Out[T]` attribute with the name and type of a running stream joins that stream. An attribute typed with a module's Spec calls that module's RPCs.
- `self.subscribe(stream, callback)` calls `callback` with each message. Callback exceptions go to the normal DimOS run log with module and stream fields.
- For diagnostics, import `setup_logger` from `dimos.utils.logging_config` and use `setup_logger()` to log messages with useful context. Read the rotating run logs with `dimos log`, normally stored in `~/.local/state/dimos/logs/<run-id>/main.jsonl`.
- `self.notify(text, **data)` wakes this conversation at most once per `notify_interval_s` (a class attribute, 10 s by default) and returns `False` when the rate limit drops the notice. Every `Image` and `PointCloud2` in `data` is saved to a file and shown with the event.

Keep the file's top level to imports and definitions, and keep setup short. The top level, `__init__` and the part of an `async def main()` before its `yield` run while DimOS loads the module, and no other module can be loaded, restarted or unloaded until they finish. Run loops and slow work in a thread or task that `main()` starts before its `yield` and stops after it.

Each module has one source file, `dimos_agent/NAME.py` in DimOS's modules directory, and every `agent_module` result names it. Write the module's code with `write`, then run `agent_module op=create name=NAME path=FILE`. To change the module, edit its source file with `edit` and run `op=restart`. Results never contain the source, so read the file when you need it.

`agent_module` takes one of these values of `op`:

- `create` copies `path` to the module's source file and starts the module. Without `path`, it uses the source file as it is.
- `restart` runs the source file's current code, whether or not the module is running. If the file does not compile or import, the old code keeps running.
- `unload` stops the module, and `delete` unloads it and removes it with its source file.
- `list` lists the modules with their source files, state and last error.

Each result says what runs afterwards and from which file, or gives the error and its traceback. If new code fails to start, nothing of the module runs. Use `dimos log` to investigate runtime failures. Use `notify()` for meaningful alerts that need the conversation to act.

A module's own `@skill` methods are served by McpServer like any other skill. McpServer refuses a skill marked `@skill(uses=[CAP_MOVEMENT])` while another skill holds movement. A call made as a plain RPC, or a message published to `cmd_vel`, does not pass through McpServer.

A message that starts with `[DimOS event]` comes from an agent module that this conversation created, not from the user. Act on it as that module's purpose says. Notices arrive live over MCP. Once admitted, they use the conversation's normal persistent input queue; notices missed while disconnected are not recovered.

The `job` tool keeps cron jobs: `create`, `update`, `delete`, `list`, `get`, and `run` to fire one now. A job has a 5-field cron expression and an IANA timezone, which is this machine's zone when omitted. Ask for the location or timezone instead of guessing. Cron cannot express sunset, so state the fixed time you chose. A job's run is either `{"notify": TEXT}`, which starts a turn in this conversation with the message `[Job NAME] TEXT`, or `{"skill": {"name": NAME, "arguments": {...}}}`, which calls a skill through McpServer without a turn. A run that fell due while the gateway was down runs once, late, when the gateway is back.
