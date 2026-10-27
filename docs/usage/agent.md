# The on-robot agent

`dimos agent` runs Dimensional's on-robot agent. It discovers the modules, streams and RPCs of the running dimOS system, reads sensors, writes Python against the dimOS API, and deploys agent modules that keep working after its turn ends. Events from those modules and from cron jobs wake it.

The agent has two parts. The gateway is one Node process that hosts every conversation and keeps running when no terminal is attached and when dimOS restarts. HarnessRuntime is a dimOS module that the gateway loads into each running system and reaches through McpServer.

## Set up and start the agent

You need a dimOS checkout with its `.venv`, which `bash scripts/install.sh --mode dev --project-dir .` creates, and Node.js 24 or 26 first on `PATH`. The agent runs from the checkout. A dimOS installed from a wheel does not include it.

1. Build the agent:

   ```sh skip
   dimos agent version
   ```

   Every `dimos agent` command except `gateway`, `stop` and `status` first runs `npm ci` and `npm run build` in `dimos/agents` when their inputs changed, so after a pull the next command rebuilds. The first install needs network access and uses the npm installed with your Node. While a gateway runs this checkout's build, a dependency change stops with instructions instead of installing. Build once before `dimos agent service install`, because the service's `gateway` never builds.

2. Start a dimOS system. Any blueprint works. The loader adds HarnessRuntime, McpServer, and RelayBridgeModule when missing. It reuses an existing relay bridge and leaves configuration resolution to that bridge. An automatically loaded bridge uses JPEG previews and does not open a browser. No separate Zenoh bridge setup is needed.

   Automatically loaded bridges use `rtc=False`, `open_browser=False`, and `web_build=False`. An existing authored bridge using RTC with an SFU must set `rtc=False` to provide Node image previews. Existing bridge configuration stays under its owner's control. The SDK reports an explanatory error for an RTC source.

   ```sh skip
   dimos --replay run unitree-go2 --daemon
   ```


3. Open a terminal on a new conversation. `dimos agent` starts the gateway first if none runs:

   ```sh skip
   dimos agent
   ```

4. Type `/login` and choose a model provider. The gateway saves the credential in `~/.config/dimos/agent/auth.json`.

The gateway does not read the checkout's `.env`, so a provider key there does not reach it. To give the gateway an API key, enter it through `/login`. If `~/.dimcode` exists, the first gateway start copies its `auth.json` and settings once and leaves the originals in place.

## Attach to a conversation

Every conversation accepts input at any time, whether or not a terminal is attached. To pick one up again, list them and attach by number:

```sh skip
dimos agent conversations
dimos agent --conversation 3
```

`--view` with `--conversation ID` attaches read-only, so you can watch a conversation that another terminal owns. `--cwd DIR` starts a new conversation whose files and commands live in `DIR` instead of the checkout. To ask one question without a terminal, run `dimos agent run "which cameras are running?"`, which prints the answer.

In the terminal, `/help` lists the commands, such as `/model` to change the conversation's model and `/logout` to remove a credential, and the keys below.

## The terminal

The chat shows the conversation's inputs, answers and tool calls. A message sent while the agent is busy shows as `queued` under the chat, in order, until the agent takes it. With an empty editor, Up brings every queued message of yours back into the editor; Enter queues the edited text again. When the agent reaches the next boundary it takes all queued messages in one turn.

The stream sidebar is the column beside the chat. Ctrl+T shows or hides it, and Shift+Tab moves focus between chat and the sidebar. Up and Down select a stream, Left hides its body, Right shows it, and Esc returns to chat. The selected stream scrolls into view.

The sidebar shows the conversation's agent modules and their streams, jobs, and queued notices. It also shows the system's image, point-cloud, and occupancy-grid streams. Expanded previews show the SDK's received rate and age. Pictures use terminal graphics, colored cells, or ASCII, depending on terminal support. The sidebar needs at least 100 terminal columns.

The terminal opens its own Web SDK session using `inventory.environment.relay`. The relay bridge discovers native streams and selects their codecs. Point-cloud previews show voxel centers. The local `stream_get` tool reads from the harness's SDK session and exports previews at the relay's resolution. Use `stream_get op=list` to list readable channel IDs, then `stream_get op=read channel=ID`. Reads accept `mode=latest|next`, `timeout_s`, and `export=true`. Each read closes its SDK subscription when it ends. Stream additions and removals arrive through live relay manifests.

Read module diagnostics with `dimos log`. dimOS writes rotating run logs, normally in `~/.local/state/dimos/logs/<run-id>/main.jsonl`. In agent modules, use `setup_logger()` from `dimos.utils.logging_config` for diagnostics and `notify()` for meaningful alerts that need the conversation to act. Callback exceptions include the module and stream in the run log. Module notices arrive live over MCP. Inputs already admitted through InputGate use the normal persistent conversation queue.

Image previews use the terminal's available graphics or text mode over SSH. To force text previews, run `PI_IMAGE_PROTOCOL=none dimos agent`. Add `NO_COLOR=1` for ASCII previews.

Ctrl+O opens the conversation switcher. Every conversation is listed with its state, what it is running, its unread notices and who is attached; a fork is indented under its source. Typing filters the list. Enter attaches and takes the input, Tab attaches read-only, Ctrl+N starts a new conversation, Esc closes.

With nothing running and an empty editor, Esc selects a message. The arrows move between your messages and the agent's answers, and the chat scrolls to the selected one. On a message of yours, `e` puts its text in the editor; Enter then sends the edited text and the message's images into a fork of the conversation that continues from before that message, and the terminal attaches to the fork. On any message, `u` summarizes the conversation up to it into a new fork, and `f` summarizes from it. If you switch to another conversation before the fork is ready, the terminal stays where you are and names the fork. The original conversation stays as it was.

A long-running tool call that allows it, such as `bash`, shows `Ctrl+B background` while it runs. Ctrl+B moves the call to the background: the card shows that it moved, the turn goes on, and the call's result comes back later as a notice in the chat.

`/menu` opens the main menu full screen. Its Jobs tab shows every job as a lane over the last and next 12 hours, with a mark per run by outcome and a circle per upcoming due time; `r` runs the selected job now and `d` deletes it.

| Key | What it does |
|---|---|
| Ctrl+T | Show or hide the stream sidebar |
| Shift+Tab | Focus the stream sidebar or return to chat |
| Up / Down (sidebar) | Select a stream and scroll it into view |
| Left / Right (sidebar) | Hide or show the selected stream's body |
| Esc (sidebar) | Return to chat |
| Ctrl+O | Open the conversation switcher |
| Ctrl+B | Send the running tool call to the background |
| Esc | While running, stop the run. With an empty editor, select a message (then ↑/↓, `e`, `u`, `f`) |
| Up | With an empty editor, bring your queued messages back for editing |
| Ctrl-C | Clear the editor. With an empty editor, detach; the conversation keeps running |

## Run the gateway as a service

On a robot, run the gateway as the systemd user service `dimos-agent.service`, so it starts without a terminal and restarts when it fails:

```sh skip
dimos agent service install
```

`install` writes `~/.config/systemd/user/dimos-agent.service`, stops a gateway that `dimos agent` started, and enables and starts the service. The unit runs `dimos agent gateway` from this checkout's `.venv`, with the checkout as its working directory. It keeps the `PATH` of the shell that ran `install`, so the service finds the same Node. It also keeps that shell's `XDG_CONFIG_HOME`, `XDG_STATE_HOME` and `XDG_CACHE_HOME` when they are set, so the service and your terminals use the same directories.

systemd runs a user service only while you are logged in, unless lingering is on for your user. `install` prints a note when lingering is off. To start the gateway at boot, turn lingering on:

```sh skip
loginctl enable-linger "$USER"
```

`dimos agent service status` shows whether the unit file matches what `install` would write now and prints `systemctl --user status`. After you move the checkout or change Node, run `install` again. After a rebuild, run `systemctl --user restart dimos-agent.service`. `dimos agent stop` stops the gateway cleanly, and systemd does not restart a gateway that stopped cleanly. `dimos agent service uninstall` stops and disables the service and removes the unit. All three commands take `--no-systemctl`, which touches only the unit file.

## What the agent can do on the machine

`bash` runs as the user that runs the gateway, with that user's files, network, processes and devices. The agent has no sandbox and asks for no approval before a command. `bash` has no terminal, so it cannot answer a `sudo` password prompt, and passwordless `sudo` works only as far as the host's sudo policy allows. Run commands that need a password yourself, in a local terminal. One example is dimOS's transport autoconfiguration, which changes socket buffers and multicast settings through `sudo`. Do not run the gateway as root.

## Commands

| Command | What it does |
|---|---|
| `dimos agent [--conversation ID] [--view] [--cwd DIR]` | Opens a terminal. Without `--conversation`, it starts a new conversation. It starts the gateway if none runs. |
| `dimos agent run [--conversation ID] [--cwd DIR] [--json] PROMPT` | Sends `PROMPT` to a new conversation, or to conversation `ID`, and prints the answer. With `--json`, it prints the new transcript items and then the outcome as JSON lines. |
| `dimos agent conversations` | Lists the conversations, newest first. |
| `dimos agent gateway` | Runs the gateway in the foreground. The service runs this command. |
| `dimos agent status` | Shows whether the gateway runs. A socket file that nothing answers is reported as stale. |
| `dimos agent stop` | Stops the gateway. |
| `dimos agent version` | Prints the agent's version and build. |
| `dimos agent load` | Load missing HarnessRuntime, McpServer, and RelayBridgeModule instances; reuse an existing relay bridge |
| `dimos agent service install\|status\|uninstall` | Manages `dimos-agent.service`. |

Every command except `load` and `service` runs `node dimos/agents/dist/cli.js` with the same arguments. Before it starts Node, `dimos` checks the Node version and the build. It sets `DIMOS_ROOT`, `DIMOS_PYTHON`, `DIMOS_CLI`, `DIMOS_MCP_URL` and `DIMOS_RUNS_DIR` to tell the gateway where dimOS is. `DIMOS_MCP_URL` is the endpoint of the `McpServer` that `dimos agent load` adds. Both come from `McpServerConfig`: its `host` defaults to `listen_host` and its `port` to 9990, and `MCPSERVER__HOST` and `MCPSERVER__PORT` in the environment or `.env` override them, as they do for `dimos run` (see [Configuration](/docs/usage/configuration.md)). For example, `MCPSERVER__PORT=9000` in `.env` puts the gateway and the server it loads on port 9000. If the running system already has an `McpServer` that the gateway cannot reach this way, such as one started with `--mcpserver.port 9000` while the agent has no such setting, `dimos agent load` fails and names both endpoints.

A running gateway keeps the build it started with. After a rebuild, `dimos agent`, `run` and `conversations` refuse a gateway of the old build, and `status` reports it. Restart the gateway with `dimos agent stop`, or with `systemctl --user restart dimos-agent.service` when it runs as the service.

## Where the agent keeps its state

| Path | Owner | Contents |
|---|---|---|
| `~/.config/dimos/agent/` | gateway | `auth.json` and `config.json`, copied once from `~/.dimcode` when it exists |
| `~/.config/dimos/agent/models.json` | you | Optional provider overrides in pi's `models.json` format, such as a base URL |
| `~/.config/dimos/agent/skills/` | you | Your own skills, one `NAME/SKILL.md` each. The agent lists them after the bundled ones in `dimos/agents/harness/skills/`. |
| `~/.config/systemd/user/dimos-agent.service` | `dimos agent service install` | The service unit |
| `~/.local/state/dimos/agent/agent.sqlite` | gateway | Conversations, entries, tasks, submissions and documents, including jobs and the event and trajectory positions |
| `~/.local/state/dimos/agent/tool-results/` | gateway | Full text of cut tool results and inputs |
| `~/.local/state/dimos/agent/gateway.log` | gateway | The gateway's log |
| `~/.cache/dimos/agent/render/` | gateway | Rendered previews of images, point clouds and clips, pruned past 128 MiB |
| `$XDG_RUNTIME_DIR/dimos-agent.sock` | gateway | The socket that terminals attach to |
| `~/.local/state/dimos/harness/` | HarnessRuntime | Agent module metadata in `modules/` and notice attachments in `exports/` |
| `~/.local/state/dimos/modules/dimos_agent/` | HarnessRuntime | Agent module source files and their revisions |
| `~/.local/state/dimos/logs/<run-id>/main.jsonl` | dimOS | Rotating run logs, including module diagnostics |
| `~/.local/state/dimos/runs/` | dimOS | One file per running system, watched by the gateway |

Without `XDG_RUNTIME_DIR`, the socket is `~/.local/state/dimos/agent/dimos-agent.sock`. Only the gateway opens `agent.sqlite`. Other programs go through the gateway.

## How agent modules and events work

The agent does one-off work by writing a Python script that uses `Dimos.connect()` and running it with `bash`. Work that has to continue after the turn, such as a watcher, a follower or a control loop, is an agent module. HarnessRuntime's `agent_module` tool runs each module from one source file, `~/.local/state/dimos/modules/dimos_agent/NAME.py`. The agent writes the file and creates the module, then edits the file and restarts the module. A restart compiles the file into a new revision, `NAME_rN.pyc`, and runs it inside dimOS on its own worker. A file that does not compile or import leaves the old revision running. An agent module subclasses `AgentModule`, which is a `Module`. Its `In` and `Out` ports join running streams with the same name and type, and an attribute typed with a module's Spec calls that module's RPCs.

An agent module reports back with `notify(text, **data)`. One notice takes this route:

1. `notify` saves every `Image` and `PointCloud2` in `data` to `~/.local/state/dimos/harness/exports/` and publishes an `AgentNotice`. It sends at most one notice per `notify_interval_s`, 10 seconds by default.
2. HarnessRuntime finds the conversation that created the module and sends a `dimos/harness_event` notification through McpServer. Each notice has a UUID `id`, `module`, `conversation`, `at`, `text`, `data`, `exports`, and `suppressed`.
3. The gateway submits the event to that conversation, with the exported files rendered as images. The conversation starts a turn when it is idle and queues the event after the current run when it is busy, with or without a terminal attached.

EventIntake handles each live notification through EventDrafts and InputGate. The request ID is `dimos-event:<uuid>`, so resubmitting that notice refers to the same input. Notices missed while disconnected are not recovered. Once InputGate admits a notice, it has the same persistence as other conversation inputs.

A cron job created with the `job` tool runs either `{"notify": TEXT}` or `{"skill": {...}}`. A notify run enters the same route at step 3 and starts a turn in the conversation that created the job. A skill run calls the skill through McpServer without a turn. A run that fell due while the gateway was down runs once, late, when the gateway is back.

The conversation store retains admitted inputs and queued work. Interrupted tool handling follows each tool's policy. When dimOS restarts, the gateway sees the new file in `~/.local/state/dimos/runs/` and runs `dimos agent load`. Agent modules stop with dimOS and do not come back on their own.

Two gaps remain. Nobody sees the agent's answer to an event until someone opens the conversation. Event turns have no cost limit, so a module that notifies every 10 seconds starts a model turn every 10 seconds.
