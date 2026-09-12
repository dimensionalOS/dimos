# dimcode

Persistent terminal agent for DimOS, built on Pi. The gateway runs independently of robot blueprints. Existing MCP exposes skills; Pi's coding tools use the existing DimOS CLI/Python APIs. Media uses the existing Web SDK only while a renderer needs it.

## Install

Requires Node 24 or 26. From this checkout:

```sh
cd agent
npm ci
npm run check
npm pack
npm install -g ./dimensional-dimcode-0.1.0.tgz
dimcode setup --provider openai
dimcode
```

The package name is `@dimensional/dimcode`; npm publication is separate from building/installing the tarball. DimOS is optional at harness startup. To install its base package in a **new** environment:

```sh
dimcode install-dimos /path/to/new/venv
```

Use DimOS's existing installation instructions for robot extras/system packages. Existing installations and editable checkouts can be selected with `dimcode python /path/to/python` and `dimcode dimos /path/to/dimos`. Executable paths are local configuration, and each session has its own workspace.

## Authentication and service

API keys are entered outside chat and stored by Pi in the user-only auth file. An environment key can be installed with `dimcode setup --provider openai --key-env OPENAI_API_KEY`. ChatGPT subscription login uses `--provider openai-codex --oauth`. Use Anthropic API keys for Claude in dimcode. Refresh/logout use Pi's provider implementation.

```sh
dimcode service install
dimcode service status
dimcode service uninstall
```

This is a Linux **user** service. Without it, first attach starts a detached gateway. `dimcode stop` stops that gateway; `dimcode gateway` runs it in the foreground for diagnostics. `dimcode --foreground` runs gateway and terminal in one foreground process.

Config/auth live in `$XDG_CONFIG_HOME/dimcode` (or `~/.config/dimcode`), sessions in `$XDG_STATE_HOME/dimcode`, previews in `$XDG_CACHE_HOME/dimcode`. `DIMCODE_HOME` overrides the config directory. The runtime socket is user-only. Service removal preserves sessions, credentials and DimOS data.

## Connect and work

```sh
dimcode connect go2 http://127.0.0.1:9990/mcp
dimcode relay http://127.0.0.1:7780 my-robot
dimcode --cwd /path/to/app
dimcode sessions
dimcode --session SESSION_ID
dimcode --session SESSION_ID --view
dimcode run "inspect the app and explain its blueprint"
```

Endpoints are explicit; ports are examples, not instance identities. Use `/reload` after changing endpoints. Every advertised skill is registered with its original schema, metadata and remote routing. Tool names are bounded and collision-resistant. There is no static copy of the robot's tools or a parallel lifecycle service.

Terminal commands: `/new`, `/sessions`, `/resume ID`, `/models`, `/model PROVIDER MODEL`, `/login PROVIDER [oauth]`, `/logout PROVIDER`, `/abort`, `/steer TEXT`, `/follow TEXT`, `/reload`, `/image PATH`, `/expand`, `/exit`. Ctrl-C detaches. One terminal owns input; other viewers may observe. Detaching keeps the turn running. Restart restores Pi history and never automatically replays external actions.

Pi owns context loading, skills, compaction, models and coding-tool behavior. Workspace instructions and configured Pi extensions load normally. Attached terminals support serialized dialogs/notifications; executable extension UI factories belong in the terminal renderer and cannot be sent through a socket.

## Tool rendering

`dimcode_render` displays existing image, point-cloud or numeric-series exports and returns the selected PNG to the model. Point clouds use XYZ rows; series use timestamp/value pairs. See the bundled Dimensional skill for formats. The original file, SHA-256, selection metadata and display decimation remain explicit. The renderer never repeats a memory query.

Live tools select an existing relay/robot/channel. The terminal receives frames directly through the Web SDK and coalesces drawing to 10 Hz. The gateway retains one final snapshot for model context. Closing/cancelling the tool releases consumers; the last consumer closes the connection. MediaPool is generic over decoded SDK slots and accepts existing decoder registries. The initial live image renderer handles JPEG; other channel types use their owning decoder/renderer or saved exports.

Graphics use Pi terminal-image support with text fallback. Derived PNGs have a bounded 128 MiB cache; original DimOS recordings stay with DimOS. No recording, raw continuous video, new transport protocol or new DimOS gateway is introduced.

## Validation

```sh
npm run check
DIMCODE_TEST_PYTHON=/path/to/dimos/python DIMCODE_TEST_DENO=deno npm test
```

The second command adds actual DimOS MCP-handler and Node-to-Deno QUIC integration tests. Other tests cover session ownership/detach/recovery, duplicate prompts, renderer provenance and lazy media disposal. Fixture tests do not claim a physical-robot run.

Use `dimos --replay run unitree-go2 --daemon` for standard recorded-data validation. External apps are selected as `distribution-name.blueprint-name`. Lifecycle commands in this change support exact `--run ID` targeting. A Python-launched coordinator can instead be inspected with existing public Python APIs.
