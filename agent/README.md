# dimcode

Persistent terminal agent for DimOS, built on Pi. The gateway runs independently of robot blueprints. Existing MCP exposes skills; Pi's coding tools use the existing DimOS CLI/Python APIs. Media uses the existing Web SDK only while a renderer needs it.

## Install

The local executable is ready to use:

```sh
dimcode setup
dimcode
```

`dimcode` opens the fullscreen Pi-based terminal: scrollable chat, a multiline editor with command/file completion, compact tool output, and a persistent model/status footer. Shift+Enter adds a line; Ctrl-C detaches from the running gateway.

Memory results appear **inside the agent's tool card**, with an overview combining DimOS's timeline, spatial map and selected frames. Click a panel label or enter `/panel 1`, `/panel 2`, etc. to see a larger individual view; `/panel 0` returns to the overview. `/expand` shows the latest tool's full output and source hashes. Use a terminal with image support (such as Kitty) for the SVG/PNG views; other terminals show image metadata.

To inspect a saved cloud without a model call, enter these commands **inside dimcode**:

```text
/inspect /absolute/path/to/cloud.json
/view
```

Cloud exports use `{ "points": [[x, y, z]], "frame": "world", "timestamp": 123.45 }`. In the inspector, **←/→** rotates, **+/-** zooms, **g** switches graphics/Braille, **d** shows source/hash, **0** resets the view, and **Esc** returns to chat. An export can supply `selectedIndices` to highlight the exact selection produced by its owning operation. Rotation and zoom change the view only; memory analysis and filtering stay in DimOS.

Point-cloud results from `dimcode_render` open the same inspector through `/view`. The terminal verifies the source hash before loading interactive geometry; if the file changed, the original tool preview remains available. `/expand` toggles details for the latest tool. Images use supported terminal graphics; Braille supports point-cloud inspection without image support.

`dimcode setup` walks through:

1. Provider and authentication: paste an API key into a masked field, use existing credentials, or sign in with a ChatGPT subscription.
2. Model selection.
3. Workspace directory.
4. DimOS: choose an existing installation, create a new environment, or connect later.
5. An existing DimOS MCP endpoint, if available.
6. Daemon setup: select **Start at login** to install and enable the systemd user service, or start the gateway when you open dimcode.

To install the packaged local build on another machine, use Node **24 or 26** and the supplied tarball:

```sh
npm install -g ./dimensional-dimcode-0.1.0.tgz
dimcode
```

This prerelease is distributed locally; it is not published to npm yet. No source build or test commands are needed to install the tarball.

A fresh `dimcode` launch opens setup automatically. Repeating `dimcode setup` lets you change the choices; configured providers offer **Use configured credentials** so you do not need to paste a key again. Credentials never enter chat.

```sh
dimcode setup       # repeat interactive setup
dimcode tui         # open the terminal (same as dimcode)
dimcode --help      # all launch commands
```

The setup can create a new DimOS environment using `uv` (Python 3.12), or select a `dimos` executable from a pip environment or editable checkout. Install [uv](https://docs.astral.sh/uv/getting-started/installation/) first if you choose a new environment. Use DimOS's installation instructions for robot extras and system packages. The harness can run before DimOS is installed.

## What is installed / where is the source?

This package uses **upstream Pi 0.85.1**, pinned as npm dependencies. It is not a Pi fork or a native binary. `dimcode` is a Node CLI with a Dimensional gateway, terminal frontend and extensions. Pi supplies the agent runtime, provider login, coding tools, message components and tool cards. A fork is unnecessary for these customizations.

All source is in this repository's [`agent/src/`](src): [`main.ts`](src/main.ts) launches the CLI; [`setup.ts`](src/setup.ts) handles onboarding; [`terminal.ts`](src/terminal.ts) renders chat; [`gateway.ts`](src/gateway.ts) owns persistent sessions; [`media.ts`](src/media.ts) lazily connects the existing Web SDK. [`skills/dimensional`](skills/dimensional) supplies DimOS instructions.

## Authentication and service

API keys are entered outside chat and stored by Pi in the user-only auth file. An environment key can be installed with `dimcode setup --provider openai --key-env OPENAI_API_KEY`. ChatGPT subscription login uses `--provider openai-codex --oauth`. Use Anthropic API keys for Claude in dimcode. Refresh/logout use Pi's provider implementation.

```sh
dimcode service install
dimcode service status
dimcode service uninstall
```

This is a Linux **user** service. Without it, first attach starts a detached gateway. `dimcode stop` stops that gateway; `dimcode gateway` runs it in the foreground for diagnostics, or reports that it is already running. `dimcode --foreground` runs gateway and terminal in one foreground process.

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

Terminal commands: `/new`, `/sessions`, `/resume ID`, `/models`, `/model PROVIDER MODEL`, `/login PROVIDER [oauth]`, `/logout PROVIDER`, `/abort`, `/steer TEXT`, `/follow TEXT`, `/reload`, `/image PATH`, `/panel N`, `/inspect PATH`, `/view`, `/expand`, `/exit`. Ctrl-C detaches. One terminal owns input; other viewers may observe. Detaching keeps the turn running. Restart restores Pi history and never automatically replays external actions. Very large histories show a bounded recent transcript with an omission notice; the complete agent history remains on disk. Session events carry their source identity so switching sessions cannot mix transcripts.

Pi owns context loading, skills, compaction, models and coding-tool behavior. Workspace instructions and configured Pi extensions load normally. Attached terminals support serialized dialogs/notifications; executable extension UI factories belong in the terminal renderer and cannot be sent through a socket.

## Tool rendering

DimOS memory owns video/frame/point-cloud analysis. Evaluate the memory operation once with `materialize()`, then use its existing `Space.to_svg()` / `Plot.to_svg()` and selected-frame exports. Pass those exports together to `dimcode_render`:

```json
{"kind":"image","title":"Memory · plant search","views":[{"label":"Timeline","path":"timeline.svg"},{"label":"Spatial","path":"space.svg"},{"label":"Frames","path":"frames.png"}]}
```

The tool rasterizes the original SVG/PNG views for terminal display and model context. Every view reaches the model; the terminal presents a selectable overview. Each export retains its source path and SHA-256. MCP tool results containing multiple images also appear together automatically. No query or filter runs inside the renderer. Existing skills that return only a pose or JSON still require an explicit export; the harness does not invent missing views.

`dimcode_render` also accepts single `image` paths and saved `points` / `series` JSON. Point clouds use XYZ rows; series use timestamp/value pairs. Selection metadata and display decimation remain explicit. See the bundled Dimensional skill for formats.

Live tools select an existing relay/robot/channel. The terminal receives frames directly through the Web SDK and coalesces drawing to 10 Hz. The gateway retains one final snapshot for model context. Closing/cancelling the tool releases consumers; the last consumer closes the connection. MediaPool is generic over decoded SDK slots and accepts existing decoder registries. The initial live image renderer handles JPEG; other channel types use their owning decoder/renderer or saved exports.

Graphics use Pi terminal-image support with text fallback. Derived PNGs have a bounded 128 MiB cache; original DimOS recordings stay with DimOS. No recording, raw continuous video, new transport protocol or new DimOS gateway is introduced.

## Contributor development and validation

These commands are for working on the source, not the user installation:

```sh
cd agent
npm ci
npm run check
npm link
```

`npm pack` produces the distributable tarball; `npm publish --access public` requires access to the `@dimensional` npm scope. Building alone does not publish it.

To run the complete integration suite with an existing DimOS environment and Go2 recording:

```sh
DIMCODE_TEST_PYTHON=/path/to/dimos/.venv/bin/python \
DIMCODE_TEST_DENO=/path/to/deno \
DIMCODE_TEST_GO2_DB=/path/to/go2_bigoffice.db \
npm test
```

The Go2 E2E test starts the standard `unitree-go2` blueprint in **recorded-data replay**, composed with the existing cockpit relay bridge. It starts its own relay and isolates transport discovery. No robot hardware or model API key is required. It verifies:

- Multiple distinct, successfully decoded JPEG video frames and timestamped odometry.
- Full XYZ float32 point clouds, every coordinate finite, source timestamps/frame metadata, and byte-for-byte source SHA-256 matches. A test-only encoder/SDK decoder uses the existing codec registry; it does not mislabel the default XY lidar projection as XYZ.
- One shared SDK connection, lazy subscriptions, continued cloud reception after video closes, then zero viewers/subscriptions after the last renderer closes.
- Cleanup of the blueprint and relay, including failure paths.

Set `DIMCODE_TEST_REPORT=/absolute/path/report.json` to save measured counts. Without `DIMCODE_TEST_GO2_DB`, the large recording test is explicitly skipped. The Python and Deno variables separately enable the MCP-handler and QUIC relay tests. Unit/CLI tests also cover onboarding, private credentials, cancellation, `tui`, existing-gateway handling, session ownership, detach/recovery and renderer provenance.

A video stream here means consecutive JPEG frames over WebTransport. This does not claim an H.264/WebCodecs decoder, browser UI coverage, or a physical robot test. The harness's initial live terminal renderer displays JPEG; saved point clouds and series render from exact exported results. Other live types can plug into the generic SDK codec registry.
