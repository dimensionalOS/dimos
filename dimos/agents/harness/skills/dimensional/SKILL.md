---
name: dimensional
description: Build and operate DimOS apps and blueprints with the dimos CLI and Python API, and inspect real sensor and memory results.
---

`bash` runs with DimOS's Python environment first on `PATH` and `DIMOS_ROOT` set to the DimOS checkout. DimOS reads `.env` from the working directory, so run DimOS commands and scripts from `$DIMOS_ROOT` unless the task needs another directory. When a command behaves differently than you expect, check its `--help` for the installed version.

- Apps: `dimos list`, `dimos show-config`, `dimos bake`, source edits, and external packages that expose blueprints through the `dimos.blueprints` entry point group. Run an external app by the qualified `distribution-name.blueprint-name` that `dimos list` shows. A bare name selects a built-in. Do not edit the built-in registry for an external app.
- Running systems: `dimos run BLUEPRINT --daemon`, then `dimos status`, `dimos log`, `dimos stop` and `dimos restart`. Only `dimos log` takes `--run ID`. The others act on the latest run, so when several systems run, target one through `list_runs()` and `stop_entry()` in `dimos.core.run_registry`. Never act on the latest run in place of the one you mean.
- Python: `Dimos.connect()` attaches a script to the running system, and `app.list_rpcs()` and `app.describe()` discover it from code. `docs/usage/python-api.md` in the checkout explains the API. `stop()` on that connection disconnects and leaves the system running. Independent coordinators need separate transport buses or endpoints.
- Skills: a module's `@skill` methods are RPCs, and a script calls them like any RPC. `dimos mcp call NAME --arg KEY=VALUE` calls one through McpServer instead, which refuses a skill while another skill holds a capability it needs, such as movement.
- The rest of the CLI works from `bash` too: `mcp`, `agent-send`, `topic`, `map`, `rerun-bridge`, `mem`, `data`, `dataprep`, `cache`, `evals`, `hardware`, `go2tool`, `cameracalibrate`, `apriltag`, `login`, `logout` and `whoami`. `shell`, `spy`, `lcmspy`, `agentspy`, `humancli` and `top` need an interactive terminal, so do not run them through `bash`.
- Dimensional cloud login (`dimos login`) and model-provider login are separate. Never ask for credentials in chat or in tool arguments. The user logs in to a model provider with `/login` in the terminal.

For the running system (module APIs, streams, agent modules and cron jobs), read the dimos-runtime skill.

For memory, use the existing stream operations. Evaluate a query once with `materialize()`, then derive scores, selected peaks, images and comparisons from that one result. Keep timestamps, observation IDs and coordinate frames in exports. Analysis of point-cloud frames and video belongs in DimOS memory, whose `Space.to_svg()` and `Plot.to_svg()` draw spatial and timeline views. Try a memory operation on real data before you rely on it.

Files that a `bash` command creates or changes in its working directory, one folder below it or the temp directory, and existing files whose paths appear in its output, are shown to you and to the user when they are one of these:

- a PNG, JPEG, GIF, WebP, SVG, TIFF or AVIF image;
- point-cloud JSON, `{"points": [[x, y, z]], "colors"?: [[r, g, b]], "frame"?, "timestamp"?, "selectedIndices"?}`, with colors from 0 to 255;
- a frame index, `{"type": "points" | "image", "timeOrigin"?, "source"?, "frames": [{"path", "timestamp", "sha256"?}]}`, with each path relative to the index and timestamps that strictly increase.

To show a file saved anywhere else, print its path.

A frame index plays as a loop for the user. You receive a contact sheet with each frame's timestamp, and you read any single frame with `read`. Do not claim to have inspected every frame from the contact sheet alone. For a recorded interval, materialize it once in DimOS, export its frames, and write the index with the recording's origin as `timeOrigin`.

For validation, start with the `unitree-go2` blueprint in replay mode: `dimos --replay run unitree-go2 --daemon`. Do not infer a physical robot's address from a default IP.
