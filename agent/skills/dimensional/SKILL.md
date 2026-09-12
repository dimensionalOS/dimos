---
name: dimensional
description: Install DimOS, build and operate apps/blueprints, invoke deployed skills, and inspect real sensor/memory results.
---

Use the selected DimOS executable/Python environment and workspace from the session instructions. Check installed help when versions differ. Never replace an existing environment silently.

- Installation: use `dimcode install-dimos NEW_VENV` for the base package, or the existing DimOS installation recipe for required robot extras. For a checkout use its documented uv setup. Harness setup and model auth are independent of DimOS installation.
- Apps: `dimos list`, `dimos show-config`, `dimos bake`, source editing, and external package entry points in the `dimos.blueprints` group. Run external apps by the qualified `distribution-name.blueprint-name` shown by `dimos list`; bare names select built-ins. Do not edit the built-in registry for an external app.
- Runtime: `dimos run BLUEPRINT --daemon`; inspect `dimos status --run ID`, `dimos log --run ID`, `dimos stop --run ID`, `dimos restart --run ID`. Older versions may lack exact targeting: use the existing public run_registry APIs, never silently act on the latest run instead.
- Python-launched coordinators: `Dimos.connect()` and its public module/RPC inspection APIs; `stop()` on that connection disconnects only. Use separate configured transport buses/endpoints for independent coordinators.
- All deployed @skill tools come from the selected MCP endpoint. Use those tools to preserve capability arbitration and background stop semantics. Direct RPC is not a substitute for the MCP dispatcher. On endpoint changes use /reload. Missing tools do not mean every robot dependency should be installed.
- Full CLI access stays in Bash: runtime/config/apps; mcp and agent-send; shell/spy/lcmspy/agentspy/humancli/top; topic/map/rerun-bridge; mem/data/dataprep/cache; nav-eval/evals; hardware/go2tool/calibration; login/logout/whoami. Interactive programs need an interactive terminal, not a background Bash call.
- Dimensional cloud login and model-provider login are separate. Never ask for credentials in chat or tool arguments; use the terminal /login or setup flow.

For memory, use existing stream operations. Evaluate a query once with materialize(), then derive scores, selected peaks, images and comparisons from that same cache. Preserve timestamps, observation IDs and coordinate frames in exports. The docs reconstruction example includes a skipped block/API TODO; verify the chosen callable operation on actual data.

Show a visualization for each meaningful sensor/memory operation. Prefer a supported type or an existing DimOS visualizer. If neither supports the result, use inline Python to generate a self-contained SVG from the evaluated data, save it, and call `dimcode_render({kind:"image",path:"result.svg"})`. This fallback also supports custom plots, simple graphs and proposed overlays; label proposals distinctly from observations. Use the returned image as visual feedback when interpreting or refining the result. If rendering fails, report that explicitly and retain the original result.

DimOS owns reusable, agent-independent visualization semantics: geometry, axes, units, timestamps, legends, selections and generic encodings for agent understanding. Dimcode owns terminal-specific styling and interaction: card layout, theme, viewport/camera defaults, point appearance, playback controls, resizing and graphics/Braille adaptation. Terminal styling must preserve semantic colors and labels supplied by DimOS or a custom SVG. Keep terminal-specific presets out of DimOS.

All analysis of point-cloud frames and video belongs in DimOS memory. Prefer its existing `Space.to_svg()` and `Plot.to_svg()` renderers, alongside selected-frame exports from the same materialized query. Display the views together with `dimcode_render({kind:"image", title:"Memory · QUERY", views:[{label:"Timeline",path:"timeline.svg"},{label:"Spatial",path:"space.svg"},{label:"Frames",path:"frames.png"}]})`. Supply only views actually produced by that operation. Preserve the native colors, timestamps, peaks, coordinate frames and selection metadata; don't reconstruct them in the harness. The terminal displays an overview and individually selectable panels, and the model receives every image. MCP tools returning multiple `agent_encode` images are displayed together automatically; a skill returning only a pose does not provide these views.

Single saved images and JSON exports remain supported: points use existing point-cloud extraction as XYZ rows with optional `selectedIndices`; series use numeric [timestamp,value] rows from the evaluated operation. Retain original exports and metadata. The renderer labels display projection/decimation and hashes every source. It must never repeat a memory query to obtain a prettier display.

For live rendering, explicitly choose the existing relay, robot and channel. Web SDK subscriptions open only for the tool/view's lifetime. Label the view live: it is not a historical selection, and relay header timestamps may be send time. Only a selected final snapshot belongs in model context. Non-image encodings need their existing domain decoder/renderer or an explicit saved export.

For validation, start with the standard unitree-go2 and unitree-go2-agentic blueprints in replay mode. A running gateway does not require a blueprint or its LLM client. Do not infer a physical robot target from a default IP. Inspect non-moving skills first on explicitly selected hardware.
