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

Use dimcode_render for an existing saved image or JSON export: points come from the existing point cloud extraction API as XYZ rows; series are numeric [timestamp,value] rows derived from the evaluated operation. Retain original exports and their metadata. The renderer labels display projection/decimation and hashes the source file. It must never rerun a query to obtain a prettier display.

For live rendering, explicitly choose the existing relay, robot and channel. Web SDK subscriptions open only for the tool/view's lifetime. Label the view live: it is not a historical selection, and relay header timestamps may be send time. Only a selected final snapshot belongs in model context. Non-image encodings need their existing domain decoder/renderer or an explicit saved export.

For validation, start with the standard unitree-go2 and unitree-go2-agentic blueprints in replay mode. A running gateway does not require a blueprint or its LLM client. Do not infer a physical robot target from a default IP. Inspect non-moving skills first on explicitly selected hardware.
