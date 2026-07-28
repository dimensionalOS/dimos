## Why

DimOS exposes running modules through coordinator RPC and the `Dimos` Python interface, but developers lack a first-class interactive entry point for discovering modules and calling their RPC methods. Interactive debugging currently requires ad hoc Python commands or specialized clients that expose only part of a robot stack.

Adding a general DimOS shell makes the existing RPC surface directly usable from an attached IPython session. It also gives CLI-launched and direct Python-launched coordinator loops the same attachment model.

## What Changes

- Add the public `dimos shell` CLI command for attaching an IPython session to a running coordinator.
- Add public `Dimos` discovery methods for live module and RPC inspection, exact module-instance lookup, and structured descriptions.
- Preserve Python signatures and documentation on RPC proxy callables.
- Make coordinators attachable when they enter `ModuleCoordinator.loop()`, including coordinators launched directly from Python.
- Require an interactive terminal, display an immediate-execution safety notice, and disconnect without stopping the running system when the shell exits.
- Add IPython as a runtime dependency.
- This is an additive public CLI and Python API change; it does not remove or rename existing behavior.

## Affected DimOS Surfaces

- Modules/streams: Coordinator RPC lifecycle, RPC proxy metadata, and `Dimos` module/RPC discovery; no stream contracts change.
- Blueprints/CLI: All blueprints run through an active coordinator can be inspected through the new `dimos shell` command; blueprint composition and generated registries do not change.
- Skills/MCP: No MCP or skill-discovery behavior changes. Skills remain callable because `@skill` methods are RPCs.
- Hardware/simulation/replay: The shell works across hardware, simulation, and replay stacks. RPC calls execute immediately and may actuate hardware; the shell adds a startup notice but no per-call safeguard.
- Docs/generated registries: Update CLI and Python API usage docs. No blueprint registry regeneration is required.

## Capabilities

### New Capabilities

- `interactive-rpc-shell`: Attaching an interactive Python shell, discovering live module instances and RPCs, inspecting RPC metadata, and invoking RPCs against a running DimOS coordinator.

### Modified Capabilities

None.

## Impact

Developers gain a consistent interactive debugging surface without adding an MCP server or a shell module to each blueprint. Existing specialized manipulation clients remain unchanged while the general shell is evaluated. The main compatibility considerations are the new IPython runtime dependency and the lifecycle change that exposes coordinator RPC when `ModuleCoordinator.loop()` begins. Verification covers coordinator attachment, TTY-only CLI behavior, live and multi-instance discovery, RPC signature preservation, cleanup, and manual calls against a non-hardware blueprint.
