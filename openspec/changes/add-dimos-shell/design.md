## Context

`dimos run` already starts a singleton coordinator RPC service, and `Dimos.connect()` already builds module RPC proxies through `RemoteModuleSource`. The Python interface can call module RPCs and skills, load modules, restart modules, and peek streams, but it has no first-class interactive CLI or public structured discovery methods. Direct `ModuleCoordinator.build(...).loop()` launches do not start the coordinator RPC service, and `Dimos.connect()` currently requires a local run-registry entry before it probes the service.

RPC proxies copy method names and docstrings when the module class is importable, but they expose the generic `(*args, **kwargs)` callable signature. Coordinator descriptors already carry exact instance RPC names, class identity, qualified import paths, and advertised RPC names. Blueprint namespaces and explicit instance names allow multiple deployed instances of one module class.

The accepted lifecycle decision is recorded in `docs/adr/0001-coordinator-loops-are-attachable.md`. The terms DimOS shell, RPC, skill, and module instance are recorded in `CONTEXT.md`.

## Goals / Non-Goals

**Goals:**

- Add a zero-argument `dimos shell` command that attaches IPython to the coordinator on the configured bus.
- Make both CLI-launched and direct Python-launched coordinator loops attachable.
- Add live, structured module and RPC discovery to the public `Dimos` interface.
- Preserve Python signatures and documentation on importable RPC proxies.
- Support exact instance identity and existing unique-class convenience lookup.
- Keep shell startup, safety messaging, connection cleanup, and error behavior predictable.

**Non-Goals:**

- Add an injectable REPL module, MCP requirement, skill-specific helper, or stream discovery.
- Filter or authorize RPC calls.
- Add automatic reconnection, run selection, command-specific options, or non-TTY execution.
- Migrate or redesign the existing manipulation clients.
- Publish a reusable shell-launcher interface or automatic domain extension mechanism.
- Make coordinator-supplied signature metadata authoritative for clients with different installations.

## DimOS Architecture

`dimos shell` is a Typer command in the existing robot CLI. It connects through `Dimos.connect()` and launches IPython with a private namespace builder. The public namespace binds:

- `app` to the connected `Dimos` instance.
- `modules` to `app.list_modules`.
- `rpcs` to `app.list_rpcs`.
- `describe` to `app.describe`.

The `Dimos` interface delegates exact and unique-class module lookup to its `ModuleSource`. Discovery refreshes coordinator descriptors on each call, imports the advertised module class when possible, and extracts metadata from its current RPC methods. It reuses and additively extends the existing introspection `ModuleInfo`, `RpcInfo`, and `ParamInfo` records so class-level visualization callers remain compatible. Runtime records carry exact module-instance identity, qualified class identity, full documentation, and concise representations. Stream fields are not populated or exposed as part of this capability.

`RpcCall` preserves callable metadata from the original class method. It uses wrapper metadata plus an explicit signature with the leading instance parameter removed. Names-only `_RemoteProxy` calls remain callable but cannot manufacture missing type or documentation metadata.

`ModuleCoordinator.loop()` starts the coordinator RPC service idempotently before blocking. The CLI retains its explicit post-daemonization start, so the loop call is a no-op for CLI runs and does not create transport resources before the daemon fork. `Dimos.connect()` removes the registry precondition and relies on the existing coordinator probe and timeout. Run-registry metadata becomes optional banner context.

No streams, blueprint composition, DimOS `Spec` Protocols, adapter Protocols, skills/MCP exposure, or generated blueprint registries change. `@skill` methods appear in RPC discovery because they are RPCs, but the shell adds no skill-specific discovery surface.

## Decisions

1. **Use IPython as a required dependency.** Completion, history, multiline editing, inspection, and exception presentation are core to the command. A standard-library fallback would give the same command materially different behavior across installations.
2. **Use the existing `Dimos` façade.** The shell creates no dedicated module and no parallel RPC client abstraction. Public discovery methods also serve notebooks and scripts.
3. **Keep helpers thin.** Discovery behavior lives on `Dimos`; the shell only binds short aliases. `describe` returns the same structured records as list discovery rather than a separate formatted result.
4. **Use local class introspection with names-only fallback.** The supported v1 flow normally shares the daemon's installation. Extending the coordinator wire contract for authoritative signatures is unnecessary for this scope.
5. **Return all advertised RPCs.** Discovery does not hide lifecycle or inherited framework methods. The shell is a trusted Python environment, and users own the consequences of direct invocation.
6. **Make discovery live.** Each call refreshes deployed descriptors so dynamically loaded modules appear without a refresh command. There is no automatic reconnection after coordinator loss.
7. **Keep the launcher private.** Existing manipulation clients remain unchanged while the general shell is evaluated. A public reusable launcher would be a hypothetical seam with one caller.

## Safety / Simulation / Replay

The shell behaves the same for hardware, simulation, and replay blueprints. It can invoke any advertised RPC immediately, including methods that move hardware or alter lifecycle state. The startup banner states this clearly. The design adds no per-call confirmation, filtering, or authorization because the command is an explicitly trusted debugging environment with arbitrary Python execution.

Manual QA should use a non-hardware test or replay blueprint first. Hardware QA, if performed, must follow the target robot's existing bring-up and physical safety procedures; no new motion behavior is introduced by this change.

## Risks / Trade-offs

- Starting coordinator RPC from `loop()` broadens the observable lifecycle contract. Idempotent startup and post-daemonization CLI ordering prevent duplicate services and pre-fork resource creation.
- IPython increases the base runtime dependency set. The consistent interactive experience is worth the dependency for a first-class shell.
- Local introspection can lose signatures when a module class cannot be imported. The shell returns names-only metadata and does not claim information it lacks.
- Live discovery adds a coordinator round trip per call. Interactive latency is small, and correctness after dynamic module loading is more valuable than caching.
- Exposing all RPCs creates hardware and lifecycle risk. A persistent startup notice makes the trust model explicit; access control is out of scope for an arbitrary Python shell.
- Extending existing introspection records could affect renderers. New fields must be optional or defaulted, and existing rendering tests must remain green.

## Migration / Rollout

The CLI command and Python discovery methods are additive. Existing `Dimos` access, `app.skills`, MCP commands, manipulation clients, and blueprint behavior remain available. Documentation will present `dimos shell` as an experimental general debugging surface without deprecating specialized clients.

Add IPython to normal project dependencies and update the lockfile. No module or blueprint names change, so `dimos/robot/all_blueprints.py` does not require regeneration. Rollback removes the command and discovery additions; reverting coordinator-loop attachment restores the prior direct-Python behavior.

## Open Questions

None. The remaining choices, such as exact private file placement and representation formatting, are implementation details constrained by the public behavior above.
