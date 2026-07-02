# Runtime Environments

Runtime Environments let a blueprint place selected Python modules into a
separate interpreter or Python project without making the coordinator import the
worker-only dependency stack.

## Runtime Environment Registration

Register runtimes on a blueprint with `runtime_environments(...)`. Every
blueprint already has the `current` runtime for the coordinator process. A Python
Runtime Project is registered with `PythonProjectRuntimeEnvironment(name,
project)`, where `project` points at a directory containing `pyproject.toml` and
a committed `uv.lock`.

## Runtime Placement

Use `runtime_placements(...)` to map a Module Contract class to a
`RuntimePlacement(runtime, implementation)`. The `runtime` names the registered
environment. The `implementation` is an import path resolved inside that runtime
worker and must be a subclass of the Module Contract.

See `examples/dimos-demo-worker-module` for a minimal contract/implementation
split.

## Module Contract vs Runtime Implementation

The Module Contract is the class the coordinator imports. Keep it small and
coordinator-import-safe: declare RPC methods, streams, config, and types needed
for blueprint wiring, but do not import runtime-only packages. The Runtime
Implementation lives in the Runtime Project package, subclasses the contract,
and imports the dependencies needed to do the work.

This is different from a DimOS `Spec`. A `Spec` is a structural interface used
for module-reference injection: one module declares the methods it needs from
another module. A Module Contract is a concrete, deployable `Module` identity the
coordinator uses for blueprint graph construction, actor lookup, RPC topic
identity, streams, and lifecycle. Runtime Implementations subclass that contract
for this change; future descriptor work may loosen that requirement.

## Runtime Project and Locked Runtime Project

A Runtime Project is a Python project directory used by one or more worker
modules. A Locked Runtime Project has committed lockfiles so deployment can
reproduce the environment without rewriting dependency resolution state.

- uv projects require `pyproject.toml` and `uv.lock`.
- Pixi-backed uv projects require `pyproject.toml`, `uv.lock`, `pixi.toml`, and
  `pixi.lock`.

Lockfiles are source inputs. Generate or update them manually with the package
manager (`uv lock`, `pixi lock`, or a future DimOS build/update command), then
commit them.

## Runtime Reconciliation during deployment

Before worker launch, DimOS selects the active Runtime Project placements and
reconciles their prepared environments in locked mode. For uv projects it runs
`uv venv --seed` and `uv sync --locked` in the Runtime Project. For Pixi-backed
projects it runs `pixi install --locked`, creates a uv venv from the Pixi
Python, and runs `pixi run uv sync --locked`.

Reconciliation prepares `.venv` state but does not mutate lockfiles. If a lock is
missing or stale, deployment fails and asks for an explicit package-manager lock
or update step.

## Python Runtime Worker semantics

Python runtime workers still run DimOS modules with normal module lifecycle,
RPC, streams, and worker isolation. The coordinator sends the Module Contract and
optional Runtime Implementation path to the worker. Inside the worker, DimOS
imports the implementation path and verifies that it subclasses the contract
before instantiating it.

## Failure modes

- Unknown runtime name: placement references a runtime that was not registered.
- Duplicate Runtime Project path: two registered runtimes point at the same
  project directory.
- Missing project files: `pyproject.toml`, `uv.lock`, or `pixi.lock` is absent.
- Reconciliation command missing or failed: `uv`/`pixi` is unavailable, locked
  sync fails, or the lock does not match the project.
- Missing prepared Python: `.venv/bin/python` was not created by reconciliation.
- Invalid implementation path: import path is malformed or cannot be imported in
  the worker runtime.
- Implementation type error: the imported class is not a subclass of the Module
  Contract.

## Non-goals for this change

Runtime descriptors, remote Runtime Projects, automatic image/build systems, and
lockfile/build-update orchestration are future work. The current behavior is
local registration, explicit placement, locked reconciliation, and Python worker
launch.
