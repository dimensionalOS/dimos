# DimOS Runtime Deployment Language

This context defines the language for discussing how DimOS modules keep a stable module identity while their implementations run in different environments and on different machines.

## Language

**Module Contract**:
A DimOS-facing module identity that declares the streams, RPCs, config shape, and lifecycle surface other modules depend on.
_Avoid_: Stub module, fake module

**In-Environment Python Module**:
A normal Python Module that runs inside the current DimOS Python worker environment and keeps today's local, in-process-package dependency assumptions.
_Avoid_: Packaged runtime module, remote module

**PythonWorker Preservation Boundary**:
The rule that the deployment model does not replace the existing local PythonWorker path. In-environment Python modules keep using PythonWorker because its lightweight object and RPC envelope works well for normal local modules.
_Avoid_: PythonWorker replacement, universal Deployment Worker path

**Packaged Runtime Module**:
A module implementation launched through an external prepared runtime, such as a native executable or a packaged Python runtime project, while DimOS keeps a stable wrapper or contract for graph integration. This can remain under the existing NativeModule naming for now because both cases spawn an external process.
_Avoid_: Normal Python module, abstract Blueprint

**Packaged Python Runtime Backend**:
A Packaged Runtime Module backend that launches a prepared Python runtime process through Deployment Worker plus Runtime Host for both local and remote assignments, not through the normal in-environment PythonWorker. The Deployment Worker should spawn and supervise this process without importing its implementation.
_Avoid_: Native binary backend, in-environment Python module, normal PythonWorker

**Packaged Python Entrypoint**:
A DimOS-provided entrypoint installed or importable inside a packaged Python runtime project. The Deployment Worker launches it, passes a Module Launch Envelope, and the entrypoint imports and runs the packaged Python implementation.
_Avoid_: Arbitrary Python script, Deployment Worker import

**Runtime Backend**:
The mechanism that runs a module implementation after deployment prepares its requirements. Normal PythonWorker runs only in-environment Python modules; packaged Python modules and native modules use separate packaged runtime backends.
_Avoid_: Deployment package, execution target

**Deployment Worker**:
A minimal worker process launched on an execution target, local or remote, that connects to the coordinator and spawns packaged module processes on that target without importing their implementations. V1 uses one Deployment Worker per target, with one Runtime Host per packaged/native module.
_Avoid_: Remote-only worker, module process, target profile

**Runtime Host**:
A target-local process that hosts exactly one module implementation in the v1 deployment model. It parses the Module Launch Envelope, brings up the module, and handles module-level operations such as lifecycle and method calls.
_Avoid_: Deployment Worker, multi-module worker pool

**Module-Kind Worker Routing**:
The rule that worker choice depends on the deployed module kind, not on a top-level local-vs-deployment mode. Normal Python modules use PythonWorker; packaged Python and native modules use Deployment Worker plus Runtime Host.
_Avoid_: Blueprint-mode worker split, deployment-mode-only worker split

**NativeModule Target Path**:
The target architecture where NativeModule implementations are spawned and supervised through Deployment Worker plus Runtime Host for both local and remote assignments, rather than being hosted as PythonWorker-managed wrapper modules.
_Avoid_: Permanent PythonWorker native wrapper

**Side-by-Side Native Migration**:
The rule that NativeModule migration to Deployment Worker plus Runtime Host should support old and new native paths side-by-side temporarily. Migrate representative modules first, keep legacy NativeModule wrapper path for unmigrated modules, and remove the old path in a later cleanup.
_Avoid_: Native flag-day migration, breaking all native modules in one PR

**NativeModule Compatibility Stage**:
The implementation stage where native modules gain deployment prepare and automatic build support through the newly defined module contract and deployment config API, without changing existing NativeModule files. V1 can add a sidecar Python file in the native module directory that declares the new API for that package.
_Avoid_: Legacy NativeModuleConfig-driven deployment, immediate mutation of existing NativeModule wrappers

**Deployment Spec Required for Native Prepare**:
The rule that native automatic build/prepare through the new deployment API uses a Deployment Spec, even for local execution. Local target may be implicit, but native prepare should not be introduced as a one-off helper around existing NativeModule.
_Avoid_: Standalone native build helper, Deployment Spec bypass for local native

**Native Prepare Before Runtime Migration**:
The PR 3 rule that native modules may use the new Deployment Spec and prepare/build pipeline while still launching through the existing NativeModule wrapper path. Moving native runtime launch to Deployment Worker plus Runtime Host is PR 4.
_Avoid_: Combining native prepare and native runtime migration in one PR, claiming full native unification before launch path changes

**Representative Native Migration Targets**:
Use `MLSPlannerNative` for native automatic build/prepare because it proves the existing wrapper plus `rust/Cargo.toml` convention. Use a tiny native example such as rust ping-pong or native echo for the first Runtime Host migration, then migrate `MLSPlannerNative` after the path is stable.
_Avoid_: Proving native migration only on heavy production module, proving native prepare only on toy example

**Native Deployment Sidecar**:
The v1 convention that native modules using the new deployment API may add a `deployment.py` sidecar in the module package directory. The sidecar declares the native package and local deployment shape without mutating existing NativeModule wrapper files.
_Avoid_: Package-specific sidecar filenames, editing legacy wrapper just to prove deployment prepare

**Module Launch Envelope**:
The serialized handoff from DimOS to an external runtime process, carrying resolved module identity, config, stream topics, transport descriptors, implementation identity, and optional control-plane connection details.
_Avoid_: Module contract, runtime requirement

**Native-Style Unified Runtime Handoff**:
The v1 rule that Runtime Hosts receive one launch envelope that keeps launch details, module config values, and resolved connection bindings together, following the current `NativeModule.stdin_config` shape. DimOS may track where those fields originate internally, but users should not need to reason about separate handoff objects.
_Avoid_: Over-split runtime handoff, separate user-facing connection-binding object

**Deployment Implementation Sequence**:
The preferred PR split: define specs and skeletons; support local packaged Python; support native automatic build through the new API using sidecar native package declarations; migrate existing NativeModule wrappers to the new API; add SSH remote deployment for packaged Python; add SSH remote deployment for native modules.
_Avoid_: Remote-first implementation, native breaking migration before compatibility path

**Separate Remote PRs**:
The rule that SSH remote deployment for packaged Python and SSH remote deployment for native modules should be separate implementation PRs because their sync, prepare, launch, and failure modes differ.
_Avoid_: One combined remote deployment PR, hiding native remote complexity behind packaged Python remote work

**Internal Skeleton First**:
The first implementation PR should define internal specs, skeletons, interfaces, and tests for the deployment shape without adding user-facing CLI commands or public APIs. This avoids confusing users while the model is still being staged into main.
_Avoid_: Premature CLI surface, public API before behavior exists

**Coordinator Wiring Starts in PR 2**:
The first skeleton PR should keep deployment code isolated under a new deployment package and avoid touching ModuleCoordinator behavior. The second PR wires the deployment layer into local packaged Python behavior.
_Avoid_: ModuleCoordinator integration in skeleton-only PR, behavior change before feature PR

**First Public Packaged Python Stage**:
The second implementation PR may expose the first narrow user-facing API because it delivers a real local packaged Python feature. This API should be class-local and avoid remote deployment or broad deployment CLI until later stages.
_Avoid_: Test-only feature after behavior is ready, remote API in packaged-Python-local stage

**Packaged Python Prepare and Launch Stage**:
The PR 2 rule that local packaged Python must include both runtime preparation and local Python Runtime Host launch. Prepare-only packaged Python is not useful because there is no existing packaged-Python launch path equivalent to NativeModule.
_Avoid_: Prepare-only packaged Python feature, packaged Python without runtime host

**Deployment Spec Required for Packaged Python**:
The rule that packaged Python modules use a Deployment Spec even for local execution, rather than Blueprint-only runtime placement fields. This exercises the deployment planner and avoids adding another Blueprint-local runtime API.
_Avoid_: Blueprint-only packaged Python placement, repeating exploratory runtime-placement API

**Packaged Python Run Entrypoint Question**:
Open question for PR 2: how should developers run a local packaged-Python Deployment Spec before the full deployment CLI/registry exists? Options include a narrow Python API, a temporary example runner, or an early CLI surface.
_Avoid_: Deciding CLI registry before UX is ready, feature with no runnable path

**Unified Packaged Runtime Path**:
The rule that packaged Python and native modules should use the same Deployment Worker plus Runtime Host architecture. The current runtime-specific PythonWorker pool approach is useful exploration but should not merge as the long-term public packaged-Python API.
_Avoid_: Separate packaged-Python worker architecture, merging exploratory runtime pools as final API

**Exploration PR as Reference**:
The current packaged-Python runtime PR should remain open as a draft/reference while a fresh implementation branch carries the clean internal skeleton PR. The exploration PR preserves discussion and proof-of-need but should not become the public API path.
_Avoid_: Retrofitting exploratory branch into clean skeleton PR, merging reference implementation as final architecture

**Runtime Requirement**:
The stable environment or artifact declaration needed by a module implementation, such as a Python project, Pixi project, Nix flake, native project, binary target, or executable path.
_Avoid_: Machine assignment, deployment target

**Module Runtime Declaration**:
A module-owned declaration of the Runtime Requirement and launch recipe for that module implementation. It belongs with the module package, not in Blueprint wiring or Deployment Assignment.
_Avoid_: Blueprint runtime config, execution assignment

**Self-Contained Module Package**:
A deployable module unit that carries its Module Contract, implementation, runtime and build requirements, preparation recipes, and launch recipe.
_Avoid_: Execution target, machine profile

**Module Package Convention**:
A project layout convention for Self-Contained Module Packages that uses existing Python, native, Pixi, Nix, Cargo, and blueprint files before introducing a dedicated manifest.
_Avoid_: Required manifest, registry format

**Convention Preset**:
A built-in DimOS recognizer that turns a common module package layout into default prepare, sync, and launch behavior. Examples include `uv` runtime projects, Pixi-backed Python runtime projects, Cargo native packages, CMake native packages, and Nix-backed native packages.
_Avoid_: Hand-written prepare commands for every module, mandatory manifest for common layouts

**Preset Inference with Explicit Override**:
The rule that DimOS infers a Convention Preset when exactly one preset matches a module package layout. If zero presets match or multiple presets match, deployment planning requires an explicit package or preset override.
_Avoid_: Spooky multi-preset selection, required preset declaration for common layouts

**Class-Local Package Override**:
The v1 rule that module-local package overrides live on the Module Contract or NativeModule wrapper class, reusing the current NativeModule pattern where executable, cwd, build command, and stdin config are declared near the wrapper.
_Avoid_: Required sibling deployment file, immediate manifest requirement

**Deployment Sidecar Convention**:
The updated v1 convention that packaged Python and native modules using the new deployment API declare package and deployment metadata in a generic `deployment.py` sidecar next to the Module Contract or NativeModule wrapper. This keeps contract/wrapper files focused on DimOS-facing module surface and gives both module kinds one package-level API convention.
_Avoid_: Mutating contracts only to add deployment metadata, package-specific sidecar filenames

**Package vs Deployment Override Boundary**:
The rule that intrinsic prepare and launch recipes belong to the module package, while Deployment Spec chooses target assignment, prepare location, sync location, workspace, secrets, and resources. Different build commands, executables, implementation paths, or runtime roots are module-internal settings and should not be altered outside module config.
_Avoid_: Deployment-specific hidden implementation changes, mutable launch recipe in assignment, deployment-level package override

**Module Package Reference**:
The local coordinator-side reference to a Self-Contained Module Package, including enough information to find its source root, runtime declaration, launch recipe, and files or artifacts to sync.
_Avoid_: Execution target profile, deployment assignment

**Assignment-First Package Discovery**:
The default v1 rule that a Deployment Spec assigns Module Contracts to targets, and DimOS discovers the corresponding Self-Contained Module Package from each contract or wrapper anchor. Package settings remain module-internal and are not altered by Deployment Spec.
_Avoid_: Required package map, package-first deployment spec, deployment-level package override

**Preparation Strategy**:
The deployment-owned choice of where and how to realize a Runtime Requirement, such as preparing on the execution machine, preparing on the coordinator host then syncing, or cross-compiling elsewhere then syncing artifacts.
_Avoid_: Runtime requirement, build requirement

**Execution Assignment**:
The deployment-owned choice of where the module process runs.
_Avoid_: Runtime requirement, environment declaration

**Execution Target Profile**:
The concrete description of a machine or execution substrate that a deployment can use to prepare, sync, launch, and connect module processes.
_Avoid_: Runtime requirement, module contract

**Deployment Assignment**:
The deployment-owned binding that chooses which Self-Contained Module Package runs on which Execution Target Profile.
_Avoid_: Runtime requirement, module package

**Class-Keyed Deployment Assignment**:
The v1 Python API rule that Deployment Assignment keys are Module Contract classes, while assignment values are target string names. String module names are deferred for future YAML/TOML deployment definitions.
_Avoid_: String module keys in Python v1, mixed module reference styles

**Partial Deployment Assignment**:
The rule that a Deployment Spec may assign only some modules to explicit targets; unassigned modules remain on the local-default path. Deployment plans should show both assigned and default-local modules.
_Avoid_: All-or-nothing deployment assignment, implicit remote placement

**Target Profile / Assignment Separation**:
The rule that target profiles describe available machines or substrates, while deployment assignments bind modules to those targets for a specific run.
_Avoid_: Deployment profile containing everything, machine profile with module placement

**Deployment Target Map**:
The rule that a Deployment Spec contains or references the Execution Target Profiles it uses, while keeping target definitions separate from Deployment Assignments. Planning must resolve a complete target map before any prepare or run action.
_Avoid_: Assignment-only target strings with no resolved profile, mixing machine definition into module package

**String Target Assignment**:
The v1 rule that Deployment Assignments refer to targets by string name only. Target objects live in the Deployment Target Map, and assignments use those names for simplicity and future YAML/TOML compatibility.
_Avoid_: Target-object assignment values, mixed assignment reference styles

**Deployment Reconciler**:
The component that turns a Self-Contained Module Package, Execution Target Profile, and Deployment Assignment into concrete prepare, sync, launch, and connection actions.
_Avoid_: Module package, execution target

**Deployment Prepare Phase**:
The deployment phase that realizes runtime requirements before launch, such as installing Python environments, building native artifacts, cross-compiling, or syncing prepared outputs to execution targets.
_Avoid_: Run phase, module start

**Idempotent Prepare**:
The rule that deployment preparation should run required package-manager, build, and sync steps each time and rely on those tools' own caches or up-to-date checks rather than a separate DimOS deployment-state cache.
_Avoid_: DimOS freshness database, manual stale-state tracking

**Fail-Fast Startup Rollback**:
The rule that deployment run should stop already-started workers and runtime hosts if any module fails before the deployment reaches the running state.
_Avoid_: Partial startup, orphaned runtime host

**Deployment Run Lifecycle**:
The rule that deployment runs participate in the same DimOS lifecycle commands as local runs, including status, stop, restart, and logs, with the coordinator propagating lifecycle operations to deployment workers and runtime hosts.
_Avoid_: Separate deployment-only lifecycle CLI

**Deployment Worker Lease**:
A heartbeat or lease from the coordinator to a Deployment Worker that causes the worker to stop its Runtime Hosts if the coordinator disappears.
_Avoid_: Remote orphaned runtime host, indefinite detached worker

**Ephemeral Deployment Worker**:
The v1 rule that a Deployment Worker is started for a specific deployment run and exits when that run stops. A persistent target agent can implement the same control contract later.
_Avoid_: Required target daemon, persistent agent v1

**Deployment Control Plane**:
The command and lifecycle communication path between coordinator, Deployment Workers, and Runtime Hosts, used for spawn, stop, status, logs, health, and method calls.
_Avoid_: Stream data transport, sensor data plane

**Deployment Data Plane**:
The stream transport path used by module inputs and outputs, such as Zenoh, DDS, ROS, LCM, or SHM where applicable.
_Avoid_: Worker control protocol, lifecycle channel

**Deferred Data-Plane Guard**:
The v1 choice to report cross-target stream transport assumptions in deployment plans without enforcing full data-plane compatibility checks; stricter guards can be added later.
_Avoid_: v1 transport verifier, mandatory data-plane preflight

**Deployment Spec**:
A deployment-owned operational description of which module packages run on which execution targets, how their artifacts are prepared or synced, and what cross-machine connections they require. It does not define the abstract module graph or stream wiring.
_Avoid_: Blueprint, connection profile, module package

**Local-Default Deployment**:
The rule that a Blueprint runs with today's fully local worker spawning behavior unless a Deployment Spec explicitly assigns modules to non-local execution targets.
_Avoid_: Implicit remote deployment, auto-distributed run

**Implicit Local Target**:
The rule that every Deployment Spec has an implicit local execution target. Modules without explicit Deployment Assignments run on the local target by default, and deployment plans should show that local placement explicitly.
_Avoid_: Required local target declaration, hidden non-local assignment

**Explicit Remote Package Rule**:
The rule that a module assigned to a non-local execution target must be a Packaged Runtime Module, such as a native module or packaged Python runtime module. Normal in-environment Python modules are local-only.
_Avoid_: Remote normal Python module, implicit remote Python environment

## Open Language Questions

**Grounded Blueprint vs Deployment Spec Boundary**:
Should DimOS expose a named **Grounded Blueprint** object, or should **Deployment Spec** stay as the concrete operational layer beside an abstract Blueprint?
_Current leaning_: Deployment Spec should stay free of abstract Blueprint connection/profile concerns and describe deployment mechanics only.

**Module Package Convention Shape**:
What filesystem convention should DimOS use to discover Self-Contained Module Packages before a dedicated manifest exists?
_Current leaning_: convention-first discovery with explicit package overrides when a Module Contract has multiple implementations or an unusual layout. Existing native precedent includes a contract or wrapper module beside a runtime source directory, such as `mls_planner_native.py` plus `rust/Cargo.toml`. Packaged Python should mirror that shape with a contract or wrapper module beside a `runtime/` Python project.

**Module Package Anchor**:
The Module Contract or NativeModule wrapper file is the v1 anchor for discovering a Self-Contained Module Package. Discovery starts at that file and looks for sibling runtime roots such as `rust/`, `cpp/`, `runtime/`, `pixi.toml`, or `flake.nix`.
_Avoid_: Required package marker file, manifest-first discovery
