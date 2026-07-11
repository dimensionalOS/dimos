# DimOS Runtime Deployment Language

This context defines the language for discussing how DimOS modules retain stable identity while their implementations run in different environments or on different machines.

## Language

**In-Environment Python Module**:
A normal Python `Module` whose implementation is instantiated inside a local `PythonWorker` and shares that worker's Python environment.
_Avoid_: ExternalModule, remote normal Python module

**PythonWorker Preservation Boundary**:
The rule that deployment work does not replace the existing local `PythonWorker` path. In-environment Python modules continue using its lightweight object and RPC protocol.
_Avoid_: Universal external worker path, PythonWorker replacement

**ExternalModule**:
A declarative `Module` subclass whose implementation runs outside `PythonWorker`. It declares the DimOS-facing streams, config, RPC surface, and an implementation reference, but contains no build, process, watchdog, or transport behavior.
_Avoid_: NativeModule compatibility wrapper, process supervisor

**External Implementation Reference**:
The `ExternalModule` declaration that identifies what a runtime handle launches. It may be written as a string or `pathlib.Path`; the convention-discovered implementation layout, not the Python value type, determines whether it denotes a Python class or native executable.
_Avoid_: Deployment-level implementation override, value-type runtime discriminator

**NativeModule Compatibility Path**:
The existing `NativeModule` design in which a Python wrapper runs in `PythonWorker` and directly builds, launches, logs, and supervises a native subprocess. It remains available during migration to `ExternalModule`.
_Avoid_: Final external-module architecture, flag-day migration

**Convention Preset**:
A built-in preparation strategy selected automatically from a standard implementation layout. V1 heuristics recognize `python/pyproject.toml`, `rust/Cargo.toml`, and `cpp/CMakeLists.txt`; Deployment Specs override the inferred preset only for exceptional deployments.
_Avoid_: Hand-written prepare commands for every module, required manifest for standard layouts

**Unambiguous Convention Resolution**:
The rule that automatic preparation succeeds only when exactly one supported implementation layout is detected. No match or multiple matches fail planning unless the Module Deployment provides explicit Preparation and Runtime Environment behavior that selects and validates one implementation folder within the discovered package root.
_Avoid_: Convention precedence, silently selecting one of several implementations

**Deployment Spec**:
A first-class runnable deployment description that references a Blueprint, defines reusable named targets, and groups each module's execution target, build target, Preparation, and Runtime Environment choices in a Module Deployment. It does not redefine module identity or Blueprint wiring.
_Avoid_: Module package declaration, Blueprint replacement, deployment registry entry

**Execution Target**:
A named machine or execution substrate on which modules may be prepared and run. In v1, each target name identifies one distinct machine.
_Avoid_: Module implementation, Module Deployment

**Resolved Target Platform**:
The single detected and optionally asserted platform identity of an Execution Target. Worker bootstrap, Preparation, and artifact validation consume this identity rather than declaring independent destination platforms.
_Avoid_: Preparation-specific target platform, competing compiler-target declarations

**Deployment Root**:
The configured directory on an Execution Target containing DimOS-managed control environments, source snapshots, artifacts, runtime environments, run state, logs, caches, and locks. External system stores and provisioned devices may remain outside it.
_Avoid_: Generic source workspace, scattered DimOS state directories

**Hermetic Worker Bootstrap**:
The target bootstrap model in which DimOS transfers a pinned environment tool and provisions Python plus the version-matched ExternalWorker control environment inside the Deployment Root rather than relying on target-global installations.
_Avoid_: Module Runtime Environment, unmanaged system Python dependency

**Module Deployment**:
The grouped deployment policy for one Module: where it builds, where it executes, how deployable material is prepared, and how its runtime environment is materialized. Omitted choices use local placement or convention-derived behavior.
_Avoid_: Parallel assignment, build-location, and override maps

**Implicit Local Target**:
The execution target that exists in every Deployment Spec without explicit declaration. Its Deployment Root comes from GlobalConfig-backed DimOS state, and modules absent from the `modules` map run locally.
_Avoid_: Required local target declaration, implicit remote placement

**Deployment Plan**:
An immutable, validated resolution of Deployment Spec, Blueprint modules, implementation conventions, preparation behavior, concrete target definitions, Module Deployments, and worker routes.
_Avoid_: Behavioral reconciler, mutable deployment state

**Deployment Prepare Phase**:
The pre-worker phase that stages source and produces deployable material through target access, including native builds, cross-compilation, and artifact sync. It completes before the target-side ExternalWorker materializes module Runtime Environments.
_Avoid_: Module start, implicit build during run

**Preparation**:
A single deployment workflow that may coordinate build and execution targets and transfer artifacts between them. Convention Presets provide normal Preparation behavior; a Deployment Spec may substitute an exceptional Preparation.
_Avoid_: One preparation per machine, ExternalWorker lifecycle

**Runtime Environment**:
The execution-target environment materialized by ExternalWorker from staged deployment inputs before a runtime handle starts. It may install dependencies or artifacts and provision deployment-owned target resources.
_Avoid_: Build workflow, Runtime Host, ExternalWorker bootstrap environment

**Runtime Environment Spec**:
The serializable top-level import reference and JSON-compatible configuration from which ExternalWorker constructs a Runtime Environment. It is part of a Module Deployment only when convention-derived behavior needs an exceptional override.
_Avoid_: Live coordinator object, pickled class instance

**Shared Runtime Environment**:
A Runtime Environment reused within one deployment run when modules have the same source digest, execution-machine identity, and resolved Runtime Environment Spec. Setup is serialized and may rerun idempotently; teardown occurs only after all dependent runtime handles in that run stop.
_Avoid_: Planner-level workflow merging, per-module environment copies

**Deployment Source Snapshot**:
A content-addressed copy of a module package staged on a target before deployment. It includes the dependency-light contract and deployment extensions as well as implementation source, and is published atomically after transfer.
_Avoid_: Mutable shared source workspace, partial in-place rsync

**Idempotent Prepare**:
The rule that prepare executes required package-manager, build, and sync steps and relies on their caches or up-to-date checks rather than maintaining a separate DimOS freshness database.
_Avoid_: DimOS freshness cache, manual stale-state tracking

**Worker Manager**:
A coordinator-side backend scheduler that owns worker collections, placement, parallel deployment, rollback, health aggregation, and shutdown. DimOS uses one manager instance per deployment backend per coordinator.
_Avoid_: Worker process, per-machine manager

**Worker Manager Backend Boundary**:
The shared coordinator-facing contract implemented by WorkerManagerPython and WorkerManagerExternal. It gives ModuleCoordinator one deploy, undeploy, rollback, status, and Module Handle surface without requiring both backends to share a worker protocol.
_Avoid_: Shared worker implementation, PythonWorker subclass for external deployment

**Module Handle**:
The coordinator-visible proxy for one deployed module instance. A Python handle may wrap an Actor in PythonWorker; an external handle may route through ExternalWorkerClient to a runtime handle on a target machine.
_Avoid_: Python object instance, machine-specific worker process

**WorkerManagerPython**:
The manager for in-environment Python modules. It owns and schedules the local `PythonWorker` pool.
_Avoid_: External-module manager, target worker

**WorkerManagerExternal**:
The manager for `ExternalModule` deployments. It owns Module Deployments and Target Sessions, coordinates prepare and deployment, starts one `ExternalWorker` per execution machine, and aggregates rollback and health.
_Avoid_: Per-machine manager, separate deployment reconciler

**PythonWorker**:
A coordinator-side handle to one Python worker process that hosts one or more in-environment Python module instances.
_Avoid_: ExternalWorker, WorkerManagerPython

**Target Session**:
The coordinator-side access path to an Execution Target. A local session executes commands and transfers files directly; an SSH session executes commands, transfers artifacts, starts the ExternalWorker, and tunnels its control RPC.
_Avoid_: Module stream transport, Runtime Host control protocol

**ExternalWorker**:
The target-side process that owns runtime handles for all ExternalModules assigned to one execution machine for one run. It starts only after the Deployment Prepare Phase succeeds.
_Avoid_: Coordinator-side handle, one worker per module, preparation executor, persistent target agent

**ExternalWorker Client**:
The coordinator-side RPC handle to an ExternalWorker. For an SSH target, its ordinary RPC connection is carried through the Target Session's SSH tunnel.
_Avoid_: SSH command executor, Runtime Host, stream transport

**Runtime Host**:
The per-module runtime handle owned by an ExternalWorker. It receives a Module Launch Envelope, initializes control and stream bindings, supervises one ExternalModule implementation, and reports ready or failure. For native modules this may be only ExternalWorker state around a native subprocess; for packaged Python it may include a thin entrypoint process that imports the implementation inside the prepared Python environment.
_Avoid_: Worker manager, multi-module worker, required extra native wrapper process

**Python Runtime Entrypoint**:
The thin process entrypoint used by a packaged Python ExternalModule to enter its prepared Python environment, import the implementation class, and attach it to the Module Launch Envelope. It is not a second worker and should contain only bootstrap logic.
_Avoid_: PythonWorker replacement, module implementation, heavyweight runtime supervisor

**Ready Acknowledgement**:
The explicit runtime-handle signal sent after the launch envelope is parsed, the implementation is initialized, control is active, and stream bindings are ready.
_Avoid_: Treating process creation as successful module startup

**Module Launch Envelope**:
The unified serialized handoff to a runtime handle containing module identity, implementation launch metadata, module config, stream topics, transport descriptors, and control details. It extends the current `NativeModule.stdin_config` shape.
_Avoid_: Separate user-facing config and connection payloads

**Deployment Control Plane**:
The command and lifecycle path between ModuleCoordinator, WorkerManagerExternal, Target Sessions, ExternalWorker Clients, ExternalWorkers, and runtime handles. Target Sessions handle pre-worker bootstrap and preparation; ExternalWorker RPC handles deployed lifecycle, status, logs, health, and method calls. SSH may carry that RPC through a tunnel but never carries module stream data.
_Avoid_: Stream data transport

**Deployment Data Plane**:
The transport path used by module streams, such as Zenoh, DDS, ROS, LCM, or SHM where applicable.
_Avoid_: Worker control protocol, lifecycle channel

**Fail-Fast Startup Rollback**:
The rule that startup stops already-started workers and runtime handles if any module fails before the deployment reaches ready state.
_Avoid_: Partial startup, orphaned Runtime Host

**ExternalWorker Lease**:
A coordinator heartbeat or lease that causes an ExternalWorker to stop its runtime handles if the coordinator disappears.
_Avoid_: Orphaned target processes, required persistent agent
