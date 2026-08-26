# Distributed Host Architecture

Language: English | [简体中文](/experimental/docs/hosted/architecture_CN.md)

Status: Draft proposal

Scope: Discovery and constraint-based placement MVP

## Summary

Today, a DimOS `Blueprint` is built by one `ModuleCoordinator`, and all of its
modules are deployed on the same machine. Some transports, especially Zenoh,
can already carry streams between machines, but the blueprint runtime has no
concept of module placement or coordinated multi-machine lifecycle.

This proposal adds a thin distributed orchestration layer:

- Each execution device runs a small, persistent **Host service**.
- A Host joins a Zenoh fabric and advertises its generated identity, discoverable
  name, tags, capabilities, resources, compatibility, and current availability.
- A blueprint may add exact-Host or tag constraints to a module or composed
  fragment. Modules with no placement constraint continue to run locally by
  default.
- `dimos run` joins the same Zenoh fabric, discovers the available Hosts,
  resolves placement, partitions the blueprint, and asks the selected Hosts to
  start their local fragments.
- The user joins the Zenoh fabric through a router connection or LAN discovery.
- Each Host continues to use the existing `ModuleCoordinator` and worker
  implementation for local deployment.
- Streams crossing Host boundaries communicate directly over Zenoh. The
  controller and Host services do not proxy application data.

The MVP includes discovery and simple constraint-based placement. It deliberately
does not include code distribution, transparent cross-Host module references,
failover, migration, replicas, or sophisticated resource optimization.

## Motivation

A blueprint describes a logical application, but today it also implicitly
defines one deployment unit: one coordinator on one machine. This makes it
difficult to describe applications such as:

- robot drivers and control modules on an onboard computer;
- perception modules on a GPU server;
- an agent, navigation stack, or UI on a developer workstation;
- shared services running on another machine;
- one application selecting an available robot by embodiment, such as a G1;
- a fleet connected through one Zenoh router, with several independent Hosts
  advertising robots and compute resources behind it.

Zenoh already gives these modules a cross-machine data path and a shared
discovery fabric. The missing piece is a control plane that can discover
execution capacity, bind placement constraints, and start, observe, and stop one
logical application across the selected machines.

## Goals

The first version should:

1. Let each Host advertise a descriptor that the controller discovers through
   Zenoh.
2. Let a blueprint constrain a module or fragment by a unique Host name/ID or by
   a set of Host tags.
3. Fail before deployment with a useful diagnostic when no live compatible Host
   satisfies a placement constraint.
4. Start the full application with one `dimos run` command after the controller
   has joined the relevant Zenoh fabric.
5. Reuse the current blueprint compiler, `ModuleCoordinator`, workers, module
   lifecycle, and stream implementations on each machine.
6. Use Zenoh for streams that cross machine boundaries.
7. Validate unsupported distributed graphs before starting modules.
8. Give the user one run ID, an immutable placement record, and an aggregated
   view of Host and module status.
9. Preserve the behavior and API of existing single-machine blueprints.
10. Establish a foundation for discovering robot embodiments and other typed
    hardware capabilities without making fleet replication part of the MVP.

## Non-goals for the MVP

The following are intentionally out of scope:

- optimal bin-packing, cost-aware scheduling, preemption, or autoscaling;
- requesting multiple replicas or every Host that matches a selector;
- sending source code, environments, models, or assets to a Host;
- running different revisions of the application on different Hosts;
- cross-Host module references or transparent distributed object calls;
- LCM as a distributed transport;
- automatic failover, rescheduling, or module migration;
- untrusted or public-internet deployments;
- WebTransport/WebRTC connectivity or NAT traversal;
- changing placement because Hosts join or leave after an application starts;
- automatically expanding one blueprint fragment across a fleet of robots.

These can be added later without changing the basic separation between Host
discovery, the Host control plane, and the Zenoh application data plane.

## Current architecture

The current local startup path is approximately:

1. `dimos run` loads a `Blueprint`.
2. The blueprint resolves modules, configuration, stream connections, and
   module references.
3. One `ModuleCoordinator` deploys the modules through local worker managers.
4. The coordinator wires streams and module RPC proxies, then builds and starts
   the modules.
5. One local run registry entry owns the process and its logs.

Important existing components are:

- [`core/coordination/blueprints.py`](/dimos/core/coordination/blueprints.py),
  which represents and composes the logical module graph;
- [`core/coordination/module_coordinator.py`](/dimos/core/coordination/module_coordinator.py),
  which performs local deployment, wiring, and lifecycle management;
- [`core/coordination/worker_manager_python.py`](/dimos/core/coordination/worker_manager_python.py),
  which deploys Python modules into local worker processes;
- [`core/coordination/coordinator_rpc.py`](/dimos/core/coordination/coordinator_rpc.py),
  which exposes a running coordinator over the selected RPC backend;
- [`porcelain/remote_module_source.py`](/dimos/porcelain/remote_module_source.py),
  which already demonstrates controlling a separately running coordinator;
- [`core/transport_factory.py`](/dimos/core/transport_factory.py), which selects
  LCM or Zenoh transports and RPC backends;
- [`protocol/service/zenohservice.py`](/dimos/protocol/service/zenohservice.py),
  which already supports explicit Zenoh connect endpoints and multicast
  scouting.

This design keeps the local runtime intact. A Host is not a new module
runtime; it is a discoverable supervisor that creates and controls an existing
local runtime.

## Terminology

| Term | Meaning |
| --- | --- |
| Application | One logical blueprint graph that may span several Hosts. |
| Controller | The `dimos run` process that compiles, places, and coordinates an application. |
| Host | One advertised execution service. Usually one per machine, although several may sit behind one router or even share a machine. |
| Host ID | A stable, opaque, automatically generated identity used in protocol keys and run records. |
| Host name | A discoverable, human-readable label, defaulting to the machine hostname and optionally overridden. It is not a network address. |
| Host tags | User- or provider-supplied labels such as `gpu`, `g1`, or `warehouse-a` used as placement constraints. |
| Host descriptor | The Host's identity, tags, capabilities, resources, compatibility, load, and availability. |
| Placement constraint | An exact Host selector and/or required tag set attached to blueprint metadata. |
| Placement unit | A set of modules that must be deployed together on one Host. |
| Host fragment | The modules and local connections assigned to one Host for one run. |
| Local coordinator | The existing `ModuleCoordinator` used to run one Host fragment. |
| Boundary stream | A stream whose publisher and subscriber are placed on different Hosts. |
| Module reference | A typed `Spec`/module dependency wired as an RPC proxy by the coordinator. |

Network location and execution identity are separate. A locator such as
`tcp/some_host:7447` determines how the controller joins a Zenoh fabric.
Placement resolves against the Host IDs, names, tags, and capabilities
advertised on that fabric.

## Proposed architecture

The system has three related but separate concerns.

### Discovery plane

Hosts advertise live execution capacity on Zenoh. The controller first joins a
fabric, observes live Host identities, and queries their descriptors. A Zenoh
router may have one Host beside it, five Hosts behind it, five robots connected
to it, or no Host at all; routing topology does not define placement topology.

Discovery answers “what can run this application?” It does not start anything
and does not make a Host trusted merely because it can advertise.

### Control plane

After resolving placement, the controller talks to each selected Host service
to:

- verify identity, compatibility, availability, and the advertised descriptor;
- reserve the placement for a bounded prepare lease;
- submit a Host fragment and resolved configuration;
- prepare, start, stop, and inspect a local deployment;
- report startup failures and module status.

Control traffic is low-volume. The initial implementation should reuse the
existing RPC abstraction and Zenoh backend where practical. RPC identities are
scoped by Host ID and run ID so several coordinators can share one Zenoh fabric.

### Data plane

Modules continue to publish and subscribe through DimOS transports. A boundary
stream uses Zenoh directly between the publisher's Host and the subscriber's
Host. Neither the controller nor the Host service receives, relays, or
transforms application messages.

This means the application can continue moving data if the controller exits
after startup, although the MVP may keep it alive to aggregate status and
coordinate shutdown.

### Host service

The Host service is a thin, long-lived supervisor. For each accepted deployment
it creates an isolated local run process, and that process creates the existing
`ModuleCoordinator`. Isolation avoids sharing mutable global configuration,
module state, and logs with the Host service.

The first implementation may advertise one deployment slot and allow only one
active distributed deployment per Host. The descriptor and protocol should
still include capacity, `run_id`, and lease/generation fields so multi-run Hosts
can be added later without changing placement semantics.

## Host identity, discovery, and advertisement

### Starting a Host

A Host starts with a generated identity and user-supplied tags:

```sh skip
ssh some_host 'dimos host serve --tag gpu --tag something'
```

On first start, the service generates and persists an opaque `host_id`. Its
default display name is the machine hostname. A deployment that needs a stable
human name may override only the display name:

```sh skip
dimos host serve --name g1-01 --tag g1 --tag lab-a
```

The Host publishes its name and tags in its descriptor.

Names are conveniences, not protocol identities. Two live Hosts may
accidentally advertise the same name. An exact-name placement is rejected as
ambiguous in that case; the opaque Host ID can disambiguate it. Protocol keys,
leases, and persisted run assignments always use `host_id`.

### Joining the fabric

The controller only needs a way onto a Zenoh fabric containing the desired
Hosts. It can connect to a known peer/router:

```sh skip
dimos --zenoh-connect=tcp/some_host:7447 run app
```

Or it can use local multicast discovery:

```sh skip
dimos --zenoh-autodiscovery run app
```

`--zenoh-autodiscovery` enables Zenoh multicast scouting on the local network.
A distributed placement selects Zenoh as its transport backend.

Each remote Host must independently join that same fabric, either through its
local router, explicit connect configuration, or multicast scouting. Connecting
the controller to a router does not configure connectivity for a disconnected
Host.

### Advertisement protocol

The discovery and control protocol uses Zenoh liveliness for presence and
Zenoh RPC queryables for Host operations. The MVP reserves these namespaces:

| Purpose | Zenoh key expression |
| --- | --- |
| Host presence | `dimos/hosts/<host-id>/live` |
| Host control RPC | `dimos/rpc/hosts/<host-id>/<operation>` |
| Application stream | `dimos/runs/<run-id>/streams/<logical-stream>/<message-type>` |
| Module RPC | `dimos/rpc/runs/<run-id>/hosts/<host-id>/modules/<module>/<method>` |

The namespace has these required properties:

- Host presence disappears when its Zenoh liveliness token is lost;
- a descriptor is fetched from the live Host rather than trusted from a stale
  retained value;
- discovery works through routers and gossip, not only local multicast;
- control operations are addressed by opaque Host ID;
- application stream keys use a separate run-scoped namespace;
- module RPC keys are scoped by run, Host, and module instance;
- one Host restart has an observable incarnation/epoch, preventing an old lease
  from being mistaken for a current one.

Zenoh storage can retain inventory or historical resource data later, but a
stored key/value record must not be the source of truth for schedulability. The
live Host's query response and successful prepare lease are authoritative.

A minimal Host descriptor contains:

| Field | Purpose |
| --- | --- |
| `host_id`, `name`, `epoch` | Stable identity, display, and current process incarnation. |
| `tags` | Opaque placement labels configured by the user or a capability provider. |
| `capabilities` | Structured hardware facts, such as robot model/serial, GPU type, architecture, and attached devices. |
| `resources_total`, `resources_available` | CPU, memory, accelerators, deployment slots, and other schedulable capacity. |
| `protocol_version`, `plan_schema_version` | Control and deployment-plan compatibility. |
| `dimos_version`, `application_revision` | Runtime and code compatibility. |
| `active_runs`, `leases` | Current occupancy and pending reservations. |
| `health`, `last_error` | Whether the Host can accept work and why not. |

Tags stay simple in the MVP. Structured capabilities prevent an eventual
vocabulary such as robot serial numbers or GPU memory from being flattened into
an unvalidated pile of tags. Initially, `--tag g1` can enable embodiment
selection; later a Unitree capability provider can advertise the same fact
automatically and include model, serial, firmware, and safety state.

Descriptors are scheduling hints and may change immediately after discovery.
The `prepare` operation must revalidate compatibility, tags, devices, and free
resources while acquiring a bounded lease.

## Blueprint placement API

An application remains a normal `Blueprint`. Placement metadata on one or more
blueprint atoms or composed fragments activates distributed behavior.

The additive placement API is a `placement()` modifier:

```python skip
unitree_go2_markers = autoconnect(
    unitree_go2,
    MarkerDetectionStreamModule.blueprint(
        marker_length_m=0.1,
        camera_info=GO2Connection.camera_info_static,
    ).placement(tags={"gpu"}),
)
```

This runs `unitree_go2` on the controlling machine by default and places marker
detection on one discovered compatible Host tagged `gpu`.

Placement by embodiment and exact robot name uses the same mechanism:

```python skip
multi_host_g1 = autoconnect(
    unitree_g1.blueprint().placement(tags={"g1"}),
    navigation.blueprint(),
    expensive.blueprint().placement(tags={"gpu"}),
)

pinned_g1 = autoconnect(
    unitree_g1.blueprint().placement(host="g1-01"),
    navigation.blueprint(),
    expensive.blueprint().placement(tags={"gpu"}),
)
```

The first application selects one available G1 Host, runs navigation locally,
and selects one available GPU Host. The second pins only the robot fragment to
the uniquely advertised name or Host ID `g1-01`.

Placement is immutable blueprint metadata exposed through `placement()`. It is
not forwarded to module constructors, and the same modifier applies to a module
or a composed fragment.

The API has the following semantics:

- `.placement(host=...)` is an exact selector matching a unique advertised Host
  name or opaque Host ID.
- `.placement(tags={...})` requires one Host containing every listed tag.
- When both are given, the exact Host must also satisfy all tags.
- Applying `.placement(...)` to a composed fragment makes it one placement unit;
  all modules in that fragment are co-located.
- A module with no placement metadata has a soft preference for the controlling
  machine, preserving current behavior.
- `.placement(local=True)` can express a hard local constraint when needed.
- A module instance is assigned to exactly one Host. Replication or “all matching
  Hosts” requires a future explicit cardinality API.
- Placement is resolved once at startup and recorded by Host ID. A later Host
  join, name change, or load change does not silently move the application.

The controller's own runtime is represented internally by a synthetic,
run-scoped local Host identity. It participates in placement and aggregated
status, but is not advertised to other controllers and cannot be selected by a
remote Host name.

If users want two copies of a module, they must still create two named or
namespaced instances as they do for a local blueprint. Placement does not clone
blueprint atoms.

## Placement resolution

Placement solves constraints over live Host descriptors. The MVP algorithm is
small, explainable, and deterministic:

1. Resolve the full logical graph and all placement metadata.
2. Form placement units. A fragment explicitly placed together is one unit.
   Modules connected by a local-only module reference are also merged into one
   unit because such references cannot cross Hosts.
3. Add the controlling machine as the preferred candidate for unconstrained
   units.
4. Take one discovery snapshot of live Host descriptors.
5. For each unit, filter candidates by exact Host selector, required tags,
   platform and plan compatibility, required code revision, advertised devices,
   available resources, and deployment slots.
6. Intersect constraints within every co-location unit. Conflicting exact Hosts,
   an explicit local constraint combined with a remote constraint, or an empty
   candidate set is a compile/placement error.
7. Assign exact selectors first. For tag-only selectors, prefer the least
   committed compatible Host, then stable Host ID as a deterministic
   tie-breaker. More advanced scoring can be added later without changing the
   selector model.
8. Print or expose the resulting placement plan, then acquire prepare leases.
9. Freeze successful assignments by Host ID in the run record.

Unconstrained modules stay local unless merging a module-reference co-location
unit with an explicitly placed module forces them onto that module's Host. The
placement explanation should make this inheritance visible.

The controller must fail clearly when a selector cannot be satisfied. A useful
diagnostic names the module or fragment, its constraints, and why each
discovered Host was rejected, for example:

```text
No Host satisfies placement for MarkerDetectionStreamModule
  required tags: [gpu]
  discovered:
    g1-01 (4ec2...): missing tag gpu
    gpu-lab (8aa1...): busy, 0/1 deployment slots available
```

An exact selector that is absent, duplicated, incompatible, or busy is also an
error. The controller does not wait forever for a matching Host by default;
an explicit placement timeout can be added for automation.

Because discovery races with other controllers, selection alone does not claim
capacity. A Host accepts work only after `prepare` revalidates the descriptor and
creates a lease. If a lease race is lost, the MVP should report the changed
placement state; a bounded re-plan may be added, but it must be visible rather
than silently starting on an unexpected robot.

## Graph compilation and partitioning

The controller must resolve the complete logical graph before asking any Host to
start. It then compiles one immutable deployment plan per selected Host.

Compilation has the following stages:

1. Load the blueprint and apply application-level configuration.
2. Resolve module instance names, stream auto-connections, remappings,
   namespaces, module-reference requirements, and placement constraints across
   the complete graph.
3. Discover Hosts and resolve every placement unit to exactly one Host ID.
4. Classify every connection as local or cross-Host.
5. Validate transports and module references against the rules below.
6. Create a Host fragment containing local modules, local wiring, boundary
   stream endpoints, a resolved configuration snapshot, and resource/lease
   requirements.

Hosts should not independently recompile the original application. Doing so
could produce different graphs because of environment variables, installed
packages, discovery timing, or machine-local configuration. A Host receives a
resolved plan and performs only local validation and deployment.

The deployment plan needs a serializable, versioned representation. Sending
arbitrary live Python objects is convenient for a prototype, but a plan schema
is safer because it makes compatibility checks, diagnostics, and future
non-Python Hosts possible.

## Stream transport rules

For the MVP, a distributed run requires Zenoh as its shared transport backend.
LCM remains supported for existing single-machine runs, but is rejected once a
blueprint resolves onto more than one Host.

| Connection | MVP rule |
| --- | --- |
| Local stream using Zenoh | Supported. |
| Cross-Host stream using Zenoh | Supported. |
| Any stream using LCM in a multi-Host run | Rejected. |
| Local stream using shared memory | Supported when explicitly configured. |
| Cross-Host stream using shared memory | Rejected. |
| Other machine-local transports | Supported only when both endpoints are on the same Host. |

Keeping explicit shared memory for local image or point-cloud paths avoids a
performance regression. The compiler must fail before deployment if an
explicitly pinned machine-local transport crosses a Host boundary.

### Topic identity and isolation

Both ends of a boundary stream must independently derive the same Zenoh key
expression. They must also be isolated from older or concurrent runs of the
same application. Physical topics should therefore include a run-scoped prefix,
`dimos/runs/<run-id>/streams/<logical-stream>/<message-type>`.

Within that namespace, the following are required:

- identical logical stream name and message type resolve to the same key on all
  Hosts in a run;
- different run IDs never share a physical key;
- message type mismatches fail during graph validation;
- topic construction is centralized rather than reimplemented by each Host;
- discovery and control keys use separate namespaces from application streams.

Run scoping is a necessary change to the current topic factory, which currently
uses a global `dimos/<name>` namespace for Zenoh.

Application module RPC uses the corresponding run-scoped RPC namespace:
`dimos/rpc/runs/<run-id>/hosts/<host-id>/modules/<module>/<method>`. RPC and
stream traffic travel directly between the participating module runtimes over
Zenoh; the controller, Host service, and Host client do not proxy them.
The MVP still keeps module-reference-connected modules on the same Host; the
run-scoped RPC namespace prevents collisions on the shared fabric and leaves a
stable address for later cross-Host module references.

## Module-reference rules

A module reference is a typed module or `Spec` dependency that the coordinator
resolves to an injected RPC proxy. Cross-Host references introduce additional
requirements for identity, availability, timeout behavior, and lifecycle
ordering.

The MVP therefore applies one simple rule:

> A module and every module reference it consumes must belong to the same
> placement unit and resolve to the same Host.

The graph compiler merges reference-connected modules before placement. If one
module has a remote placement constraint and the other is unconstrained, both
are placed remotely. If they carry incompatible constraints, compilation fails
with the consumer, provider, reference field, and conflicting selectors.

Local module references use the existing coordinator behavior with no module
API change. Cross-Host references are outside the MVP.

## Host control protocol

The protocol should be small and versioned. A minimal logical API is:

| Operation | Purpose |
| --- | --- |
| `describe` | Return the current Host descriptor. |
| `prepare` | Revalidate constraints, acquire a bounded lease, and stage a resolved fragment. |
| `start` | Build and start the staged local deployment. |
| `status` | Return deployment state, lease state, and module summaries. |
| `stop` | Gracefully stop a run, escalating according to existing local policy. |
| `logs` | Return log metadata or a bounded stream for one run. |

Every mutating request must include at least:

- `run_id`, generated once by the controller;
- target `host_id` and observed Host `epoch`;
- protocol and plan schema versions;
- a request ID or generation number for idempotency;
- the placement constraints and resources the Host must revalidate;
- a lease deadline for staged but uncommitted work.

Repeated `prepare`, `start`, or `stop` requests for the same generation should
return the current result rather than starting duplicate deployments. A Host
must reject a conflicting run when it has no free deployment slot. An expired
prepare lease must clean up staged state without killing a committed run.

Host services use opaque-ID-scoped RPC names under
`dimos/rpc/hosts/<host-id>/`. Module RPC names are scoped by run, Host, and
module instance wherever they are visible on the shared Zenoh fabric.

Connecting a Host client and server means that both join the same Zenoh fabric:
the server declares its liveliness token and control queryables, while the
controller uses one shared Zenoh RPC channel to address every selected Host.
There is no dedicated socket or Zenoh session per Host client.

`HostClient` is an internal startup facade owned by the controller. It uses one
shared Zenoh RPC channel to discover, prepare, and start selected Hosts. After
startup, application module RPC and streams communicate directly through their
run-scoped Zenoh keys. The controller may retain the control channel for
`status`, `logs`, and `stop`, but application code does not explicitly call a
`HostClient` and the client is not part of the application data path.

## Startup sequence

A distributed startup should follow this order:

1. **Compile:** the controller resolves the complete logical application and
   co-location units.
2. **Join and discover:** it joins the configured Zenoh fabric, collects live
   Host descriptors, and adds its own local capacity.
3. **Place:** it resolves exact names/IDs and tags, validates constraints, and
   produces an explainable placement plan.
4. **Prepare and lease:** the controller sends each selected Host its resolved
   fragment. Every Host revalidates identity, versions, code, tags, devices,
   resources, availability, imports, configuration, and transport capability
   while acquiring a bounded prepare lease.
5. **Commit:** only after all Hosts prepare successfully does the controller ask
   all Hosts, plus its local coordinator, to start.
6. **Observe:** the controller waits for every local coordinator to report
   started, then records the application as running with its resolved Host IDs.
7. **Rollback on failure:** if any Host fails before the application reaches
   running, the controller stops all Hosts that prepared or started successfully.

This is a lightweight two-phase startup, not a durable distributed transaction.
Rollback is best effort and status must report any unreachable Host explicitly.

Modules may begin publishing before remote subscribers are ready. Boundary
streams must follow Zenoh's configured delivery semantics; startup must not
silently imply buffering or replay that the transport does not provide.

## Lifecycle and failure semantics

The distributed run has an aggregate state derived from its Hosts:

| State | Meaning |
| --- | --- |
| `placing` | Discovery or constraint resolution is in progress. |
| `preparing` | At least one Host is leasing, validating, or staging its fragment. |
| `starting` | All Hosts prepared and at least one is starting. |
| `running` | Every required Host reports its local deployment running. |
| `degraded` | The application started, but a Host or required module was later lost. |
| `stopping` | A coordinated stop is in progress. |
| `stopped` | Every reachable Host has stopped the run. |
| `failed` | Placement or startup failed, or the application cannot satisfy a required constraint. |

MVP failure behavior is intentionally conservative:

- no module is automatically moved to another Host;
- a Host disappearing during placement makes it ineligible;
- loss of a selected Host after commit marks the application degraded;
- the Host service reports local module failure through existing coordinator
  status;
- stopping an application contacts the Host IDs recorded in the run, not
  whichever Hosts now have the same names or tags;
- stopping reports unreachable Hosts rather than claiming a clean stop;
- a restarted Host does not automatically recreate its last deployment unless
  an explicit recovery design is added;
- a Host cleans up partially started local coordinators and expired prepare
  leases before advertising the corresponding capacity as available.

The controller and Hosts should use liveliness plus a heartbeat or lease to
detect loss of control connectivity. Controller lease expiry should be a status
signal only in the first MVP; it must not kill healthy robot control modules
without an explicit, robot-specific safety policy.

## Configuration, code, and artifacts

The first version assumes each selected Host already has:

- the same compatible DimOS and application code installed;
- all required Python and native dependencies;
- required models, calibration files, and other assets;
- access to machine-local devices and credentials.

The controller sends resolved application configuration, while Host-local
settings such as device paths or secrets may be supplied by Host configuration.
The precedence between application and Host-local configuration is:

1. immutable placement and run metadata from the controller;
2. application configuration resolved by the controller;
3. allowlisted Host-local overrides for device-specific values and secrets.

Preflight compares a release identifier or source revision and fails on an
incompatible version by default. Automatically distributing code and artifacts
is a later deployment-system concern, not part of Host orchestration.

## Multi-embodiment direction

Tags enable a useful first step:

```python skip
unitree_g1.blueprint().placement(tags={"g1"})
```

This means “bind this placement unit to one live, compatible, available Host
advertising `g1`,” not “run on every G1.” The selected robot must be recorded by
opaque Host ID and, when available, robot serial number. A prepare lease makes
the physical embodiment exclusive to that run.

The longer-term capability model should distinguish:

- **identity:** this particular robot, such as `g1-01` or a serial number;
- **kind:** broad embodiment compatibility, such as G1 or Go2;
- **capabilities:** arms, grippers, cameras, locomotion, payload, firmware, and
  safety state;
- **resources:** CPU, GPU, memory, battery, and deployment slots;
- **policy:** whether a controller is authorized to claim and command it.

This lets the same placement mechanism select a robot, an onboard controller,
a nearby edge GPU, or cloud compute. Fleet fan-out, selecting a count of robots,
anti-affinity, and coordinated multi-robot safety remain explicit later phases.

## Observability

One distributed run has one application `run_id`. Every log and status record
also includes `host_id`, the discovered Host name at placement time, and, where
applicable, `module_name`.

At minimum, aggregated status should show:

- application name, run ID, and aggregate state;
- the Zenoh discovery/connect mode used for the run;
- every selected Host ID, its name, matched tags, version, heartbeat, and state;
- why each placement unit selected its Host;
- modules deployed on each Host and their lifecycle state;
- the first startup or runtime error per Host;
- unresolved cleanup work for unreachable Hosts.

Useful commands are:

```sh skip
dimos host list
dimos run app --explain-placement
dimos status --run <run-id>
dimos log --run <run-id> --host gpu-lab
dimos stop --run <run-id>
```

`dimos host list` queries the live fabric. Status, logs, and stop use the
immutable Host-ID assignments stored in the run record even if a display name
later changes.

For the MVP, logs can remain on each Host and be fetched or tailed on demand.
Central log storage is not required, but the user should not need to know the
Host's filesystem layout to find a module's logs.

## Security and network assumptions

The MVP targets a trusted robot or development network with a controlled Zenoh
deployment. A discovered Host is not proof of identity, and this design must not
be presented as safe for arbitrary public networks or an untrusted shared router.

Before broader deployment, the control plane needs authenticated Host identity,
controller authorization, encrypted transport, replay protection, lease
ownership, secret handling, and audit logs. These requirements should be
isolated behind the Host client/service protocol so they do not affect module
APIs. Exact physical-robot selection should eventually use a cryptographic
identity in addition to a human-readable name.

## Compatibility and breaking changes

The design is additive at the user-facing module and blueprint level:

- Existing local `Blueprint` objects and `dimos run <blueprint>` keep their
  current behavior.
- `Module`, `In`, `Out`, `@rpc`, and local module-reference APIs do not change.
- The existing local coordinator and worker implementation remain the runtime
  on every machine.
- A blueprint with no remote placement metadata remains local.
- A blueprint with remote placement metadata remains a `Blueprint`.
- Network locators define Zenoh fabric membership; Host descriptors define
  placement candidates.

Required internal changes are:

1. Add immutable placement metadata to `BlueprintAtom`/composed `Blueprint`
   values without forwarding it to module constructors.
2. Add co-location grouping, Host discovery, constraint resolution, and a graph
   partitioner.
3. Add a Host service/client, persistent Host identity, descriptors, liveliness,
   leases, and Host CLI commands.
4. Scope coordinator/control RPC identities by Host ID and run ID.
5. Add run-scoped Zenoh topic and RPC namespaces.
6. Extend run registry, status, log, and stop operations with immutable Host-ID
   assignments and placement explanations.
7. Define a serializable, versioned Host deployment plan and descriptor schema.

The main behavioral restrictions are visible only when a run resolves onto
more than one Host:

- Zenoh is required;
- LCM is rejected;
- shared-memory and other local transports cannot cross Hosts;
- module references create co-location constraints and cannot cross Hosts;
- code and dependencies must already exist on each selected Host.
