## ADDED Requirements

### Requirement: Launch placed modules in Python Runtime Workers
Placed Python modules SHALL execute in Python worker processes launched from their registered runtime environment.

#### Scenario: Placed module uses runtime worker pool
- **GIVEN** a Module Contract placed into runtime `detector-runtime`
- **WHEN** deployment launches the module
- **THEN** the module executes in a Python Runtime Worker for `detector-runtime`
- **AND** unplaced Python modules continue to use the default Python worker pool.

### Requirement: Preserve Python module semantics
Python Runtime Workers SHALL preserve normal DimOS Python module semantics for placed modules.

#### Scenario: Placed module exposes RPC and streams
- **GIVEN** a placed module with typed streams and RPC methods declared by its Module Contract
- **WHEN** the module deploys in a Python Runtime Worker
- **THEN** stream wiring, RPC calls, lifecycle calls, module refs, and actor proxy behavior work as they do for default Python workers
- **AND** `@skill` methods remain available through the normal Python module tooling.

### Requirement: Keep existing worker protocol behavior
Runtime-aware worker deployment SHALL preserve the existing request/response control behavior for Python workers.

#### Scenario: Coordinator calls runtime module method
- **GIVEN** a placed module deployed in a Python Runtime Worker
- **WHEN** the coordinator sends a method call through the module actor proxy
- **THEN** the call is routed to the runtime implementation instance
- **AND** results and errors are returned through the normal worker response path.

### Requirement: Fail clearly when runtime worker cannot start
Deployment SHALL report actionable errors when a Python Runtime Worker cannot start or connect to the coordinator.

#### Scenario: Runtime worker entrypoint fails
- **GIVEN** a placed module whose runtime environment cannot launch the Python worker entrypoint
- **WHEN** deployment attempts to start the runtime worker
- **THEN** deployment fails with an error naming the runtime environment
- **AND** no placed module is reported as successfully deployed in that failed worker.
