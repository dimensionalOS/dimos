## ADDED Requirements

### Requirement: Interactive shell attachment

DimOS SHALL provide a `dimos shell` command that opens an IPython session attached to the coordinator on the configured transport bus.

#### Scenario: Attach to a CLI-launched run
- **GIVEN** a DimOS blueprint is running through `dimos run`
- **WHEN** a developer runs `dimos shell` from an interactive terminal
- **THEN** the command connects to that coordinator using the default connection timeout
- **AND** opens an IPython session without requiring an MCP server or an additional blueprint module

#### Scenario: Attach to a direct Python run
- **GIVEN** a coordinator built directly from Python has entered its blocking run loop
- **WHEN** a developer runs `dimos shell` on the same configured transport bus
- **THEN** the command attaches even when no CLI run-registry entry exists
- **AND** identifies the run as unregistered when registry metadata is unavailable

#### Scenario: Reject non-interactive execution
- **GIVEN** standard input or standard output is not an interactive terminal
- **WHEN** a caller runs `dimos shell`
- **THEN** the command exits with a clear error
- **AND** directs automation to use the `Dimos` Python interface

#### Scenario: No coordinator is available
- **GIVEN** no coordinator responds on the configured transport bus
- **WHEN** a developer runs `dimos shell`
- **THEN** the command exits after the default connection timeout
- **AND** reports that no running DimOS coordinator was found

### Requirement: Shell environment and safety

The shell SHALL provide a small predefined namespace for direct RPC debugging and SHALL make the immediate execution model visible.

#### Scenario: Initial namespace
- **GIVEN** the shell attaches successfully
- **WHEN** IPython starts
- **THEN** the namespace contains `app`, `modules`, `rpcs`, and `describe`
- **AND** `app` is the connected `Dimos` instance
- **AND** the helper names call the corresponding public discovery operations on `app`

#### Scenario: Startup safety notice
- **GIVEN** the shell attaches to any hardware, simulation, or replay stack
- **WHEN** the startup banner is displayed
- **THEN** it states that RPC calls execute immediately against the running system
- **AND** the shell does not add per-call confirmations or access restrictions

#### Scenario: Invoke an RPC
- **GIVEN** a discovered module exposes an RPC method
- **WHEN** the developer calls that method through `app`
- **THEN** the shell invokes the RPC with the supplied Python arguments
- **AND** displays or returns the remote result through normal IPython behavior

#### Scenario: Exit the shell
- **GIVEN** an attached shell session
- **WHEN** the developer exits IPython
- **THEN** the shell closes its client connection
- **AND** leaves the coordinator and deployed modules running

#### Scenario: Connection loss
- **GIVEN** an attached shell session
- **WHEN** the coordinator stops or restarts
- **THEN** affected RPC operations fail visibly
- **AND** the shell does not reconnect or replace cached proxies automatically

### Requirement: Live module discovery

The `Dimos` Python interface SHALL expose live, structured discovery of deployed module instances.

#### Scenario: List deployed modules
- **GIVEN** an attached `Dimos` instance
- **WHEN** the caller requests the module list
- **THEN** the result contains structured records for the currently deployed module instances
- **AND** each record distinguishes the exact instance name from its Python class identity

#### Scenario: Discover a newly loaded module
- **GIVEN** a shell is already attached
- **WHEN** a module is added to the running coordinator
- **THEN** the next module or RPC discovery call includes that module without an explicit refresh

#### Scenario: Resolve an exact instance
- **GIVEN** two deployed instances share the same module class
- **WHEN** the caller requests one by its exact instance name
- **THEN** `Dimos` returns the proxy for that instance

#### Scenario: Resolve a unique class name
- **GIVEN** exactly one deployed instance has a requested module class name
- **WHEN** the caller uses that class name as a convenience lookup
- **THEN** `Dimos` returns that instance

#### Scenario: Reject an ambiguous class name
- **GIVEN** multiple deployed instances share a requested module class name
- **WHEN** the caller uses only that class name
- **THEN** the lookup fails with an ambiguity error
- **AND** reports the exact instance names the caller can use

### Requirement: RPC discovery and description

The `Dimos` Python interface SHALL expose structured metadata for all advertised RPC methods, including methods marked as skills.

#### Scenario: List all RPCs
- **GIVEN** an attached `Dimos` instance
- **WHEN** the caller requests RPC discovery without a module selector
- **THEN** the result contains every advertised RPC for every current module instance
- **AND** does not filter inherited lifecycle or framework RPCs

#### Scenario: Filter RPCs by module
- **GIVEN** an attached `Dimos` instance
- **WHEN** the caller requests RPC discovery for a module proxy or module name
- **THEN** the result contains only RPCs for the resolved module instance

#### Scenario: Concise discovery output
- **GIVEN** a collection of structured RPC records
- **WHEN** IPython displays that collection
- **THEN** each RPC has a concise representation containing its module, name, parameters, and return type
- **AND** full documentation remains available through record fields and description

#### Scenario: Describe a module or RPC object
- **GIVEN** a module proxy or RPC callable obtained through `app`
- **WHEN** the caller passes it to `describe`
- **THEN** the result is the corresponding structured module or RPC record

#### Scenario: Describe a qualified string
- **GIVEN** a fully qualified module-instance and RPC name
- **WHEN** the caller passes that string to `describe`
- **THEN** the result is the matching structured RPC record
- **AND** an unqualified ambiguous RPC name is rejected

### Requirement: Inspectable RPC proxies

RPC proxy callables SHALL preserve the locally available Python method metadata used by standard inspection tools.

#### Scenario: Inspect a typed RPC
- **GIVEN** the client can import the Python class for a running module
- **WHEN** the caller uses `inspect.signature`, `help`, or IPython inspection on an RPC proxy
- **THEN** the displayed signature preserves parameter kinds, annotations, defaults, and return annotation
- **AND** omits the implementation method's leading `self` parameter
- **AND** preserves the method docstring

#### Scenario: Inspect a names-only proxy
- **GIVEN** the client cannot import the Python class for a running module
- **WHEN** the caller discovers that module's RPCs
- **THEN** RPC names remain available and callable
- **AND** unavailable signature metadata is represented as unknown rather than fabricated
