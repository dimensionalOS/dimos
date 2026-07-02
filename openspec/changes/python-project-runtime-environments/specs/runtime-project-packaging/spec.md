## ADDED Requirements

### Requirement: Runtime Projects are lockfile-backed
Runtime Projects SHALL include committed lockfile state sufficient for deployment-time reconciliation without rewriting project metadata.

#### Scenario: uv runtime project has lockfile
- **GIVEN** a uv-backed Runtime Project with `pyproject.toml` and committed `uv.lock`
- **WHEN** deployment reconciles the project
- **THEN** reconciliation uses locked package-manager behavior
- **AND** source-controlled project metadata is not rewritten.

#### Scenario: Runtime project lockfile is missing or stale
- **GIVEN** a Runtime Project without usable committed lockfile state
- **WHEN** deployment reconciles the project in locked mode
- **THEN** reconciliation fails clearly
- **AND** the error explains that lockfile mutation belongs to a manual package-manager command or future build/update command.

### Requirement: Runtime Projects may use uv or Pixi-backed uv
Runtime Projects SHALL support uv-backed environments and MAY support Pixi-backed uv environments.

#### Scenario: Pixi-backed runtime project
- **GIVEN** a Runtime Project with Pixi project metadata and uv-managed Python environment requirements
- **WHEN** deployment reconciles the project
- **THEN** Pixi environment reconciliation runs in locked mode before uv synchronization
- **AND** uv synchronization also uses locked, non-mutating behavior.

### Requirement: Runtime-only dependencies stay out of coordinator imports
Runtime Project dependencies SHALL be importable by the Runtime Implementation without requiring those dependencies in the coordinator environment.

#### Scenario: Coordinator imports dependency-light contract
- **GIVEN** a Module Contract file that does not import runtime-only dependencies
- **AND** a Runtime Implementation in a Runtime Project that imports runtime-only dependencies
- **WHEN** the coordinator imports the blueprint and builds the graph
- **THEN** coordinator import succeeds without installing runtime-only dependencies
- **AND** the Runtime Implementation can use those dependencies inside the Python Runtime Worker.
