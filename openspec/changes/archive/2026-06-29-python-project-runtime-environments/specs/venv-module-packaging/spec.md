## ADDED Requirements

### Requirement: Venv Module packages can use Pixi-backed native dependencies
The system SHALL support venv Module packages whose Python dependencies require native or C++ build dependencies supplied by an optional package-local `pixi.toml`.

#### Scenario: Package declares Python dependency closure with pyproject
- **WHEN** a venv Module package is used as a Python project runtime
- **THEN** its `pyproject.toml` declares the Python package dependencies that uv installs into the project-local `.venv`

#### Scenario: Package declares native build environment with pixi
- **WHEN** a venv Module package includes `pixi.toml`
- **THEN** Pixi supplies the native build tools and libraries used while uv installs the package dependency closure

### Requirement: GPD demo proves native Python binding import in worker runtime
The system SHALL include a GPD worker demo package that proves a Python binding with native build requirements can be prepared and imported in a placed venv worker.

#### Scenario: Demo package depends on pinned GPD source
- **WHEN** the GPD demo package is prepared
- **THEN** uv installs a dependency on `TomCC7/gpd` pinned to commit `c088d8ae2f7965b067e9a12b3c0dacdbe9da924a`

#### Scenario: Demo Module imports GPD in worker
- **WHEN** the GPD demo blueprint runs with its dummy Module placed into the project runtime
- **THEN** a Module RPC lazily imports `gpd.core` inside the worker process and reports import success through normal DimOS RPC behavior

#### Scenario: Pixi integration test is optional when Pixi is unavailable
- **WHEN** the automated test environment does not have Pixi installed
- **THEN** Pixi/GPD integration tests are skipped while uv-only and command-construction tests still run
