## ADDED Requirements

### Requirement: Runtime preparation is blueprint-scoped
The system SHALL provide an explicit runtime preparation command that resolves runtime environment names by loading a blueprint configuration before preparing any environment.

#### Scenario: Prepare active placement runtimes for blueprint
- **WHEN** the user runs `dimos runtime prepare <blueprint>` with valid run-like configuration flags
- **THEN** the system loads the blueprint, identifies active module placements, and prepares only the runtime environments referenced by those active placements

#### Scenario: Runtime name is resolved within blueprint
- **WHEN** the user runs `dimos runtime prepare <blueprint> --runtime <name>`
- **THEN** the system resolves `<name>` against the runtime environment registry produced by that blueprint configuration

#### Scenario: Runtime exists but is unused
- **WHEN** the requested runtime name exists in the blueprint registry but is not referenced by an active module placement
- **THEN** the preparation command fails with an error explaining that the runtime is not used by the active blueprint configuration

### Requirement: Runtime preparation is explicit and repeatable
The system SHALL keep runtime environment preparation separate from blueprint execution and SHALL make the preparation command safe to rerun when project manifests change.

#### Scenario: Prepare uv-only project runtime
- **WHEN** an active runtime placement references a Python project runtime with `pyproject.toml` and no `pixi.toml`
- **THEN** runtime preparation runs `uv venv --seed` and `uv sync` from the project directory

#### Scenario: Prepare Pixi-backed uv project runtime
- **WHEN** an active runtime placement references a Python project runtime with both `pyproject.toml` and `pixi.toml`
- **THEN** runtime preparation runs `pixi install`, `pixi run uv venv -p .pixi/envs/default/bin/python --seed`, and `pixi run uv sync` from the project directory

#### Scenario: Prepare reruns sync commands
- **WHEN** a prepared project runtime already has a `.venv`
- **THEN** runtime preparation still runs the applicable Pixi and uv sync commands instead of skipping solely because the `.venv` exists

### Requirement: Blueprint run fails fast for unprepared project runtimes
The system SHALL NOT install, sync, or otherwise mutate Python project runtime environments during normal blueprint run or build.

#### Scenario: Project runtime is missing venv Python
- **WHEN** `dimos run <blueprint>` evaluates an active placement using a Python project runtime whose `.venv/bin/python` is missing
- **THEN** blueprint build fails before worker launch with a message naming the runtime, blueprint, project path, missing executable, and the `dimos runtime prepare` command to run

#### Scenario: Project runtime exists
- **WHEN** `dimos run <blueprint>` evaluates an active placement using a Python project runtime whose `.venv/bin/python` exists
- **THEN** the system attempts worker launch without running Pixi or uv sync commands
