## ADDED Requirements

### Requirement: Python project runtime environments resolve by convention
The system SHALL support a named Python project runtime environment that resolves Python worker launch material from a project directory using DimOS-defined conventions.

#### Scenario: Register Python project runtime environment
- **WHEN** a blueprint registers `PythonProjectRuntimeEnvironment(name="worker", project=Path("packages/worker"))`
- **THEN** the runtime environment registry stores the environment under `worker` without requiring explicit manifest, venv, or Pixi path fields

#### Scenario: pyproject is required
- **WHEN** a Python project runtime environment is resolved for preparation or worker launch and the project directory does not contain `pyproject.toml`
- **THEN** resolution fails with an actionable error explaining that first-slice Python project runtimes require a uv project

#### Scenario: Pixi is optional
- **WHEN** a Python project runtime environment project contains `pyproject.toml` but no `pixi.toml`
- **THEN** the system treats it as a uv-only Python project runtime

#### Scenario: Pixi-backed uv project is detected
- **WHEN** a Python project runtime environment project contains both `pyproject.toml` and `pixi.toml`
- **THEN** the system treats it as a Pixi-backed uv runtime where Pixi supplies the Python used to create the project-local uv `.venv`

### Requirement: Python project runtime launch is non-mutating
The system SHALL resolve Python project runtime launch commands without performing environment preparation during worker deployment.

#### Scenario: Resolve uv-only launch command
- **WHEN** a uv-only Python project runtime is used for worker launch
- **THEN** the worker launcher receives a non-mutating command equivalent to `uv run --no-sync python -m dimos.core.coordination.venv_worker_entrypoint ...`

#### Scenario: Resolve Pixi-backed launch command
- **WHEN** a Pixi-backed uv runtime is used for worker launch
- **THEN** the worker launcher receives a non-mutating command equivalent to `pixi run uv run --no-sync python -m dimos.core.coordination.venv_worker_entrypoint ...`
