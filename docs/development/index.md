# Development Guide

This guide gets you from clone to running tests. For CLI usage, see [docs/usage/cli.md](/docs/usage/cli.md).

## Setup

### Clone

```bash
# Skip LFS data on clone (downloads on demand later)
export GIT_LFS_SKIP_SMUDGE=1
git clone https://github.com/dimensionalOS/dimos.git
cd dimos
```

### Install Dependencies

```bash
# Create the virtualenv and install all dev dependencies
uv sync --all-extras --no-extra dds
source .venv/bin/activate
```

### Pre-commit Hooks

```bash
pre-commit install
```

This enables automatic checks on every commit: formatting (ruff), linting (ruff), license headers, trailing whitespace, JSON/TOML/YAML validation, editorconfig, LFS guards, and more.

> **Tip:** Pre-commit auto-fixes most formatting issues. If it modifies files, re-stage them and commit again.

## Linting & Type Checking

```bash
# Format
ruff format dimos/

# Lint (with auto-fix)
ruff check --fix dimos/

# Type check (strict mode, Python 3.12)
mypy dimos/
```

Ruff config lives in `pyproject.toml` under `[tool.ruff]`. Mypy config is under `[tool.mypy]`.

## Testing

Run the fast test suite:

```bash
pytest --numprocesses=auto dimos
```

Or equivalently:

```bash
./bin/pytest-fast
```

Run slow tests (integration-level, before opening a PR):

```bash
./bin/pytest-slow
```

See [testing.md](/docs/development/testing.md) for markers, fixtures, mocking patterns, and advanced test configuration.

## Project Structure

```
dimos/              Main package
├── core/           Module system, streams, blueprints, transports
├── agents/         Agent framework (MCP client/server, skills)
├── robot/          Hardware drivers (Unitree, drone, arms)
├── navigation/     SLAM, planners, costmaps
├── perception/     Detectors, VLMs, depth, audio
├── memory/         Spatial and temporal memory
├── manipulation/   Arm control, grasping
├── simulation/     MuJoCo integration
├── msgs/           Message type definitions
└── skills/         Agent skill implementations
docs/               Documentation
├── usage/          User-facing docs (modules, blueprints, CLI, transports)
├── development/    Contributor docs (this guide, testing, profiling)
├── platforms/      Hardware platform guides
├── capabilities/   Feature guides (navigation, agents, manipulation)
└── installation/   System setup guides
blueprints/         Blueprint runfiles
bin/                Dev scripts (pytest shortcuts, hooks)
data/               LFS-managed datasets
stubs/              Mypy type stubs for untyped dependencies
```

## Further Reading

| Topic | Link |
|-------|------|
| Testing | [testing.md](/docs/development/testing.md) |
| Docker images | [docker.md](/docs/development/docker.md) |
| LFS data loading | [large_file_management.md](/docs/development/large_file_management.md) |
| Profiling | [profiling_dimos.md](/docs/development/profiling_dimos.md) |
| Writing docs | [writing_docs.md](/docs/development/writing_docs.md) |
| Conventions | [conventions.md](/docs/development/conventions.md) |
| Grid testing | [grid_testing.md](/docs/development/grid_testing.md) |
