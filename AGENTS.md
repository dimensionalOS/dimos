# Repository Guidelines

## Project Structure & Module Organization
- Core package lives in `dimos/`; key areas include `agents/` (agent orchestration), `robot/` (Unitree/WebRTC control & CLI), `skills/`, `perception/`, `navigation/`, and `utils/` (CLI tools, LCM helpers).
- Integration and scenario tests sit in `tests/` (webcam, ROS, manipulation, navigation, streaming) plus marker-based suites under `dimos/` as configured in `pyproject.toml`.
- Assets for docs and demos are in `assets/`; environment presets live in `default.env`; Docker/Nix helpers are under `docker/` and `flake.nix`.

## Setup, Build, and Run
- Create a venv (`python3 -m venv venv && source venv/bin/activate`) then install extras: `pip install -e .[cpu,dev]` (use `[cuda,dev]` on GPU hosts).
- Copy env defaults and fill secrets: `cp default.env .env`.
- Quick sanity test: `pytest -s dimos/` (respects default markers) or target a single path: `pytest tests/test_agent.py -s`.
- CLI entrypoints are installed via `pip install -e .`; examples: `dimos` (robot CLI), `agentspy`, `skillspy`, `foxglove-bridge`, `lcmspy`.
- For replay without hardware: `CONNECTION_TYPE=replay python dimos/robot/unitree_webrtc/unitree_go2.py`.

## Coding Style & Naming Conventions
- Python 3.10+ with type hints preferred; `mypy` is strict (see `pyproject.toml` exclusions).
- Run `ruff check .` (line length 100) before pushes; `yapf` is available for formatting if needed.
- Use snake_case for modules/functions, PascalCase for classes, SCREAMING_SNAKE_CASE for constants; favor explicit imports and small, testable functions.

## Testing Guidelines
- Default pytest config excludes heavy/gpu/ROS/visual tests via markers (`-m 'not vis and not benchmark ...'`); mark new tests appropriately.
- Co-locate light unit tests near code when practical; add integration/streaming cases under `tests/`.
- For hardware/ROS-dependent runs, opt-in with markers (e.g., `-m ros`) and document prerequisites in the PR.

## Commit & Pull Request Guidelines
- Recent history uses short imperative messages (e.g., `fix tests`, `cleanup logic`); follow that style and keep commits focused.
- In PRs, include: purpose, key changes, run commands/results (tests or linters), affected platforms/hardware (sim vs real), and links to issues/tasks. Add screenshots/GIFs for UI or visualization changes and sample logs for streaming/robot runs.

## Security & Configuration Tips
- Never commit API keys or robot IPs; rely on `.env`/environment variables (`OPENAI_API_KEY`, `CLAUDE_API_KEY`, `ALIBABA_API_KEY`, etc.).
- Large binaries/models belong in data stores or LFS, not regular commits. Document any required external downloads or model paths when adding new features.
