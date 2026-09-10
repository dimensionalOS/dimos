# dimup

Follow the repository AGENTS.md. This directory is an independent Python package;
it must not depend on the DimOS SDK. Run its tests with `uv run --project installer
--with pytest pytest -c installer/pyproject.toml installer/src`.

uv owns Python, apt/Homebrew own host libraries, and Cargo/Nix build native modules.
Activation never installs dependencies. Keep setup, project creation, and runtime
commands separate. The published tool exposes only `dimup`, never `dimos`.
