# Isolated Python runtimes own their dependencies

Each isolated runtime syncs its own lockfile, installs the DimOS code from the shared checkout
with `--no-deps`, and launches with `uv run --no-sync`. The former host-dependency
overlay cannot support OmniGibson's NumPy 1 requirement alongside the host's Rerun /
NumPy 2 stack. Runtime manifests must explicitly declare the dependencies used by
their implementation and DimOS transport/message imports; the isolated environment
is intentionally not a full host installation.

Both source and installed hosts resolve runtime projects and editable DimOS code
through `get_project_root()`. Environments live in the project-specific DimOS cache.
All isolated callers use the same preparation path.
