# Isolated Python runtimes own their dependencies

Each isolated runtime syncs its own lockfile, installs the matching host DimOS code
with `--no-deps`, and launches with `uv run --no-sync`. The former host-dependency
overlay cannot support OmniGibson's NumPy 1 requirement alongside the host's Rerun /
NumPy 2 stack. Runtime manifests must explicitly declare the dependencies used by
their implementation and DimOS transport/message imports; the isolated environment
is intentionally not a full host installation.

Source checkouts use editable host code. Installed hosts use the exact installed
DimOS version. All isolated callers use the same preparation path.
