# Workspace creator guidance

Follow the root AGENTS.md. Use `--manifest-path installer/Cargo.toml` for Cargo.

The creator only scaffolds and restores projects. `src/setup/workspace.rs` creates
files once; restoration must preserve developer-owned source and manifests.
`resources/activate.sh` and `environment.py` implement shared Bash/Zsh/direnv
activation and complete manual deactivation. Activation never installs anything.
The Python `dimos.cli.doctor` command owns shared readiness checks and must remain
usable without importing the robotics stack at entry.

uv owns Python; Pixi owns userspace dependencies; Nix owns custom native builds.
Do not silently drop extras, add fallback package managers, or defer required
native builds. The profile references remain unitree-go2 and
xarm7-planner-coordinator. Generic ARM builds do not establish Jetson support.

Run Cargo tests/Clippy, the lightweight Python and shell tests, and
`installer/run/acceptance <profile>`. `run/package` builds release assets but does
not publish. Do not introduce another release cadence or global management CLI.
