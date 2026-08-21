## Why

Complete `dimos eval` benchmarks currently run their evaluator and Pi runtime in the mutable host environment even though their native simulators are isolated. The LIBERO autoresearch wrapper proved that a locked outer image removes checkout, dependency, temporary-filesystem, and GPU-runtime drift, but keeping that wrapper benchmark-specific duplicates the execution boundary every future Evaluation would need.

## What Changes

- Replace the LIBERO-specific wrapper with a shared, locally built evaluation image and launcher used by every built-in `dimos eval` command.
- Make container execution mandatory for the public CLI while keeping the existing Python runner as the private inside-container implementation.
- Grant the outer image GPU access and the host rootless Podman socket so existing native simulator containers remain siblings.
- Stage inputs, outputs, caches, and IPC on explicit identical absolute mounts, then atomically publish the completed run.
- Reuse the shared launcher for LIBERO autoresearch without changing its benchmark command.
- Remove unused external Evaluation entry-point discovery; the executable registry contains built-ins only.

## Capabilities

### New Capabilities

- `containerized-evaluation`: Reproducible, container-only execution for built-in `dimos eval` benchmarks and the LIBERO autoresearch panel.

### Modified Capabilities

None.

## Impact

This replaces the specialized launcher with a shared evaluation module, changes the public `dimos eval run` execution path, simplifies registry discovery, and updates focused tests and documentation. It does not change runtime profiles, task instructions, perception, planning, robot control, benchmark cases, or native scoring.
