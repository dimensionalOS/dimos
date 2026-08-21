## Context

The unified `dimos eval` runner owns result publication and injects either the code-policy or live-agent Pi runtime into a native Evaluation. LIBERO-PRO and VLN-CE already keep their native simulators in sibling OCI containers, but the evaluator, Pi extension, DimOS checkout, and orchestration still depend on the host Python environment. The LIBERO autoresearch wrapper demonstrated the required outer topology and also exposed why it belongs at the shared runner seam.

## Goals / Non-Goals

**Goals:**

- Build one outer image from digest-pinned bases, `uv.lock`, and the Pi package lock.
- Make public built-in Evaluation execution container-only without changing the run specification.
- Execute exact candidate sources by immutable image ID and reuse the host rootless Podman service for sibling simulators.
- Preserve daemon-visible paths and atomically publish complete result directories.
- Reuse the same launcher for the autoresearch panel and a cheap environment check.

**Non-Goals:**

- Changing native simulator images, task cases, native scores, runtime profiles, the Agent Harness, or Evo.
- Publishing images to a registry or adding CI deployment.
- Supporting external Evaluation plugins, host-mode execution, Docker-in-Docker, privileged containers, or remote multi-host Podman.

## Decisions

### Put the container seam above the shared runner

The public CLI is a dependency-light host launcher. A hidden command invokes the existing runner only inside the built image. The launcher mounts the rootless Podman socket, sets `CONTAINER_HOST`, and uses host networking, so existing Evaluation implementations create sibling simulator containers without modification. There is no public host fallback.

### Run by the image ID emitted by Podman

The launcher builds locally with an IID file and runs the resulting `sha256:` identifier, not a mutable tag. The generic recipe pins Python and Node base digests, installs the frozen extras required by all built-ins, and builds Pi with `npm ci`. No project image is pulled from a registry.

### Stage and publish host-visible paths explicitly

The launcher creates a private staging root beside the requested output and mounts it at the identical absolute path. The inside runner writes there, sibling Podman containers can resolve its paths, and the host moves a complete result into place after exit. The specification directory is mounted read-only at the same absolute path; built-in task manifests must remain relative to it. Cache, Evo result/traces, and short IPC scratch paths are explicit mounts.

### Forward an allowlist of environment variables

The CLI forwards the selected API-key variable plus known Hugging Face credentials; autoresearch additionally forwards its three Evo variables. Values never appear in command arguments or raised errors. Every run receives host networking, the rootless socket, and a fresh run-scoped NVIDIA CDI specification, matching the current built-in resource contract.

### Keep the executable registry built-in only

Installed entry-point discovery has no in-repository adapter and cannot be reproduced by the locked local image. Remove that hypothetical seam rather than introduce plugin-specific image metadata. New executable Evaluations are added explicitly to the built-in registry and therefore enter the same image through the checked-in dependency lock.

## Risks / Trade-offs

- **Host Podman socket grants container-management authority** → Mount only the rootless user's socket and document that the image is locally built from the current trusted checkout.
- **Identical absolute mounts expose selected host directories** → Mount only the input directory, private staging root, cache, and explicit Evo paths; reject filesystem root.
- **Large locked image build** → Keep simulator dependencies in the existing separate image and rely on local layer caching.
- **NVIDIA Container Toolkit or the Podman socket can be absent** → Fail in the cheap `check` mode with an actionable error before running the panel; keep generated CDI data scoped to run scratch.

## Migration Plan

Replace the existing specialized wrapper and remove plugin discovery in the container-execution PR. Verify both built-ins through the public command, then use the unchanged autoresearch benchmark command. This intentionally replaces host execution rather than preserving a compatibility path.

## Open Questions

None.
