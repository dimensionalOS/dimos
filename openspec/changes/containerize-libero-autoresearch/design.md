## Context

The evaluation has two processes with different responsibilities. DimOS, Pi, perception, planning, and the policy run in the outer environment. The existing LIBERO-PRO image contains the simulator and exposes native policy/control RPCs. The simulator already uses rootless Podman and mounts verified task assets and artifacts by absolute host path.

The first Evo search exposed host-environment failures: the wrong checkout could be imported, `/tmp` filled, long Unix socket paths failed, and shared editable installs changed candidate activation. The container layer must remove those variables without changing the evaluator or requiring privileged nested containers.

## Goals / Non-Goals

**Goals:**

- Build an outer image from a digest-pinned base, `uv.lock`, and the Pi package lock.
- Execute the exact candidate sources baked into that image by immutable image ID.
- Reuse the host rootless Podman service to launch the existing simulator as a sibling.
- Preserve host-visible absolute paths for sibling asset and artifact mounts.
- Provide a cheap environment check before an expensive panel run.

**Non-Goals:**

- Changing the LIBERO image, task cases, native score, Agent Harness, manipulation stack, or Evo.
- Publishing images to a registry or adding CI deployment.
- Supporting Docker-in-Docker, privileged containers, remote multi-host Podman, or concurrent GPU experiments.

## Decisions

### Use an outer runner with sibling simulator containers

The host launcher mounts the rootless Podman socket and sets `CONTAINER_HOST` in the outer runner. The existing `LiberoPodmanContainer` therefore creates siblings in the host service. Host networking lets the outer runner reach the siblings' loopback-published RPC ports. This avoids a privileged nested daemon and keeps the simulator lifecycle unchanged.

### Run by the image ID emitted by Podman

The launcher builds locally with an IID file and runs the resulting `sha256:` identifier, not a mutable tag. The recipe pins the Python and Node base image digests, uses `uv sync --frozen` with only the agents, EdgeTAM/perception, manipulation, CUDA, and GraspGenX extras required by this evaluation, and uses `npm ci`. No registry publication is required.

### Preserve daemon-visible paths explicitly

The output directory, Evo result/traces directories, and DimOS cache are bind-mounted at the same absolute paths inside the outer runner. Paths passed from the outer process to the host Podman daemon therefore remain valid. A separate host scratch directory is mounted at short `/runner-tmp` and exported as `TMPDIR`, `TMP`, and `TEMP` to prevent capacity and Unix socket length failures.

### Forward an allowlist of environment variables

Only OpenAI/Hugging Face credentials and the three Evo result variables are forwarded. The launcher generates a fresh NVIDIA CDI spec inside the per-run scratch directory and supplies that directory to the outer `podman run`, so stale system CDI files cannot affect the run and no host configuration is changed. The Podman socket, host network, and NVIDIA CDI device are explicit command arguments. The launcher never copies the full host environment into the runner.

## Risks / Trade-offs

- **Host Podman socket grants container-management authority** → Mount only the rootless user's socket and document that the image is locally built from the current trusted checkout.
- **Identical absolute mounts expose selected host directories** → Mount only cache/output/result paths, reject broad filesystem roots, and keep the source baked into the image.
- **Large locked image build** → Keep simulator dependencies in the existing separate image and rely on local layer caching.
- **NVIDIA Container Toolkit or the Podman socket can be absent** → Fail in the cheap `check` mode with an actionable error before running the panel; keep generated CDI data scoped to run scratch.

## Migration Plan

Add the container launcher alongside the native autoresearch command. Build and check it locally, then use it for the upstream baseline. Rollback is removal of this additive layer; the native evaluator remains unchanged.

## Open Questions

None.
