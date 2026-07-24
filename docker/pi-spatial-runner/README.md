# Dimos spatial runner image

The build creates a deterministic source archive on the host, then builds the
native wheel offline in a dedicated builder derived from the same immutable
Bookworm image as the runtime. The runtime context contains only the
Containerfile, locked export, real wheel filename, and manifest.

## Local smoke

```bash
BASE=docker.io/library/python@sha256:72d3d75f2639ab82b34b29390ad3d6e0827c775befee94edda8e9976818f488d
python docker/pi-spatial-runner/build.py --base-image "$BASE" \
  --tag dimos-spatial-runner-smoke:local --allow-dirty
```

`--allow-dirty` bootstraps a local, nonpublishable builder and records its
local image ID. It never makes that ID a registry digest, and the automatically
bootstrapped tag is removed in a `finally` cleanup path. The offline builder
uses `--network=none`, an inspected sdist at `/input:ro`, `/out:rw`, and an
exec-capable `/build` tmpfs.

## Production

Use a clean checkout, an immutable runtime base, and a separately published
digest-pinned builder made from `Builder.Containerfile`:

```bash
python docker/pi-spatial-runner/build.py --base-image python:3.12-slim-bookworm@sha256:<digest> \
  --builder-image registry.example/dimos-spatial-builder@sha256:<digest> \
  --tag registry.example/dimos-spatial-runner:0.0.13
```

The builder label must declare the exact runtime base digest. The build rejects
host/Linux or pure wheel tags, cross-architecture wheels, and ABI requirements
above GLIBC 2.36. Runtime installation remains hash-checked and `--no-deps`;
the locked core dependency set includes Open3D's `ipywidgets` requirement.

Local `--allow-dirty` builder bootstrap is explicitly nonpublishable: its apt
inputs are mutable Debian repository state and its tag is ephemeral. Production
builders must be built in a controlled/snapshot workflow and published by
immutable digest; production builds require that named digest.

Production source packaging also requires a pre-provisioned `uv` executable at
exactly 0.9.17. The `uvx --from uv==0.9.17` fallback is restricted to dirty,
nonpublishable local smoke builds and is not an artifact-reproducibility claim.

The manifest keeps HEAD revision/tree identity separate from the optional
dirty `working_tree_identity` diagnostic (tracked diff only). The authoritative
packaged identity is the inspected sdist inventory hash plus sdist byte hash;
the dirty diagnostic is never treated as packaged source identity.

The default tests do not invoke Podman or network. Opt-in integration uses
`DIMOS_SPATIAL_RUNNER_INTEGRATION=1 uv run pytest docker/pi-spatial-runner/test_integration.py`.
