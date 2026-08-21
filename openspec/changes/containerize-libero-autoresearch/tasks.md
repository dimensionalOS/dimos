## 1. Build the Locked Runner Image

- [x] 1.1 Add a digest-pinned multi-stage container recipe that installs the frozen DimOS environment and locked Pi extension.
- [x] 1.2 Update the build context exclusions so only required Pi package sources are included.

## 2. Add the Host Launcher

- [x] 2.1 Build the existing simulator and outer image locally and capture the outer image ID.
- [x] 2.2 Construct the rootless socket, host network, NVIDIA CDI, identical path mounts, short scratch mount, and environment allowlist.
- [x] 2.3 Add benchmark and cheap check modes with actionable preflight failures.

## 3. Verify and Document

- [x] 3.1 Add unit tests for immutable image execution, mount topology, environment filtering, unsafe mount rejection, and commands.
- [x] 3.2 Document build, check, benchmark, socket, GPU, cache, and output usage.
- [x] 3.3 Run focused formatting, lint, unit tests, and an outer-container check.
