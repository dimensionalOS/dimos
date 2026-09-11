# Ubuntu installation

Use the official installer on Ubuntu 22.04/24.04, on x86_64 or ARM64:

```sh skip
curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/main/scripts/install.sh | bash
```

Choose system packages for the CI-tested path. The installer sets up apt dependencies, uv, Python 3.12, and dimOS, then verifies the CLI and native libraries.

Choose **library** for the published package or **dev** for a source checkout. For example:

```sh skip
curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/main/scripts/install.sh | bash -s -- --mode dev --no-nix --project-dir ./dimos
cd dimos
source .venv/bin/activate
```

An existing checkout is reused without pulling or switching branches. Developer mode includes test and lint dependencies; see [testing](/docs/development/testing.md) for additional groups.

Both modes pass CPU installation CI on Ubuntu 22.04/24.04 and x86_64/ARM64. Linux ARM64 excludes the unsupported `scene` extra. Jetson CUDA setup is not supported.

See [installer options and local testing](/docs/installation/index.md).
