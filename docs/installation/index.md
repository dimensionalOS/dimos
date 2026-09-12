# Installation

The recommended way to install dimOS is the guided `install.sh` script. It installs system dependencies, uv, Python, and dimOS into a project virtual environment (or a source checkout for contributors). Recent clean Ubuntu CI installs took about 2–4 minutes (allow longer on a laptop, slower connection, or first Homebrew/Nix setup). Run it from a terminal:

```sh skip
curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/main/scripts/install.sh | bash
```

The installer asks for a mode and project directory, sets up system dependencies and uv, installs Python 3.12 and dimOS, and verifies the CLI and native libraries. It can also configure LCM networking and run an optional replay check. Follow the printed activation instructions when it finishes.

| Platform | Installation path | Validation |
| --- | --- | --- |
| [Ubuntu 22.04/24.04](/docs/installation/ubuntu.md), x86_64/ARM64 | apt | Both modes pass CPU installation CI |
| [macOS](/docs/installation/osx.md), Apple Silicon | Homebrew | CI paused; local testing needed |
| [NixOS / other Linux](/docs/installation/nix.md), including Arch | Nix | Not covered by installation CI |

Linux ARM64 excludes `scene` because `usd-core` has no wheel. CUDA extras require Linux x86_64; Jetson CUDA setup is not supported. Installation checks do not qualify robot hardware or GPU workloads.

## Choose a mode

- **Library (recommended)** installs the published package in a project virtual environment.
- **Developer** clones `main` and installs the checkout with test and lint dependencies. An existing checkout is reused without pulling or switching branches.

Install a CPU library environment without prompts or replay:

```sh skip
curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/main/scripts/install.sh | bash -s -- \
  --mode library --project-dir ./dimos-app \
  --non-interactive --no-nix --no-cuda --no-sysctl --skip-tests
cd dimos-app
source .venv/bin/activate
uv run dimos --help
```

Use `--mode dev --project-dir ./dimos` for a source checkout. Use `--extras base,unitree` to select capabilities (developer mode defaults to `all` with platform exclusions). See [dependency tiers](/docs/requirements.md#dependency-tiers).

`--skip-tests` skips replay only (installation verification still runs). `--no-sysctl` skips network tuning. These commands can install system packages on your host.

For all options:

```sh skip
curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/main/scripts/install.sh | bash -s -- --help
```
