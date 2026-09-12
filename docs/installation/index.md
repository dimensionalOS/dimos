# Installation

The guided installer sets up system dependencies, uv, Python 3.12, and dimOS. Run it from a terminal:

```sh skip
curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/main/scripts/install.sh | bash
```

Choose an installation mode, destination, and one or both capabilities. The installer detects the package manager and CPU/CUDA backend, shows the work it will perform, and asks for one confirmation. It then installs and verifies the environment. Existing checkouts and virtual environments are reused without clearing them or switching branches.

## Choose capabilities

Nothing is preselected. The same choices apply to both installation modes:

| Capability | Included dependencies |
| --- | --- |
| **Navigation** | Unitree, mapping, MAVLink drones, agents, perception, visualization, web interfaces, and simulation |
| **Manipulation** | Arm control and planning, agents, perception, visualization, web interfaces, and simulation |
| **Both** | Both bundles; shared dependencies are installed once |

These bundles cover common workflows, not every hardware or model requirement. G1 DDS, Habitat, RealSense, and vendor drivers need specialized setup. Credentials and model/robot assets are configured when you run a workflow. CPU environments support control, planning, and CPU inference; some perception workloads require CUDA or MPS. See the [navigation](/docs/capabilities/navigation/index.md), [manipulation](/docs/capabilities/manipulation/index.md), [Go2](/docs/platforms/quadruped/go2/index.md), and [G1](/docs/platforms/humanoid/g1/index.md) guides.

## Choose a mode

- **Library** installs the published package in a project virtual environment.
- **Developer** clones `main`, or reuses an existing checkout, and installs its selected capabilities plus contributor test/lint groups. Those groups bring additional dependencies beyond the capability selection.

## Agent-assisted and unattended installation

Give your agent the [setup-dimos skill](https://github.com/dimensionalOS/dimos/blob/main/.agents/skills/setup-dimos/SKILL.md). It works before cloning and inside a checkout. A copyable prompt:

> Read https://raw.githubusercontent.com/dimensionalOS/dimos/main/.agents/skills/setup-dimos/SKILL.md and set up a DimOS navigation environment in ./dimos-app using library mode. Verify it and report the next commands.

Non-interactive runs require explicit mode, destination, and capabilities. For a CPU library environment:

```sh skip
curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/main/scripts/install.sh | bash -s -- \
  --non-interactive --mode library --project-dir ./dimos-app \
  --capabilities navigation --no-cuda
cd dimos-app
source .venv/bin/activate
dimos --help
```

For an existing contributor checkout:

```sh skip
bash scripts/install.sh --non-interactive --mode dev --project-dir . \
  --capabilities navigation,manipulation
```

Use `--capabilities manipulation` for arms only. Add `--dry-run` to preview an explicit command without changes. Required system packages install automatically; unattended runs fail with instructions if administrator access is unavailable, rather than prompting for a password. An administrator can provision prerequisites first, or you can authenticate with `sudo -v` in a terminal and rerun where that authorization applies.

Verification always checks the CLI, native libraries, selected capability dependencies, and PyTorch backend. It does not launch a blueprint or download model assets. The final summary includes activation instructions and next commands. In a checkout, use `uv run --no-sync` to preserve the selected environment; plain `uv run` may resync a different set of extras.

## Platform setup and overrides

| Platform | Default setup | Validation |
| --- | --- | --- |
| [Ubuntu 22.04/24.04](/docs/installation/ubuntu.md), x86_64/ARM64 | apt | Clean CPU installation CI, both modes |
| [macOS 14+](/docs/installation/osx.md), Apple Silicon | Homebrew | CI paused; local testing needed |
| [NixOS / other Linux](/docs/installation/nix.md), including Arch | Nix | Not covered by installation CI |

The summary identifies any Homebrew/Nix bootstrap. `--use-nix` selects Nix explicitly; `--no-nix` selects apt/Homebrew on supported platforms or preinstalled system dependencies on other Linux distributions. Existing Nix installations need flakes enabled. Library mode reuses existing flake files or downloads missing ones; developer mode uses the checkout's flake.

CUDA selection requires a detected NVIDIA GPU on Linux x86_64. `--no-cuda` selects CPU dependencies. Jetson CUDA setup is not supported. PyTorch verification does not qualify complete GPU workloads or promise GPU execution for every inference backend.

LCM network tuning is opt-in: add `--configure-network` to apply and persist Linux UDP buffer settings. On NixOS, set `networking.kernel.sysctl` entries `net.core.rmem_max` and `net.core.rmem_default` to `67108864` in `configuration.nix` instead. macOS does not support this flag.

The installer exposes capability choices; advanced direct `uv`/`pip` installations can still select [individual package extras](/docs/requirements.md#dependency-tiers).

For all flags and environment equivalents:

```sh skip
bash scripts/install.sh --help
```
