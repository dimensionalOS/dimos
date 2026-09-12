# Installation

## Install with an agent (recommended)

Ask your coding agent to guide you through setup. Copy this prompt:

> Read https://raw.githubusercontent.com/dimensionalOS/dimos/main/.agents/skills/setup-dimos/SKILL.md and help me install DimOS. Ask me about the installation options before running setup, then verify the environment and show me how to get started.

The agent checks your host, asks about capabilities, installation mode, destination,
and native/Nix setup, then runs the installer with your choices and reports the
verification results. The [setup skill](https://github.com/dimensionalOS/dimos/blob/main/.agents/skills/setup-dimos/SKILL.md)
works before cloning or inside an existing checkout; no separate skill installation
is needed. Include any known preferences in your request to skip those questions.

## Install from a terminal

The guided installer sets up system dependencies, uv, Python 3.12, and dimOS. Run it from a terminal:

```sh skip
curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/main/scripts/install.sh | bash
```

Choose an installation mode, destination, and one or both capabilities. Choose native dependencies or Nix; the installer detects the CPU/CUDA backend, shows the work it will perform, and asks for one installation confirmation. It then installs and verifies the environment. Existing checkouts and virtual environments are reused without clearing them or switching branches.

## Choose capabilities

Use the arrow keys to move, Space to toggle navigation and manipulation independently, and Enter to confirm. Nothing is preselected. Interactive runs use Gum (downloaded temporarily if needed); if it cannot be loaded, a built-in menu provides the same arrow-key/Space selection. Unattended runs do not download the menu helper. The same choices apply to both installation modes:

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

Verification always checks the CLI, native libraries, selected capability dependencies, and PyTorch backend. It does not launch a blueprint or download model assets. The final summary includes activation instructions and examples from the [README](https://github.com/dimensionalOS/dimos#featured-runfiles): Go2 replay with its Rerun viewer for navigation, and xArm7 keyboard teleop for manipulation. Try the examples separately. Keyboard teleop uses mock hardware when no xArm address is configured; open its printed visualization URL. In a checkout, use `uv run --no-sync` to preserve the selected environment; plain `uv run` may resync a different set of extras.

## Platform setup and overrides

Interactive runs ask how to provide system dependencies before the installation summary. Ubuntu/WSL offer native apt packages (recommended) or Nix; macOS offers native Homebrew packages (recommended) or Nix. Other Linux distributions offer Nix (recommended) or dependencies you have already installed manually. NixOS uses Nix.

`--use-nix` and `--no-nix` bypass the question. Unattended runs use these platform defaults unless overridden:

| Platform | Default setup | Validation |
| --- | --- | --- |
| [Ubuntu 22.04/24.04](/docs/installation/ubuntu.md), x86_64/ARM64 | apt | Clean CPU installation CI, both modes |
| [macOS 14+](/docs/installation/osx.md), Apple Silicon | Homebrew | CI paused; local testing needed |
| [NixOS / other Linux](/docs/installation/nix.md), including Arch | Nix | Not covered by installation CI |

The summary identifies any Homebrew/Nix bootstrap. `--use-nix` selects Nix explicitly; `--no-nix` selects apt/Homebrew on supported platforms or preinstalled system dependencies on other Linux distributions. The installer enables `nix-command` and `flakes` for its Nix commands without changing your Nix configuration. Library mode reuses existing flake files or downloads missing ones; developer mode uses the checkout's flake.

CUDA selection requires a detected NVIDIA GPU on Linux x86_64. `--no-cuda` selects CPU dependencies. Jetson CUDA setup is not supported. PyTorch verification does not qualify complete GPU workloads or promise GPU execution for every inference backend.

LCM network tuning is opt-in: add `--configure-network` to apply and persist Linux UDP buffer settings. On NixOS, set `networking.kernel.sysctl` entries `net.core.rmem_max` and `net.core.rmem_default` to `67108864` in `configuration.nix` instead. macOS does not support this flag.

The installer exposes capability choices; advanced direct `uv`/`pip` installations can still select [individual package extras](/docs/requirements.md#dependency-tiers).

For all flags and environment equivalents:

```sh skip
bash scripts/install.sh --help
```
