---
name: setup-dimos
description: Install and verify a DimOS environment for navigation, manipulation, or both. Use for a new project or an existing contributor checkout, including unattended setup. Hardware commissioning and blueprint execution are separate tasks.
---

# Set up DimOS

Use the repository installer to provision system packages, Python, and a project
environment. Finish with verified dependencies and concrete next commands.

## Determine the installation

- Inspect the OS/architecture, available GPU, intended directory, and any existing
  checkout or `.venv`. Do not modify another project or replace its environment.
- Infer capabilities from the user's task: `navigation` for Unitree, mapping,
  simulation, and MAVLink; `manipulation` for arm control/planning; or
  `navigation,manipulation` for both. Both include agents, perception, visualization,
  web interfaces, and simulation. Ask if intent is missing; do not silently select both.
- Use `dev` for work on DimOS itself or an existing DimOS checkout. It includes
  test/lint dependencies. Use `library` for applications using the published package.
  Resolve the destination from the user's request or clarify it before installing.
- Read the installer's `--help` for the version being used. For an existing checkout,
  use its installer. Before cloning, download `scripts/install.sh` from the same
  DimOS revision as these instructions (normally `main`). No skill installation is
  required to read and follow this file.

## Run the installer

Inside an existing checkout, for example:

```bash
bash scripts/install.sh --non-interactive --mode dev --project-dir . \
  --capabilities navigation
```

For a new library project, download and inspect the script before running it:

```bash
curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/main/scripts/install.sh \
  -o /tmp/dimos-install.sh
bash /tmp/dimos-install.sh --help
bash /tmp/dimos-install.sh --non-interactive --mode library \
  --project-dir ./dimos-app --capabilities manipulation
```

Replace the example choices with those established above. To create a new
contributor checkout, use `--mode dev --project-dir ./dimos`; use `--branch` only
when a particular branch is requested. Existing checkouts keep their current branch.

- Always supply mode, directory, and capabilities explicitly. Non-interactive runs
  reject missing choices before changing the host. Add `--dry-run` when a preview
  helps inspect the proposed work; it requires the same explicit choices.
- Interactive installation offers native dependencies or Nix before confirmation;
  unattended installation defaults to apt on Ubuntu/WSL, Homebrew on macOS, and Nix
  on other Linux. Honor the user's setup preference with `--use-nix` or `--no-nix`;
  both bypass the question. `--no-nix` on other Linux assumes native prerequisites
  are already installed. NixOS uses Nix by default. Existing Nix needs flakes enabled.
- Let the installer detect CUDA on supported Linux x86_64 hosts. Use `--no-cuda`
  when the user requests CPU execution or GPU access is unavailable. Jetson CUDA
  setup is outside this installer.
- Required system packages and any package-manager bootstrap are part of setup.
  Respect existing user authorization and tool permissions. Unattended privilege
  failures are blockers to report with the requested prerequisite; never script
  password entry or loop on sudo failures. The user can provision prerequisites or
  authenticate in a terminal and rerun where that authorization applies.
- Persistent LCM buffer tuning requires `--configure-network`; use it only when
  requested or established as necessary and authorized. On NixOS use the documented
  `networking.kernel.sysctl` configuration instead.

## Verify and report

The installer runs bounded CLI, native-library, capability dependency, and PyTorch
checks. Exit zero after verification means the environment is ready; a dry run is
only a preview. On failure, identify the failed step and resolve its cause before
retrying. Do not clear an existing environment, change branches, or broaden the
capability selection as a repair shortcut.

Use the printed activation instructions. In Nix environments, enter `nix develop`
first. In a contributor checkout, `uv run --no-sync` preserves the installed extras;
plain `uv run` can resync a different environment. Use the project `.venv` explicitly
when invoking commands without activation.

Report the absolute directory, mode, capabilities, backend, verification results,
and shell-quoted activation and next commands. Explain any remaining prerequisites
relevant to the user's intended workflow. Do not claim robot readiness or complete
GPU workload support from dependency checks alone.

For next steps, use the README examples: `dimos --replay run unitree-go2` for
navigation, with the normal Rerun viewer, and `dimos run keyboard-teleop-xarm7` for
manipulation. Show both when both capabilities were selected, as separate examples.
Replay may initially show a black viewer while its data downloads. Keyboard teleop
uses mock hardware when no xArm address is configured; direct the user to its printed
visualization URL. These are suggestions to run after setup, not verification steps.

Do not launch blueprints or download models as part of environment verification.
Even importing some manipulation blueprints resolves remote robot assets. G1 DDS,
Habitat, RealSense, vendor drivers, credentials, and model assets require subsequent
workflow-specific setup. CPU environments cannot run every perception backend.

See the [installation guide](https://github.com/dimensionalOS/dimos/blob/main/docs/installation/index.md)
for platform limitations and the
[package extras](https://github.com/dimensionalOS/dimos/blob/main/docs/requirements.md#dependency-tiers)
for advanced direct `uv`/`pip` installation.
