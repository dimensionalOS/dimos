---
name: setup-dimos
description: Install and verify a DimOS environment for navigation, manipulation, or both. Use for a new project or an existing contributor checkout, including unattended setup. Hardware commissioning and blueprint execution are separate tasks.
---

# Set up DimOS

Use the repository installer to provision system packages, Python, and a project
environment. Finish with verified dependencies and concrete next commands.

## Guide the user through setup

Start with a read-only inspection of the OS/architecture, GPU, intended directory,
and any existing checkout or `.venv`. Then ask about the unresolved choices below,
in a short batch with suggested answers. Explain recommendations in everyday terms.
Do not silently infer a capability or installation mode from an ambiguous request.
Reuse choices the user already gave; an explicit unattended setup request does not
need another interview.

| Ask the user | Explain the choices |
| --- | --- |
| What do you want to build? | Navigation for mobile robots, mapping, simulation, and MAVLink; manipulation for arms and planning; both for both workflows. Both bundles include agents, perception, visualization, and simulation dependencies. |
| Use DimOS in an application, or develop DimOS itself? | Library uses the published package; developer uses a source checkout and adds test/lint tools. Recommend developer for an existing DimOS checkout. |
| Where should it be installed? | Suggest an absolute path, such as `./dimos-app` for an application or `./dimos` for a new checkout. Identify existing projects before reusing them. |
| Native dependencies or Nix? | Recommend apt on Ubuntu/WSL, Homebrew on macOS, and Nix on other Linux. Native setup on other Linux requires manually provisioned prerequisites. NixOS uses Nix. |

Mention the detected GPU and recommend automatic backend selection; offer CPU-only
when relevant. Explain that required system packages and a missing package manager
will be installed. Leave persistent LCM network tuning off unless requested or
established as necessary and authorized.

Once choices are resolved, show a compact summary with mode, absolute destination,
capabilities, native/Nix setup, and CPU/GPU preference. Continue under the user's
existing setup authorization; do not ask them to approve the same choices again.
The conversation supplies the interactivity: run the installer with explicit flags
and `--non-interactive` so the agent never needs to operate its terminal menus.

Read the installer's `--help` for the version being used. Inside a checkout, use its
installer. Before cloning, download `scripts/install.sh` from the same revision as
this skill (normally `main`). No skill installation is required to follow this file.

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
  are already installed. NixOS uses Nix by default. The installer enables the required Nix features for its commands.
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

Use the printed activation instructions. In Nix environments, enter `nix --extra-experimental-features "nix-command flakes" develop`
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
