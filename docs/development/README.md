# Development Guide

1. [How to setup your system](#1-setup) (pick one: system install, nix flake + direnv, pure nix flake)
2. [How to hack on DimOS](#2-how-to-hack-on-dimos) (which files to edit, debugging help, etc)
3. [How to make a PR](#3-how-to-make-a-pr) (our expectations for a PR)

<br>

# 1. Setup

All the setup options are for your convenience. If you can get DimOS running on TempleOS with a package manager you wrote yourself, all the power to you.

---

## Setup Option A: System Install

### Why pick this option? (pros/cons/when-to-use)

* Downside: mutates your global system, causing (and receiving) side effects causes it to be unreliable
* Upside: Often good for a quick hack or exploring
* Upside: Sometimes easier for CUDA/GPU acceleration
* Use when: you understand system package management (arch linux user) or you don't care about making changes to your system

### How to setup DimOS

```bash
# System dependencies

# On Ubuntu 22.04 or 24.04
if [ "$OSTYPE" = "linux-gnu" ]; then
    sudo apt-get update
    sudo apt-get install -y curl g++ portaudio19-dev git-lfs libturbojpeg python3-dev pre-commit
# On macOS (12.6 or newer)
elif [ "$(uname)" = "Darwin" ]; then
    # install homebrew
    /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
    # install dependencies
    brew install gnu-sed gcc portaudio git-lfs libjpeg-turbo python pre-commit
fi

# install uv for python
curl -LsSf https://astral.sh/uv/install.sh | sh && export PATH="$HOME/.local/bin:$PATH"

# this allows getting large files on-demand
export GIT_LFS_SKIP_SMUDGE=1
git clone -b dev https://github.com/dimensionalOS/dimos.git
cd dimos


# create & activate a virtualenv (needed for dimos)
uv venv && . .venv/bin/activate

# install dimos's python package with everything enabled
uv pip install -e '.[base,dev,manipulation,misc,unitree,drone]'

# setup pre-commit
pre-commit install

# test the install (takes about 3 minutes)
uv run pytest dimos
```

Note, a few dependencies do not have PyPI packages and need to be installed from their Git repositories. These are only required for specific features:

- **CLIP** and **detectron2**: Required for the Detic open-vocabulary object detector
- **contact_graspnet_pytorch**: Required for robotic grasp prediction

You can install them with:

```bash
uv add git+https://github.com/openai/CLIP.git
uv add git+https://github.com/dimensionalOS/contact_graspnet_pytorch.git
uv add git+https://github.com/facebookresearch/detectron2.git
```

<!-- Enable this option once the dockerfile (ghcr.io/dimensionalos/ros-python:dev) is public and debugged! -->
<!-- ## Setup Option B: Dev Containers (Recommended)

### Why pick this option? (pros/cons/when-to-use)

* Upside: Reliable and consistent across OS's
* Upside: Unified formatting, linting and type-checking.
* Upside: Other than Docker, it won't touch your operating system (no side effects)
* Downside: It runs in a VM: slower, issues with GPU/CUDA, issues with hardware access like Webcam access, Networking, etc
* Upside: Your IDE-integrated vibe coding agent will "just work"
* Use when: You're not sure what option to pick

### Quickstart

First [install Docker](https://docs.docker.com/get-started/get-docker/) if you haven't already.

Install the *Dev Containers* plug-in for VS Code, Cursor, or your IDE of choice. Clone the repo, open it in your IDE, and the IDE should prompt you to open in using a Dev Container.

### Don't like IDE's? Use devcontainer CLI directly

Terminal within your IDE should use devcontainer transparently given you installed the plugin, but in case you want to run our shell without an IDE, you can use `./bin/dev`
(it depends on npm/node being installed)

<details>
<summary>Click to see how to use it in the command line</summary>

```sh
./bin/dev
devcontainer CLI (https://github.com/devcontainers/cli) not found. Install into repo root? (y/n): y

added 1 package, and audited 2 packages in 8s
found 0 vulnerabilities

[1 ms] @devcontainers/cli 0.76.0. Node.js v20.19.0. linux 6.12.27-amd64 x64.
[4838 ms] Start: Run: docker start f0355b6574d9bd277d6eb613e1dc32e3bc18e7493e5b170e335d0e403578bcdb
[5299 ms] f0355b6574d9bd277d6eb613e1dc32e3bc18e7493e5b170e335d0e403578bcdb
{"outcome":"success","containerId":"f0355b6574d9bd277d6eb613e1dc32e3bc18e7493e5b170e335d0e403578bcdb","remoteUser":"root","remoteWorkspaceFolder":"/workspaces/dimos"}

  ██████╗ ██╗███╗   ███╗███████╗███╗   ██╗███████╗██╗ ██████╗ ███╗   ██╗ █████╗ ██╗
  ██╔══██╗██║████╗ ████║██╔════╝████╗  ██║██╔════╝██║██╔═══██╗████╗  ██║██╔══██╗██║
  ██║  ██║██║██╔████╔██║█████╗  ██╔██╗ ██║███████╗██║██║   ██║██╔██╗ ██║███████║██║
  ██║  ██║██║██║╚██╔╝██║██╔══╝  ██║╚██╗██║╚════██║██║██║   ██║██║╚██╗██║██╔══██║██║
  ██████╔╝██║██║ ╚═╝ ██║███████╗██║ ╚████║███████║██║╚██████╔╝██║ ╚████║██║  ██║███████╗
  ╚═════╝ ╚═╝╚═╝     ╚═╝╚══════╝╚═╝  ╚═══╝╚══════╝╚═╝ ╚═════╝ ╚═╝  ╚═══╝╚═╝  ╚═╝╚══════╝

  v_unknown:unknown | Wed May 28 09:23:33 PM UTC 2025

root@dimos:/workspaces/dimos #
```

The script will:

* Offer to npm install `@devcontainers/cli` locally (if not available globally) on first run.
* Pull `ghcr.io/dimensionalos/dev:dev` if not present (external contributors: we plan to mirror to Docker Hub).

You’ll land in the workspace as **root** with all project tooling available.

</details> -->

## Setup Option B: Nix Flake + direnv

### Why pick this option? (pros/cons/when-to-use)

* Upside: Faster and more reliable than Dev Containers (no emulation)
* Upside: Nearly as isolated as Docker, but has full hardware access (CUDA, Webcam, networking)
* Downside: Not hard, but you need to install/understand [direnv](https://direnv.net/) (which you probably should do anyway)
* Downside: Nix is not user-friendly (IDE integration is not as good as Dev Containers)
* Use when: you need reliability and don't mind a one-time startup delay

### Quickstart

Install and activate [direnv](https://direnv.net/).

```sh
# Install Nix
curl --proto '=https' --tlsv1.2 -sSf -L https://install.determinate.systems/nix | sh -s -- install
. /nix/var/nix/profiles/default/etc/profile.d/nix-daemon.sh
# make sure flakes are enabled
mkdir -p "$HOME/.config/nix"; echo "experimental-features = nix-command flakes" >> "$HOME/.config/nix/nix.conf"

# this allows getting large files on-demand
export GIT_LFS_SKIP_SMUDGE=1
git clone -b dev https://github.com/dimensionalOS/dimos.git
cd dimos

# activate the nix .envrc
cp .envrc.nix .envrc
# this is going to take a while
direnv allow
direnv reload
direnv status

# create virtualenv (needed for dimos)
uv venv && . .venv/bin/activate
# install dimos's python package with everything enabled
uv pip install -e '.[base,dev,manipulation,misc,unitree,drone]'
# test the install (takes about 3 minutes)
uv run pytest dimos
```

## Setup Option C: Nix Flake - Isolated/Reliable

### Why pick this option? (pros/cons/when-to-use)

* Use when: you need absolute reliability (use this if you want it to work first try) and don't mind a startup delay
* Upside: Doesn't need direnv, and has most of the other benefits of Nix
* Downside: Have to manually enter the environment (like `./venv/bin/activate` but slower)
* Upside: If you're using a basic shell, you'll get a nicely customized shell
* Downside: If you have hyper-customized your shell (fish, riced zsh, etc), you'll have to deal with someone else's preferences
* Downside: Your vibe coding agent will basically be unable to run tests for you (they don't understand how to enter the environment)

### Quickstart

```sh
# Install Nix
curl --proto '=https' --tlsv1.2 -sSf -L https://install.determinate.systems/nix | sh -s -- install
# make sure flakes are enabled
mkdir -p "$HOME/.config/nix"; echo "experimental-features = nix-command flakes" >> "$HOME/.config/nix/nix.conf"

# this allows getting large files on-demand
export GIT_LFS_SKIP_SMUDGE=1
git clone -b dev https://github.com/dimensionalOS/dimos.git
cd dimos

# activate the nix development shell
nix develop '.#isolated'
```

Once inside the shell, run:

```sh
# create virtualenv (needed for dimos)
uv venv && . .venv/bin/activate
# install dimos's python package with everything enabled
uv pip install -e '.[base,dev,manipulation,misc,unitree,drone]'
# test the install (takes about 3 minutes)
uv run pytest dimos
```

<br>

# 2. How to Hack on DimOS

## Debugging

Enable maximum logging: by adding `DIMOS_LOG_LEVEL=DEBUG RERUN_SAVE=1` as a prefix to the command. For example:

```bash
DIMOS_LOG_LEVEL=DEBUG RERUN_SAVE=1 dimos run unitree-go2
```

This will save the rerun data to `rerun.json` in the current directory.

## Where is `<thing>` located? (Architecture)

<!-- * If you want to add a `dimos run <your_thing>` command see [dimos_run.md](/docs/development/dimos_run.md) -->
* If you want to add a `dimos run <your_thing>` command see [dimos_run.md](/dimos/robot/cli/README.md)
* If you want to add a camera driver see [depth_camera_integration.md](/docs/depth_camera_integration.md)
<!-- * For edits to manipulation see [manipulation.md](/docs/development/manipulation.md) and [manipulation base](/dimos/hardware/manipulators/base/component_based_architecture.md) -->
* For edits to manipulation see [manipulation.md](/dimos/hardware/manipulators/README.md) and [manipulation base](/dimos/hardware/manipulators/base/component_based_architecture.md)
* `dimos/core/`: Is where stuff like `Module`, `In`, `Out`, and `RPC` live.
* `dimos/robot/`: Robot-specific modules live here.
* `dimos/hardware/`: Are for sensors, end-effectors, and related individual hardware pieces.
* `dimos/msgs/`: If you're trying to find a type to send a type over a stream, look here.
* `dimos/dashboard/`: Contains code related to visualization.
* `dimos/protocol/`: Defines low level stuff for communication between modules.
* See `dimos/` for the remainder

## Testing

We use both pytest and manual testing.

```sh
pytest # run all tests at or below the current directory
```

### Testing Cheatsheet

| Action                      | Command                      |
| --------------------------- | ---------------------------- |
| Run tests in current path   | `pytest`                     |
| Filter tests by name        | `pytest -k "<pattern>"`      |
| Enable stdout in tests      | `pytest -s`                  |
| Run tagged tests            | `pytest -m <tag>`            |

We use tags for special tests, like `vis` or `tool` for things that aren't meant to be ran in CI and when casually developing, something that requires hardware or visual inspection (pointcloud merging vis etc)

You can enable a tag by selecting -m <tag_name> - these are configured in `./pyproject.toml`

<br>

# 3. How to Make a PR
- Open the PR against the `dev` branch (not `main`).
- **No matter what, provide a few-lines that, when run, let a reviewer test the feature you added** (assuming you changed functional python code).
- Less changed files = better.
- If you're writing documentation, see [writing docs](/docs/agents/docs/index.md) for how to write code blocks. <!-- THIS IS FOR THE (already finish) NEXT DOC PR: If you're writing documentation, see [writing docs](/docs/development/writing_docs.md) -->
- If you get mypy errors, please fix them. Don't just add # type: ignore. Please first understand why mypy is complaining and try to fix it. It's only okay to ignore if the issue cannot be fixed.
- If you made a change that is likely going to involve a debate, open the github UI and add a graphical comment on that code. Justify your choice and explain downsides of alternatives.
- We don't require 100% test coverage, but if you're making a PR of notable python changes you should probably either have unit tests or good reason why not (ex: visualization stuff is hard to test so we don't).
- Have the name of your PR start with `WIP:` if its not ready to merge but you want to show someone the changes.
- If you have large (>500kb) files, see [large file management](/docs/data.md) for how to store and load them (don't just commit them).
- So long as you don't disable pre-commit hooks the formatting, license headers, EOLs, LFS checks, etc will be handled automatically by [pre-commit](https://pre-commit.com). If something goes wrong with the hooks you can run the step manually with `pre-commit run --all-files`.
- If you're a new hire at DimOS:
    - Did we mention smaller PR's are better? Smaller PR's are better.
    - Only open a PR when you're okay with us spending AI tokens reviewing it (don't open a half-done PR and then fix it, wait till the code is mostly done).
    - If there are 3 highly-intertwined bugs, make 3 PRs, not 1 PR. Yes it is more dev work, but review time is the bottleneck (not dev time). One line PR's are the easiest thing to review.
    - When the AI (currently Greptile) comments on the code, respond. Sometimes Greptile is dumb as rocks but, as a reviewer, it's nice to see a finished conversation.
