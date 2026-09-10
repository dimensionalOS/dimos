# Test dimOS development setup

This guide tests the installer PR before it merges. The commands below use
`feat/dimup-release` for both the installer and SDK; nothing needs to be published
or merged to `main` first.

Automatic machine setup covers Ubuntu 22.04/24.04 x86_64 and Apple Silicon
macOS 14+ and uses administrator access to prepare host prerequisites. On other
platforms, including Arch Linux, setup prints manual prerequisite instructions
without installing system packages. Setup also configures the shell setting described
below. Once prerequisites are installed, both `dimup init` and `dimup dev` work
without a host-platform gate.
uv owns Python, apt/Homebrew install host libraries, and Cargo and Nix build
native modules. You do not need Python installed before starting. Deno and generated frontend
assets are not required; the experimental web UI has a separate development workflow.

## Prepare your machine

```bash
curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/feat/dimup-release/installer/bootstrap.sh \
  | DIMUP_REF=feat/dimup-release bash
```

The bootstrap installs uv if needed, downloads the PR branch, installs its
standalone `dimup` tool, and runs machine setup. Follow its printed PATH command
before continuing. On macOS, install Xcode Command Line Tools if setup requests
them with `xcode-select --install`. On Arch, setup prints the prerequisites to
install manually; it does not install system packages or block application creation.

`DIMUP_REF` selects the installer source. The `--ref` below separately selects the
application's SDK revision or the contributor checkout revision. Keep both overrides
while testing this PR.

Rerun machine setup when prerequisites are missing:

```bash
dimup setup
```

Setup adds `GIT_LFS_SKIP_SMUDGE=1` to your configured Bash, Zsh, or Fish startup
files, including on manually prepared platforms. This prevents automatic LFS
file downloads in **all repositories** opened from future terminals. Explicit
`git lfs pull` still downloads files; dimOS can fetch datasets when needed.
Setup reports each configured file and the command to apply the setting in your
current terminal. Rerunning setup does not duplicate the setting. Other shells
receive manual instructions.

Choose a path:

| Goal | Command |
| --- | --- |
| Build your own application using the SDK | `dimup init my-robot --ref feat/dimup-release` |
| Edit dimOS itself and contribute changes | `dimup dev dimos --ref feat/dimup-release` |

## Create and run

```bash
dimup init my-robot --ref feat/dimup-release
cd my-robot
source .dimos/activate.sh
dimos run my-robot.demo
```

The destination must be new or empty. Initialization resolves the PR branch once
and pins that full SDK commit in the application. Expect live dependency output,
completed stages, and a `Ready · my-robot` summary before running the example.

The example should print `image 160x120`. Stop it with Ctrl-C.

## Check and change the application

```bash
pytest -q
dimos doctor
```

The generated test should pass, and doctor should report passing checks for the
application environment, SDK revision, blueprint, tools, and runtime libraries.

Change the image width from `160` to `192` in both `src/my_robot/demo.py` and
`tests/test_demo.py`, then run:

```bash
pytest -q
dimos run my-robot.demo
```

Expect `image 192x120`. Stop with Ctrl-C. To check dependency installation, run
`uv add humanize`, then rerun `pytest -q` and `dimos doctor`.

Each application installs its own `dimos`. `dimup` does not install a competing
global runtime. The generated application includes the full desktop SDK extras
except `dds` and `unitree-dds`; it does not include dimOS's repository development groups.

## Check a clone

Commit the application to your own repository. On another prepared machine, or
in a new directory on this machine, clone it and install from its lockfile:

```bash
git clone <application-repository> my-robot
cd my-robot
uv sync --locked
source .dimos/activate.sh
dimos run my-robot.demo
pytest
```

Commit application source, `pyproject.toml`, `uv.lock`, `.envrc`, and the `.dimos`
activation files. Installed environments and setup logs are ignored. Cloning
does not require rerunning initialization or locating the original dimup version.

## Work on dimOS itself

On a prepared machine, choose a new or empty directory:

```bash
dimup dev dimos --ref feat/dimup-release
cd dimos
source .dimos/activate.sh
dimos doctor
git switch -c feat/my-change
```

Expect `Ready · dimos`. The checkout retains Git history and the upstream
`origin` remote. A branch ref creates or selects a tracking branch; a tag or
commit leaves a detached checkout, so create your feature branch before committing.
Without `--ref`, the command selects `main`.

The command installs dimOS editable into this checkout's `.venv`, with managed
Python 3.12, desktop extras except `dds` and `unitree-dds`, and the repository's
test and lint tools. It uses the checked-in lockfile and installs the existing
pre-commit and commit-message hooks. Setup files and logs are ignored by Git;
`git status --short` should be empty when setup finishes.

Doctor should report `PASS  Editable DimOS checkout` along with passing environment,
tool, and runtime-library checks. Run a small repository test:

```bash
pytest --noconftest -o addopts= dimos/utils/test_sequential_ids.py
```

Edit a Python source file and rerun its test or blueprint. The next process uses
your edited source without reinstalling dimOS. Native modules build when needed.
This setup does not build the experimental web UI, download bulk datasets, or
install documentation/browser/self-hosted test groups.

For a PR, fork dimOS on GitHub when ready, add your fork as a remote, and push
your feature branch. Open the PR against upstream `main`; see
[Contributing](/CONTRIBUTING.md) for the review process. `dimup dev` does not
require GitHub login, create forks, or prepare existing checkouts.

## Activation and failures

Source `.dimos/activate.sh` in each new Bash or Zsh terminal. `deactivate` restores
the prior environment. For automatic activation, install direnv, configure its
shell hook, review `.envrc`, and run `direnv allow`. Fish users use this direnv
path. Activation never installs packages. These steps apply to both applications
and contributor checkouts.

Initialization shows each stage with its elapsed time and streams dependency
installation output to the terminal. The full output is also saved in
`.dimos/setup.log`. Redirected output uses plain lines; set `NO_COLOR=1` to disable
terminal color. On failure, it prints the failed command
and `.dimos/setup.log` location and keeps the project directory. Fix the reported
problem, then remove that directory or choose another empty directory to retry.
`dimos doctor` checks the application or contributor checkout without modifying it.

Jetson provisioning and robot-host deployment are separate workflows. The internal native replay test is not part of application creation.

## Share test results

Include your OS and architecture, the SDK commit recorded in `pyproject.toml`
(or `git rev-parse HEAD` for a contributor checkout),
whether creation/test/doctor/edit/clone passed, and the failed stage or relevant
`.dimos/setup.log` excerpt if something went wrong.

After these changes merge, the normal installation guide can use the `main`
bootstrap URL and omit the installer and SDK ref overrides. Until then, use the
PR commands above.
