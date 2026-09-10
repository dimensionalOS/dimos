# Test dimOS application setup

This guide tests the installer PR before it merges. The commands below use
`feat/dimup-release` for both the installer and SDK; nothing needs to be published
or merged to `main` first.

Automatic machine setup covers Ubuntu 22.04/24.04 x86_64 and Apple Silicon
macOS 14+ and uses administrator access to prepare host prerequisites. On other
platforms, including Arch Linux, setup prints manual prerequisite instructions
without changing the machine. Once those prerequisites are installed, use the same
`dimup init` command; application creation does not reject the host platform.
uv owns Python, apt/Homebrew install host libraries, and Cargo and Nix build
native modules. You do not need Python installed before starting.

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
application's SDK revision. Keep both overrides while testing this PR.

Rerun machine setup when prerequisites are missing:

```bash
dimup setup
```

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

## Activation and failures

Source `.dimos/activate.sh` in each new Bash or Zsh terminal. `deactivate` restores
the prior environment. For automatic activation, install direnv, configure its
shell hook, review `.envrc`, and run `direnv allow`. Activation never installs packages.

Initialization shows each stage with its elapsed time and streams dependency
installation output to the terminal. The full output is also saved in
`.dimos/setup.log`. Redirected output uses plain lines; set `NO_COLOR=1` to disable
terminal color. On failure, it prints the failed command
and `.dimos/setup.log` location and keeps the project directory. Fix the reported
problem, then remove that directory or choose another empty directory to retry.
`dimos doctor` checks the application without modifying it.

Jetson provisioning, robot-host deployment, and contributor checkout setup are
separate workflows. The internal native replay test is not part of application creation.

## Share test results

Include your OS and architecture, the SDK commit recorded in `pyproject.toml`,
whether creation/test/doctor/edit/clone passed, and the failed stage or relevant
`.dimos/setup.log` excerpt if something went wrong.

After these changes merge, the normal installation guide can use the `main`
bootstrap URL and omit the installer and SDK ref overrides. Until then, use the
PR commands above.
