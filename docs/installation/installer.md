# Create a DimOS application

The supported development platforms are Ubuntu 22.04/24.04 x86_64 and Apple
Silicon macOS 14+. Setup uses administrator access to prepare host prerequisites.
uv owns Python, apt/Homebrew install host libraries, and Cargo and Nix build
native modules. You do not need Python installed before starting.

## Prepare your machine

```bash
curl -fsSL https://github.com/dimensionalOS/dimos/releases/latest/download/dimup.sh | bash
```

The bootstrap installs persistent `dimup` and runs machine setup. Follow its PATH
instruction if the command is not yet available in your terminal. On macOS,
install Xcode Command Line Tools if setup requests them with `xcode-select --install`.

Rerun machine setup when prerequisites are missing:

```bash
dimup setup
```

## Create and run

```bash
dimup init my-robot
cd my-robot
source .dimos/activate.sh
dimos run my-robot.demo
```

The destination must be new or empty. Initialization resolves `main` once and
pins its full SDK commit. To select another revision, use
`dimup init my-robot --ref <branch-or-commit>`.

The example produces synthetic images and prints `image 160x120`. Stop with
Ctrl-C. Edit `src/my_robot/demo.py` and its test to change the image dimensions:

```bash
pytest
dimos run my-robot.demo
uv add <package>
dimos doctor
```

Each application installs its own `dimos`. `dimup` does not install a competing
global runtime. The generated application includes the full desktop SDK extras
except `dds` and `unitree-dds`; it does not include DimOS's repository development groups.

## Clone an application

Prepare the destination machine with the bootstrap, then:

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

Initialization reports named stages. On failure, it prints the failed command
and `.dimos/setup.log` location and keeps the project directory. Fix the reported
problem, then remove that directory or choose another empty directory to retry.
`dimos doctor` checks the application without modifying it.

Jetson provisioning, robot-host deployment, and contributor checkout setup are
separate workflows. The internal native replay test is not part of application creation.
