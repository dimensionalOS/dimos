# Installation

The recommended way to install dimOS is the guided `install.sh` script. It installs system dependencies, uv, Python, and dimOS into a project virtual environment (or a source checkout for contributors). Run it from a terminal:

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

## Install from a source checkout

`--mode dev` builds the checkout, including its C++ extension, so it needs `gcc`
or `clang`. Web bundles are large binary assets and are not present in a plain
git checkout. A checkout without them installs a UI-less wheel; set
`DIMOS_ALLOW_MISSING_COCKPIT=1` to allow that build instead of failing on the
missing cockpit and SDK assets:

```sh
DIMOS_ALLOW_MISSING_COCKPIT=1 pip install /path/to/dimos
```

The flag only skips the web UI bundle; messages, native code and the Python API
are installed as usual. Build the web bundles with `deno task build` in
`web/sdk` and `web/cockpit` when you need the local relay UI.

### Use the checkout in your own pip builds

The message build backend (`dimos.message_codegen.backend`) belongs to the
checkout and is not in the released `dimos` on PyPI yet. pip builds a project in
an isolated environment, so installing the checkout into your active environment
is not enough: `pip install .` still resolves build requirements from the index.
Select the checkout explicitly with a build constraint. The
`--build-constraint` option requires pip 25.3 or newer.

```sh
python -m pip --version   # 25.3 or newer

# 1. Build one wheel from the checkout (gcc or clang; same prerequisites as the
#    source install above, including DIMOS_ALLOW_MISSING_COCKPIT=1 when the
#    checkout has no built web bundles).
DIMOS_ALLOW_MISSING_COCKPIT=1 \
  python -m pip wheel --no-deps --wheel-dir /tmp/dimos-wheel /path/to/dimos
wheel=$(ls /tmp/dimos-wheel/dimos-*.whl | head -n 1)

# 2. Install that wheel in the environment you run your project from.
python -m pip install "$wheel"

# 3. Point a build constraint at the same wheel for the isolated build.
echo "dimos @ file://$wheel" > /tmp/dimos-build-constraint.txt

# 4. Install your project; `-e` for an editable install.
python -m pip install --build-constraint /tmp/dimos-build-constraint.txt .
python -m pip install --build-constraint /tmp/dimos-build-constraint.txt -e .
```

A build constraint only affects the isolated environment used to build the
project; it does not install anything, which is why step 2 installs the wheel in
your own environment separately. For a checkout you are working on, `python -m
pip install -e /path/to/dimos` replaces step 2. Once a release contains the
backend, plain `pip install .` works and the constraint can be dropped.

[the external message example](/examples/external-messages/README.md) is a
complete runnable project that follows this recipe, including the expected
output.
