# Create a DimOS development workspace

The workspace creator uses DIOS installation logic to prepare an ordinary Python
project. Your source and dependency manifest belong to you. Daily work uses the
existing `dimos` CLI, Python, pytest, and uv. There is no separate `dim` CLI.

## Create, activate, develop

Until a verified release includes the initializer, build it from a checkout:

```bash
cargo build --locked --manifest-path installer/Cargo.toml
installer/target/debug/create-dimos my-robot --profile navigation
cd my-robot
source .dimos/activate.sh
dimos run my-robot.hello
pytest
dimos doctor
```

Creation prompts for the directory and profile if omitted in an interactive
terminal. Unattended creation requires both. `--dry-run` previews without writing.
The destination must be empty; the initializer never merges a scaffold into an
unrelated existing project.

| Profile | Robot dependency reference | Python extras |
| --- | --- | --- |
| navigation | unitree-go2 | unitree,cpu |
| manipulation | xarm7-planner-coordinator | manipulation,cpu |

Both profiles start with the same hardware-free `hello` module. Run it, edit
`src/my_robot/hello.py`, and rerun. Stop it with Ctrl-C. The profile prepares
robot dependencies; it does not make the starter connect a robot or fetch models.
Profiles cover these references, not every navigation/manipulation/simulation
workflow, and do not promise Jetson GPU inference support.

```text
my-robot/
├── pyproject.toml       package, dependencies, blueprint entry points
├── uv.lock
├── src/my_robot/hello.py
├── tests/test_hello.py
├── README.md
├── .envrc
└── .dimos/             profile, Pixi manifests, activation; local env is ignored
```

The user package is installed editable. Register additional blueprints using
[external package entry points](/docs/usage/blueprints.md#publishing-external-blueprints).
Use `uv add` to add dependencies and `uv sync` after editing package metadata.
Creation and restoration preserve subsequent source and manifest edits.

## Activation and optional direnv

`source .dimos/activate.sh` activates Pixi's native tools and libraries followed
by uv's `.venv`. It sets the shared-library and compiler environment too. Repeated
activation is safe, and `deactivate` restores the prior exported environment.
The script works from Bash or Zsh, including when sourced from another directory.
Activation never downloads dependencies or provisions Nix.

For automatic activation, the generated `.envrc` sources the same script.
Install direnv and configure its [shell hook](https://direnv.net/docs/hook.html),
review `.envrc`, then run `direnv allow`. Entering/leaving the directory loads and
unloads the environment. The initializer does not install direnv, edit shell
startup files, or authorize `.envrc`. Missing direnv is advisory; manual activation
always remains available. `.env` files are not automatically exported by this hook.

## Restore and contribute

Commit the package, lockfiles, `.envrc`, and `.dimos` configuration. Do not commit
`.venv`, `.dimos/.pixi`, `.dimos/tools.json`, or verification results. After cloning,
restore with the same release's initializer:

```bash
/path/to/create-dimos --restore /path/to/my-robot
```

Restore uses the recorded profile and existing locks, regenerates local tool
locations, and leaves source and manifests intact. Installed environments are not
portable; clone the source without environments and restore at the new location.
If dependencies need updating, use uv explicitly before locked restoration.

To work on DimOS itself rather than an SDK application:

```bash
/path/to/create-dimos /path/to/dimos --contributor --profile manipulation
```

An empty contributor directory is cloned from DimOS (`--branch` selects the
branch); an existing checkout is not reset or switched. Subsequent setup uses
`--restore`. Clone-time LFS downloads are deferred. `--wheel /absolute/file.whl`
lets SDK creation test an unpublished package; the path is recorded in the
workspace's uv sources and must remain accessible for restoration.

## Dependency ownership and doctor

```text
workspace creator
├── uv: Python 3.12, editable package, dependencies, .venv, uv.lock
├── Pixi: userspace tools/libraries and its lockfile; no Python interpreter
└── Nix: custom native build environment
```

Setup bootstraps uv/Pixi in the user's home and Nix through its official multiuser
installer. The host needs Bash, curl, CA certificates, archive utilities, and sudo
for initial Nix provisioning. Unattended Nix bootstrap requires passwordless sudo.
Host GPU drivers, JetPack, device permissions, networking, and kernel configuration
remain host responsibilities. No fallback package manager or reduced extras are used.

Targets are Ubuntu 22.04/24.04 x86_64 and AGX Orin with JetPack 6.2 / L4T 36.4.3.
Other Linux x86_64 distributions are marked unverified. A generic ARM machine
cannot establish Jetson support. The current references use wheel extensions;
a custom NativeModule addition fails verification until explicit setup-time build
preparation is provided.

`dimos doctor` checks configuration, interpreter, activation, tools/Nix, native
libraries, imports, a JPEG round trip, profile dependencies, editable SDK
registration, and optional direnv. It aggregates failures, returns nonzero for
required failures, and makes no repairs. Native checks run in bounded subprocesses.
It does not check robot networking or execute robot workflows.

Doctor is dispatched before importing the robotics CLI so missing native packages
can be diagnosed. With an installed but inactive environment, use
`.venv/bin/dimos doctor`. If Python or DimOS itself is missing, use the initializer
with `--restore`; an absent command cannot diagnose itself.

Only successful setup writes `.dimos/verified.json`; reruns invalidate previous
success first. Workflow verification is explicitly recorded as not run.

## Delivery and acceptance

Release CI builds native creator binaries on Ubuntu 22.04 x86_64 and ARM64 and
attaches them, checksums, and `create-dimos.sh` to the matching DimOS release. The
bootstrap downloads a temporary binary, verifies its checksum, runs it, and cleans
up. No permanent installer CLI or independent version stream is introduced.

Once a release contains these assets, select its version explicitly:

```bash
# Set DIMOS_VERSION to a verified release that includes creator assets.
curl -fsSL "https://github.com/dimensionalOS/dimos/releases/download/v${DIMOS_VERSION}/create-dimos.sh" \
  | bash -s -- my-robot --profile navigation
```

Existing releases do not gain these assets automatically. Public promotion stays
withdrawn until the Ubuntu matrix, dedicated Orin checks, and team robot-workflow
verification are accepted. This change provides delivery automation, not a new release.

`installer/run/acceptance <profile>` checks contributor installation, a normal
SDK wheel, starter execution/tests, edited source and dependencies, restoration,
and a fresh clone. CI also checks bootstrap failures and Bash/Zsh/direnv behavior.
The dedicated Orin job remains a trusted manual dispatch; it is pending provisioning.
The setup flow can be printed as Mermaid with `installer/run/flow_chart`.
