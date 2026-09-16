# Dependencies and Runtime Environments

You choose the robot stack you want to run. dimOS works out which Python
packages it needs and can install them in a separate environment for you.
For normal use, you can keep using `dimos run` without choosing packages by hand.

A **blueprint** is a named stack, such as `unitree-go2`, that combines robot
connection, visualization, and other modules. Different blueprints need
different software. Simulation needs a simulator; an agentic stack needs AI
libraries. Installing every optional package takes space and can introduce
conflicts. This system prepares packages for the stack and configuration you
actually chose.

## A few words you will see

| Word | Meaning |
|---|---|
| Dependency | Software a part of dimOS needs in order to work. |
| Extra | A named group of optional Python packages, such as `sim`, `agents`, or `web`. Installing `dimos[sim]` means installing dimOS with its simulation packages. |
| Environment | A Python installation with its own set of installed packages. Your active virtual environment is one example. |
| Managed environment | An environment dimOS creates and reuses for running a stack, with packages separate from your development environment. |
| Profile | The operating system, processor type, and CPU/CUDA choice used to select compatible packages. |

Extras still exist. The new part is that dimOS chooses them for built-in
blueprints. It installs whole extras, so this does not promise the smallest
possible set of individual packages.

## Getting started

These commands assume you already have a working `dimos` command. A core
installation (`pip install dimos` in a Python environment) includes the
dependency commands. Automatic preparation also needs **uv 0.9.25 or newer**.

If you are starting from a repository checkout, you can create and activate
an environment with the core packages:

```sh skip
uv sync --no-default-groups
. .venv/bin/activate
```

`--no-default-groups` skips the repository's default test dependencies. An
existing development installation is fine too; you do not need to replace it.

For example, to inspect and then run the Go2 stack in MuJoCo simulation:

```sh skip
dimos --simulation mujoco deps unitree-go2
dimos --simulation mujoco run unitree-go2
```

Invoked directly as shown, `deps` reports requirements and checks the existing
environment without preparing a managed environment or starting the stack.
If you prefix it with `uv run`, uv can install project dependencies first;
see below. On a supported host, the second command:

1. Checks the packages in the environment that launched `dimos`.
2. Uses it if the packages satisfy the requirements. Otherwise, prepares or
   reuses a managed environment, validates it for this stack and
   configuration, and continues there automatically.
3. Starts the stack.

The first preparation may download substantial packages and take time.
Later runs can reuse the result. You do not need to activate a managed
environment yourself, and preparation leaves your original environment's
packages alone.

### Using `uv run`

`uv run dimos ...` is still supported. In a checkout, uv normally updates the
project environment before starting dimOS, installing missing or changed
dependencies. This happens even when the dimOS command is only `deps`.
See [uv's automatic sync behavior](https://docs.astral.sh/uv/concepts/projects/sync/#automatic-lock-and-sync).

This repository includes the `tests` dependency group by default, so that
update can install much more than the selected blueprint needs. uv does not
use the blueprint name to choose packages.

Once your project environment has dimOS installed, use `--no-sync` to launch
it without that update:

```sh skip
uv run --no-sync dimos --simulation mujoco deps unitree-go2
uv run --no-sync dimos --simulation mujoco run unitree-go2
```

Alternatively, activate `.venv` and invoke `dimos` directly, as above. Use
ordinary `uv run` when you want uv to update the project environment first.

`--no-sync` applies to the outer uv command. dimOS's `run` command can still
prepare a managed environment if the blueprint needs missing packages.
Add `--environment current` to `dimos run` if you want it to require the
existing environment instead.

## Which command should I use?

| What you want to do | Command |
|---|---|
| See available stacks | `dimos list` |
| Find out what a stack needs | `dimos deps unitree-go2` |
| Start a stack | `dimos run unitree-go2` |
| Install its runtime packages ahead of time | `dimos prepare unitree-go2` |
| Investigate whether your environment can run it | `dimos doctor unitree-go2` |
| See environments dimOS has prepared | `dimos envs list` |

Use the **same configuration** when inspecting, preparing, diagnosing, and
running. For example, put `--simulation mujoco` before each command if you
intend to use MuJoCo. The commands read global flags, environment settings,
and the configuration file through the same parser. Simulation, the viewer,
and the relay can change the requirements.

## Understanding what a blueprint needs

```sh skip
dimos deps unitree-go2
dimos --simulation mujoco deps unitree-go2
dimos deps unitree-go2-agentic --why perception
```

With the default configuration, `unitree-go2` needs the `unitree` and `web`
extras. Enabling MuJoCo also adds `sim` and either `cpu` or `cuda` for its
inference backend. The agentic blueprint adds AI requirements, including
`perception`. This is why installing only the robot connection package is
not always enough for a whole stack.

| Report label | How to read it |
|---|---|
| `Profile` | The hardware/package choice detected for this machine, or the one you requested. |
| `Extras` | Optional package groups needed in addition to core dimOS. The lines underneath name the code areas that need them. |
| `Backend` | A library with alternative implementations, such as ONNX Runtime for CPU or CUDA inference. |
| `Native`, `System`, `Tools` | Additional compiled code, host-provided libraries, or command-line programs the stack needs. These may need separate setup. |
| `Checkout`, `Release` | Commands to install the extras yourself, either in a source checkout or an environment with a released dimOS package. |
| `Environment` | Whether the current environment has the required Python packages at acceptable versions. |

`--why perception` shows which imports make that extra necessary. It also
accepts a package or import name. Requirements added by configuration rules
may have no import chain to show. Add `--json` if you want to process the
report in a script.

## How dimOS determines the groups

The groups shown by `dimos deps` are installation **extras**, such as `unitree`,
`web`, and `sim`. The `tests` group that uv can install is a separate
development dependency group.

dimOS calculates the extras from declarations in the source files, checked
by reading the code without running the blueprint. The result is saved in
the [blueprint catalog](/dimos/deps/blueprint_catalog.json) and shipped with
dimOS.

The process works like this:

1. **Start at the blueprint's source file.** The built-in registry identifies
   that file. The scanner follows imports into other dimOS files that would
   be loaded with it, the implementations its registry selections name (an
   `adapter_type="xarm"` in the blueprint), and the subprocesses those files
   declare. It works at file level, rather than tracing every function call
   made by the running robot.
2. **Translate imports into package names.** A maintained mapping says, for
   example, that `unitree_webrtc_connect` comes from the
   `unitree-webrtc-connect` package. `pyproject.toml` declares which packages
   core and each extra provide directly. A package dimOS imports must be
   declared directly, even when another package would install it anyway;
   `uv.lock` is only used to explain how an undeclared package is present
   today. A package declared by core needs no extra.
3. **Read each file's declaration.** A file that needs an extra, a backend,
   a host-provided module, a tool or a native artifact says so with a
   literal `Requires(...)` next to its code, for example
   `REQUIRES = Requires(extras=("perception",))` in the YOLO detector. The
   requirements of a blueprint are the union of the declarations of the
   files it reaches. Nothing is inferred from where an import sits: moving an
   import from module scope into a constructor changes nothing.
4. **Audit the declarations.** The scanner checks every third-party import
   in every file, eager or lazy, against the file's declarations and core.
   An import a file has not declared fails the catalog check with the
   declaration to add. A file that imports something lazily on behalf of
   callers that declare it lists the name in `defers`.
5. **Record what configuration selects.** Implementations chosen by
   configuration live in registry manifests (`_registry.py` files): hardware
   adapters and control tasks, the Go2 connection selected by
   `--simulation`/`--replay`, and the simulation module manipulator stacks
   start. The catalog stores each implementation's requirements and which
   configuration field selects it.
6. **Save the result and select it at launch.** Planning reads the catalog,
   resolves the selections it can read from the request (global flags, and
   the `hardware`/`tasks` options of the control coordinator from the command
   line, the config file or the environment), and uses the hardware profile
   to choose CPU or CUDA backend extras. If you request several built-in
   blueprints together, their requirements are combined.

For a concrete example, `unitree-go2` imports the Go2 connection code, which
imports the shared Unitree connection code. That file declares the `unitree`
extra because it imports `unitree_webrtc_connect`. This is why the report
includes `unitree`. You can follow that path with
`uv run --no-sync dimos deps unitree-go2 --why unitree`.

### Is this exact? Can it get the answer wrong?

**It is repeatable, and complete for what the planner can read, but not
minimal.** The same sources, lockfile, and declarations produce the same
catalog, regardless of what the developer happens to have installed. A plan
is reported as *incomplete* when the request contains something the planner
cannot vouch for; an incomplete plan never switches environments.

| Situation | What can happen |
|---|---|
| Shared files or conditions the scanner cannot interpret | It can include requirements for features you will not use. For an unrecognized condition, it considers both branches. Whole extras also install more than just the individual imported package. |
| Switching configuration | Catalog variants add requirements to the default set; they do not subtract from it. Turning off a feature does not necessarily remove its dependency group. |
| A module option that selects an implementation | `hardware` and `tasks` on the control coordinator are read from the request. An unknown instance, an unknown adapter or task name, or a value that is not JSON makes the plan incomplete. Choices made through the Python API (`.transports(...)`, a `detector=` callable, an explicit `instance_name`) are not read. |
| An external blueprint | It publishes no dependency metadata, so the plan is incomplete; install its requirements yourself and use `--environment current`. |
| A lazy import of another dimOS file | The importing file must declare what that file needs, or select it through a registry manifest; the audit does not follow lazy first-party imports. |
| Platform differences | Coverage comes from direct declarations, which may carry a platform marker. `deps` and `doctor` report a declaration excluded on the current machine instead of assuming it is present. |
| An incorrect declaration or a stale local catalog | The calculated extras can disagree with the current code's needs. Regenerating the catalog catches stale output; a wrong declaration needs a developer to fix it. |

Some extra installations reflect real Python import requirements: a shared
file can import a library as soon as it is loaded, even when you never call
the function that uses that library. Removing that requirement may require
moving the optional feature into a separate file and declaring it there.

### What catches mistakes?

The catalog-generation checks fail for unknown import names, imports (eager
or lazy) that no declaration covers, declarations of unknown or aggregate
extras, selections that are not literals, and a committed catalog that
disagrees with generated output. Some known undeclared packages are warnings
rather than failures. The checks cannot see dependencies that only a
subprocess or a dynamic import by name reveals; declare those by hand.

`doctor` and preparation provide another check by inspecting the selected
environment and attempting blueprint imports and backend checks. Their
package checks use declared requirements; they do not independently verify
every dependency of every installed package. They also do not exercise every
skill or runtime branch. A missing package used only by a later action can
therefore survive both catalog checks and `doctor`.

Use `deps --why` to investigate a surprising group, and `doctor` with the same
configuration and environment you will run. A successful report is useful
evidence that setup is ready; running the features you actually need remains
the check for dependencies that appear only during use.

## Choosing where a stack runs

The default is `--environment auto`. Change it when you want explicit control:

| Option on `dimos run` | What happens |
|---|---|
| `--environment auto` | Use the current environment if its packages satisfy the plan; otherwise prepare or reuse a managed one. Only a complete plan switches; an incomplete one stays in the current environment and says why. |
| `--environment current` | Stay in the current environment. If required packages are missing or incompatible, stop and show installation instructions. |
| `--environment managed` | Use a managed environment even if the current one has the packages. Prepare it if necessary. |
| `--environment /path/to/venv` | Check and use the virtual environment at that path. It must already contain Python and dimOS. Replace the path with your own. |

For example, to keep a simulation in your development environment:

```sh skip
dimos --simulation mujoco run unitree-go2 --environment current
```

If this reports missing packages, run the `Checkout` or `Release` installation
command printed by `deps`, then try again. This is useful when you maintain
your own environment or are debugging with your own package versions.

Manual extras still work. `unitree` now supplies the Unitree connection SDK;
it does not pull in the full standard stack. `base` bundles `agents`, `web`,
`perception`, and `visualization`. Use the blueprint's `deps` report to choose
the complete set. The [extra list](/docs/requirements.md) explains the groups.

Automatic environment selection happens in `dimos run` for complete plans:
cataloged built-in blueprints and modules, with selections the planner can
read. For an incomplete plan, `auto` stays in the current environment and
prints why, `managed` refuses, and an explicit virtualenv path is used as
given. If you call the Python blueprint API directly, install its
requirements in the Python environment you use.

## Preparing before you need to run

Use `prepare` to do installation separately from starting the robot or
simulator. On the machine that will run the simulation:

```sh skip
dimos --simulation mujoco prepare unitree-go2
dimos --simulation mujoco doctor unitree-go2 --environment managed
dimos --simulation mujoco run unitree-go2 --environment managed --offline
```

`prepare` prints the environment's location. It installs packages, then
validates the environment for the requested stack: backends, host-provided
modules, tools and the blueprint import. It does not start the blueprint.
An environment whose packages match is reused, and each validation is
recorded under its `validations/` directory together with the stack, the
dependency-affecting configuration, the profile and the host it was made
for. A request with a shape that has not been validated yet, such as a
second blueprint sharing the same packages or a different `--simulation`
value, validates again before it runs. `doctor` performs fresh checks when
you want to see the current state.

Combining `--environment managed` with `--offline` makes the final command
require that prepared environment. If it is missing, startup fails instead
of preparing it. With `auto`, an already sufficient current environment can
still be used, even with `--offline`.

Offline mode also enables the package installer's and Hugging Face libraries'
offline settings. Preparation does **not** preload model weights or replay
data, and offline mode does not make network services work offline. Arrange
the assets and services your stack needs separately.

There is also `dimos prepare ... --offline`: it can attempt a new installation
from packages already in uv's cache, and fails if it needs a download.
An offline `run` never makes that attempt.

## Diagnosing a problem

```sh skip
dimos --simulation mujoco doctor unitree-go2
dimos --simulation mujoco doctor unitree-go2 --environment managed
```

The first checks the environment that launched `dimos`. The second checks
the matching managed environment, which must already be prepared. You can
also pass `--environment /path/to/venv` to inspect your own environment.
`doctor` diagnoses; it does not install or repair packages.

The launcher's package check is only a starting point. `doctor` goes further:
it checks for conflicting packages, selected native modules, host-provided
modules, required tools, CPU/CUDA backend availability, and whether the
blueprint imports. It does not start the robot or test a complete run.

| Finding | What to do next |
|---|---|
| Missing package or wrong version | Let `run` use a managed environment, or follow the printed installation command in the environment you maintain. |
| CUDA backend fails | Check the machine's NVIDIA setup, or select the CPU profile if that suits your workload. |
| Missing native module, host library, or executable | Follow the relevant feature's setup instructions. Python extras alone may not provide it. |
| Managed environment is not prepared | Run `prepare` with the same configuration and profile you intend to use. |
| Two packages provide the same library | Inspect the warning. For example, `onnxruntime` and `onnxruntime-gpu` share files and can overwrite one another. |

Each prerequisite is reported as `satisfied`, `missing` or `unchecked`. For
scripts, `doctor` returns failure for missing or incompatible packages and
for any prerequisite reported missing: a backend, a host-provided module, a
tool, an in-tree native module or a blueprint that does not import. Native
executables built separately are reported as `unchecked`, and conflicting
packages are warnings. Read those messages even when the command succeeds.

Managed CUDA preparation handles the ONNX Runtime overlap by reinstalling
the GPU provider after installation and checking it. Both package names can
still appear in a warning; check the backend result too.

## Hardware profiles

Normally, let dimOS detect the profile. The implemented choices are:

| Profile | Machine |
|---|---|
| `linux-x86_64-cpu` | Linux on an Intel/AMD processor, using CPU inference. |
| `linux-x86_64-cuda` | Linux on an Intel/AMD processor with NVIDIA CUDA. Detection requires driver support for CUDA 12 or newer. |
| `linux-aarch64-cpu` | Linux on a 64-bit ARM processor, using CPU inference. |
| `macos-arm64-cpu` | macOS on Apple Silicon, using CPU inference. |

You can pass `--profile` to `deps`, `doctor`, `prepare`, and `run`. For example,
on an Intel/AMD Linux machine:

```sh skip
dimos --simulation mujoco deps unitree-go2 --profile linux-x86_64-cpu
```

Use the same override when preparing and running. A profile must match the
current machine's operating system and processor type; it cannot prepare an
ARM environment on x86. The CPU profile does not guarantee small downloads:
on Linux x86_64, the checkout lock still selects PyTorch wheels that include
CUDA libraries.

Jetson has no automatically selected managed profile. You can explicitly
choose `linux-aarch64-cpu` for CPU packages or maintain the environment
yourself. Intel macOS has no matching profile. On an unsupported host,
`run` in `auto` mode warns and continues in the current environment.
Preparation needs a supported, matching profile.

An explicit override allows preparation to finish with a warning when only
backend checks fail. This is useful when preparing without the accelerator
available, but does not establish that it will work. Run `doctor` on the
machine where you will use the stack.

## Reusing and cleaning up environments

Managed environments normally live in `~/.local/share/dimos/envs/` (under
`XDG_DATA_HOME` if set). `dimos envs list` shows their names, profiles, extras,
source, and whether a dimOS process is using or preparing them. `dimos status`
shows the running stack's Python environment.

Reuse depends on the profile, Python major/minor version, extras, and source.
For a checkout, that includes its directory and the contents of
`pyproject.toml` and `uv.lock`; for a release, the tested lock it ships.
Blueprints with matching inputs can share an environment. A dependency change
creates a new one instead of updating packages under an existing run.

Ordinary source edits still apply to a checkout's editable installation;
a managed environment is not a frozen copy of your code.

Every environment has two lock files in `envs/.locks/`. A run holds the
environment's use lease for as long as it lives, including a stack started
with `--daemon` or restarted with `dimos restart`. Preparation holds the
environment's prepare lock: a second `dimos run` or `dimos prepare` for the
same environment waits for the first to finish, while environments with
different names prepare in parallel. The lock files are empty and stay behind
after an environment is removed.

| Cleanup command | What it removes |
|---|---|
| `dimos envs remove NAME` | One environment; replace `NAME` with a name from `dimos envs list`. |
| `dimos envs prune` | Incomplete environments, environments whose checkout disappeared, and environments for the current checkout whose dependency files changed. |
| `dimos envs prune --all` | Every managed environment no dimOS process uses, including reusable ones. |

Removal is refused while a dimOS process uses or prepares the environment,
and pruning skips such environments and says so. Only names from
`dimos envs list` are accepted, never paths. The leases are file locks, which
are unreliable on network file systems such as NFS. `dimos cache clean` leaves
managed environments in place.

## What automatic setup covers

Preparation installs Python packages. It does not generally install operating
system libraries, NVIDIA drivers, host-provided vendor SDKs, or build the
separate Rust modules and executables used by some features. Deno is a
specific exception: online preparation can fetch it when the plan requires
it. Use `deps` and `doctor` to identify prerequisites.

External blueprints, such as `my-stack.go2`, have no dependency catalog entry,
so any request that names one has an incomplete plan: `dimos run` stays in
the current environment and says why, alone or in combination with built-in
blueprints. Install both sets of requirements yourself and use
`--environment current`, or pass the path of a virtualenv that has them.

From a checkout, preparation uses versions from `uv.lock` with
`uv sync --frozen`, without the default development groups. From an installed
release, it installs the same dimOS version with the package versions of that
release's tested lock, which the package ships as constraints together with
the repository's resolver settings. If those versions cannot be installed on
this machine, preparation fails instead of silently resolving something else;
upgrade dimOS or use `--environment current`. For an unpublished release,
`prepare` accepts `--find-links` with the directory containing your wheel;
this option applies only to installed releases.

## If you change a blueprint or add a dependency

You do not maintain an extras list on each blueprint. Declare new packages
in the appropriate extra in [pyproject.toml](/pyproject.toml) and update
`uv.lock`. Declare a package directly in the extra whose code imports it,
even when another package already installs it; the catalog check names the
extra and the package that currently pulls it in. Then declare the extra in
the file that imports the package, with a literal `Requires(...)` at module
scope (see [`dimos/deps/requires.py`](/dimos/deps/requires.py)); the check
names the declaration to add. A new import name may also need a mapping to
its package in [`dimos/deps/import_map.py`](/dimos/deps/import_map.py).

Keep optional heavy imports near the feature that uses them. A file reached
by every stack that declares an AI library makes every stack need it. A file
that imports lazily on behalf of callers which declare the requirement lists
the import name in `defers`. Configuration that picks an implementation goes
through a `_registry.py` manifest and a `Requires(selectors=...)` declaration,
never through a bare inline import; the audit does not follow lazy
first-party imports.

After adding or renaming a built-in blueprint, regenerate its registry first.
After dependency changes, regenerate the catalog and the tested lock that
releases ship:

```sh skip
pytest dimos/robot/test_all_blueprints_generation.py
pytest dimos/deps/test_catalog_generation.py
pytest dimos/deps/test_constraints_generation.py
```

Review and commit generated files with your change. These tests rewrite the
files locally and report failure if they left uncommitted changes; CI checks
that the committed files are current. Do not edit
`dimos/deps/blueprint_catalog.json` or `dimos/deps/constraints.txt` by hand.
