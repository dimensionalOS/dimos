# Adding a hardware wizard

`dimos hardware <robot> setup` brings a robot up from the machine it runs on. This is how you add
one. Written for a coding agent and the FDE driving it; every signature below is pasted from the
code, so check it against the file when it looks stale.

Two files, always:

| File | Holds |
|---|---|
| `src/setup/<robot>.rs` | the stages — one pure function per stage, plus one `observe` that reads the machine |
| `src/hardware.rs` | the `Robot` variant, the preflight, the plan that lists the stages, the `installer.json` record |

Shared stages already exist: `setup/deps.rs` (system packages, uv), `setup/sysconfig.rs` (sysctl,
loopback multicast, memlock), `setup/jetson.rs` (nvpmodel, jetson_clocks), `setup/verify.rs` (the
critical checks). Compose those before writing anything new.

---

## 1. What a wizard is

A wizard never touches the machine. It reads probes, returns `Vec<Stage>`, and `plan::run` does the
rest — which is what makes `--dry-run` trustworthy and re-runs free.

```rust
// src/setup/<robot>.rs — pure over its inputs, one struct of everything it needed to look at
pub struct Observed { /* what is already installed, built, configured */ }
pub fn observe(home: &Path, venv_python: &Path, dir: &Path) -> Observed;   // the only I/O
pub fn <thing>_stage(home: &Path, obs: &Observed) -> Stage;                // one per stage
```

`observe` is read-only, so it runs under `--dry-run` too, and every command it runs goes through
`probe::capture(program, args, env, timeout_s)` so a wedged robot cannot hang it. Everything else
is a pure function of its result: same probes in, same plan out, and an empty `Stage.actions`
means "already there".

---

## 2. The contract

### `Action` — `src/plan.rs:29`

```rust
pub enum Action {
    /// argv spawned directly, never a shell; `sudo` routes through `Ctx.sudo`; killed at timeout_s.
    Run { argv: Vec<String>, sudo: bool, cwd: Option<PathBuf>, env: Vec<(String, String)>, timeout_s: u64 },
    /// Noop when the file already holds `contents`; with sudo, write $TMPDIR then `install -m`.
    WriteFile { path: PathBuf, mode: u32, contents: String, sudo: bool },
    /// A `# DIMOS-ADDED: <marker>` block in a per-user rc file; empty `lines` removes it.
    EnsureBlock { file: PathBuf, marker: String, lines: Vec<String> },
    Copy { from: PathBuf, to: PathBuf, mode: u32 },
    Rename { from: PathBuf, to: PathBuf },
    Remove { path: PathBuf, sudo: bool },
    VerifySha256 { file: PathBuf, sums_file: PathBuf },
}

impl Action {
    /// The one Run constructor; `run`, `sudo` and `run_in` are its borrowed spellings.
    pub fn run_owned(argv: Vec<String>, sudo: bool, cwd: Option<&Path>, env: &[(&str, &str)], timeout_s: u64) -> Action;
    pub fn run(argv: &[&str], timeout_s: u64) -> Action;
    pub fn sudo(argv: &[&str], timeout_s: u64) -> Action;
    /// A Run with a working directory and environment; never sudo, which would drop the env.
    pub fn run_in(argv: &[&str], cwd: Option<&Path>, env: &[(&str, &str)], timeout_s: u64) -> Action;
    pub fn display(&self) -> String;            // the `--dry-run` line and the consent prompt
    pub fn view(&self) -> state::ActionView<'_>; // the redacted shape the action log gets
}

pub fn owned(argv: &[&str]) -> Vec<String>;   // the one way to build an owned argv
pub fn text(path: &Path) -> String;           // the one spelling of a path as an argv word
```

A sudo action that needs an environment carries it in argv (`["env", "K=V", "cmd", ...]`) — see the
rules. A systemd unit is `sysconfig::unit_actions(name, contents)`: write as root, `daemon-reload`,
`enable --now`.

### `Stage` and `Plan` — `src/plan.rs:241`, `src/plan.rs:319`

```rust
pub struct Stage {
    pub name: &'static str,
    pub critical: bool,
    /// Needs `--yes` or one interactive confirm; a sudo action does not imply it, a package install does.
    pub consent: bool,
    /// Empty means the machine is already in the wanted state, reported as `Outcome::Already`.
    pub actions: Vec<Action>,
    /// argv that must exit 0 after the actions; the post-condition, not the verify stage.
    pub post: Option<Vec<String>>,
    /// A failure is one `!!` line carrying the child's last output line: never a stop, never a human.
    pub warn_only: bool,
}

impl Stage {
    pub fn new(name: &'static str, critical: bool) -> Stage;
    pub fn push(self, action: Action) -> Stage;
    pub fn run(self, argv: &[&str], timeout_s: u64) -> Stage;
    pub fn sudo(self, argv: &[&str], timeout_s: u64) -> Stage;
    pub fn post(self, argv: &[&str]) -> Stage;
    pub fn consent(self) -> Stage;
    pub fn warn_only(self) -> Stage;
    pub fn needs_sudo(&self) -> bool;
}

pub struct Plan { pub command: String, pub stages: Vec<Stage>, pub notes: Vec<String> }
```

A critical stage that fails, is declined, or cannot get root stops the plan. A `warn_only` stage
(the torch import, `apt-get update`) prints `!! <stage>: <the child's last line>` and the plan goes
on with exit 0, so write the check so its last line is the operator's fix. `notes` is the only way
a stage builder reaches the operator with something it did not plan — a warning, a manual step, a
reason a stage is empty. Push them in `<robot>_plan`, not in the stage.

### Probes — `src/probe.rs:123`

```rust
pub struct Probes {
    pub platform: Platform,   // os, arch, glibc, jetson, gpu, pkg, systemd, home, user, shell
    pub kernel: Kernel,       // sysctl, lo_multicast, multicast_route, memlock_conf_bytes,
                              // nvpmodel_maxn, sysctl_conf, enabled_units
    pub tools: Tools,         // uv, git, curl, nix, login_path_has_local_bin, dpkg_status, brew_list
    pub installed: Option<state::Installed>,
    /// The rc files the user's login shell reads, as they stand, so a block is planned once.
    pub rc: Vec<RcFile>,
    pub ifaces: Vec<(String, Ipv4Addr)>,
    pub current_exe: PathBuf,
}
impl Probes { pub fn detect(sysctl_keys: &[&str], home: &Path) -> Result<Probes>; }

/// The one bounded read-only spawn: trimmed stdout on exit 0, None on failure or at the deadline.
pub fn capture(program: &str, args: &[&str], env: &[(&str, &str)], timeout_s: u64) -> Option<String>;
```

`Probes::detect` runs once in `main`. If your robot needs a fact nobody probes yet, add a parse
function to `probe.rs` (pure, fixture-tested) and read the machine through `capture` in your
`observe` — never a `Command` inside a stage builder, never a spawn without a deadline.

### Verify — `src/setup/verify.rs:16`

```rust
pub enum Target { Host, G1 { cyclonedds_home: PathBuf, interface: String }, Jetson }
pub fn stages(target: &Target, venv: &Path, dir: &Path, blueprint: Option<&str>) -> Vec<Stage>;
pub fn is_check(name: &str) -> bool;   // `update --dry-run` reports these as not run, not pending
```

Every check is one `Action::Run` of `bash -lc '<venv>/bin/python -c "..."'`, so what it proves is
what a user's own login shell gets — the PATH and `CYCLONEDDS_HOME` rc blocks included. The G1
target ends with a live `rt/lowstate` read on `interface`, so a green verify means a robot answered.
Reuse a variant when the checks match; add one when your robot has its own stack to prove, and
give it a live check too: a plan that only proves imports is the "green install, dead robot" bug.

### Sudo — `src/sudo.rs:49`

```rust
pub enum Sudo { Root, Passwordless, Askpass(PathBuf), Stdin(Secret), Tty, Unavailable(String) }
impl Sudo {
    /// The argv to spawn plus the bytes to feed its stdin; the password is only ever in those bytes.
    pub fn wrap(&self, argv: &[String]) -> (Vec<String>, Option<Vec<u8>>);
    pub fn available(&self) -> bool;
    pub fn human_fix(&self) -> String;
}
```

You never call this. Set `sudo: true` on the action; `plan::run` wraps it and stops the stage with
an exit-2 `NeedsHuman` carrying `human_fix()` when root is not reachable.

### Output — `src/plan.rs:503`

```rust
pub mod say { pub fn info(msg: &str); pub fn ok(msg: &str); pub fn warn(msg: &str); pub fn fail(msg: &str) }
```

`-> ok !! xx` on stderr; colour only on the prefix, only on a TTY with `NO_COLOR` unset. No other
file prints or prompts.

---

## 3. The rules

- **A probe per stage.** A second run must plan nothing but the checks. If a stage cannot answer
  "is this already done?", it is not a stage yet.
- **`--dry-run` renders the whole plan and touches nothing.** Only `observe` may read, and only
  through `probe::capture`, which has a deadline.
- **A critical verify is last.** `verify::stages(...)` appended after everything, and the run fails
  when it fails — a green install with a dead robot is the bug this exists to stop.
- **sudo only through the action's `sudo: true`.** Never `sudo` inside an argv, never a sudo clone
  into `/opt`.
- **A sudo `Run` carries its env in argv** (`["env", "K=V", "cmd", ...]`) — sudo's `env_reset` drops
  `Action.env`, and `plan::sudo_env_violations` fails the run if you forget.
- **Secrets never reach argv, the plan text, or the action log.** Pass a path or an env key.
- **argv vectors, never shell strings.** The one exception is `bash -lc` in verify, where a login
  shell is the thing being tested.
- **Functions ≤25 lines; the plan builder reads as a list of calls.**
- **No `unwrap`/`expect` on an I/O path.** `?` with context that says got-vs-want or the next command.
- **Comments answer WHY, one line.** No TODO, no FIXME.

---

## 4. Registering it

Four edits, in this order.

**1. `src/cli.rs`** — a variant on `HardwareTarget`, beside the two that are already there:

```rust
pub enum HardwareTarget {
    G1 { #[command(subcommand)] verb: HardwareVerb },
    Jetson { #[command(subcommand)] verb: HardwareVerb },
    /// Your robot.
    Demo { #[command(subcommand)] verb: HardwareVerb },
    #[command(external_subcommand)]
    Other(Vec<OsString>),
}
```

Add the arm to `hardware_argv` too, so `dimos hardware demo calibrate` rebuilds the Python argv.

**2. `src/hardware.rs`** — a `Robot` variant, its `key` (which is also the `installer.json` hardware
key), an `owned` arm, a `run` arm, a preflight arm, and the plan:

```rust
use crate::setup::{demo, deps, g1, jetson, sysconfig, verify};

pub enum Robot { G1, Jetson, Demo }
// Robot::key   -> "demo"
// owned        -> HardwareTarget::Demo { verb: HardwareVerb::Setup(args) } => Some((Robot::Demo, args))
// ready        -> Robot::Demo => demo_ready(probes)
// run          -> Robot::Demo => demo_plan(args, probes, cfg, &installed)

pub fn demo_plan(args: &HardwareSetupArgs, probes: &Probes, cfg: &Platforms, installed: &Installed) -> Plan {
    let mut stages = vec![demo::marker_stage(&probes.platform.home)];
    stages.extend(checks(verify::Target::Host, installed, args));
    Plan { command: Robot::Demo.command(), stages, notes: notes(probes) }
}
```

`Robot` is matched exhaustively in `key`, `run`, `ready` and `setup_first`, so the compiler names
every place the new variant has to be handled. `run` answers exit 2 with `setup_first` when there
is no `installer.json`, before `preflight`. The G1 keeps its stage list in a `pub fn g1_stages`
because `update` rebuilds it from the `installer.json` record; do the same when `update` must be
able to repair your robot.

**3. `src/setup/mod.rs`** — `pub mod demo;`.

**4. `installer/platforms.toml`** — the system packages your extra needs, keyed by the
`[project.optional-dependencies]` name in `pyproject.toml` (`pkgs.rs` tests fail on any other key):

```toml
[extras.demo]
apt  = ["cmake"]   # why this package, on the same line
brew = ["cmake"]
```

`only_arch = ["x86_64"]` when pyproject gates the wheels; `preflight` turns that into the refusal
message the operator reads.

---

## 5. The tests it must add

Co-located in `#[cfg(test)] mod tests` at the bottom of the same file, one invariant per test, names
that state the invariant, fixture strings not real I/O.

In `src/setup/<robot>.rs`:

- `a_fresh_<robot>_plans_every_build_stage` — default `Observed`, assert the stage names in order.
- `a_configured_<robot>_plans_nothing` — the `Observed` a finished machine produces, assert the
  planned stage names are `[]` (`g1.rs` is the model).
- one test per non-obvious action: the exact argv, the file mode, the rendered unit text.

In `src/hardware.rs`:

- `<robot>_preflight_refuses_<the wrong machine>_naming_the_setup_command` — the error text is what
  the operator does next, so assert on it.
- `a_configured_<robot>_plans_nothing_but_the_checks` — planned names are the verify stages only.
- `<robot>_plan_ends_with_the_critical_verify` — name is `"verify"`, `critical` is true.
- `no_hardware_plan_asks_sudo_to_carry_an_environment_it_would_drop` already covers your plan once
  it is in the loop; add it to the array.

Fixtures: build `Platform`/`Kernel`/`Tools`/`RcFile` literals (copy them from `hardware.rs`'s test
module; a G1 fixture needs `ifaces` to hold its `--interface`). `state::TmpDir` is the throwaway
`HOME` when a test genuinely needs one — do not write a second one.

---

## 6. Worked example: `src/setup/jetson.rs`

The smallest complete wizard in the crate: 76 lines of runtime code, and every decision a bigger one
makes.

**`:1`** — one-line module docstring naming who calls it: `setup`, `hardware jetson setup`, and
`hardware g1 setup`, because the G1's onboard computer is an Orin NX.

**`:3-6`** — imports only from `plan`, `probe`, `state` and the shared `sysconfig`. A stage file
never imports `cli` or another robot's module.

**`:8-11`** — `STEP_TIMEOUT_S = 60` and `LD_PRELOAD_FIX`. Units in the name; a bare `60` in an
argv is unreviewable. The fix text lives here once and `verify.rs` imports it, so the note and the
check can never disagree.

**`:14`** — `pub fn stage(platform: &Platform, kernel: &Kernel) -> Stage`. Pure, two probe structs
in, one `Stage` out. There is no `home`, no `Ctx`, no `&mut` anything.

**`:15`** — `Stage::new("jetson perf", false)` — non-critical. Performance mode is a tuning step;
a machine without `nvpmodel` still runs DimOS, so it must not stop the run.

**`:16-18`** — the platform guard. Off a Jetson the stage is empty, which the runner reports as
`ok already`. This is why `setup` and `update` can call it unconditionally.

**`:19-22`** — the first probe-gated action. `needs_maxn` is `kernel.nvpmodel_maxn == Some(false)`,
so `None` (no `nvpmodel` binary) plans nothing rather than guessing. The one-line comment carries
the WHY that naming cannot: mode 0 is MAXN on every Orin SKU and it survives a reboot.

**`:23-25`** — the second, gated on `platform.systemd` as well: the unit is the persistence
mechanism, so without systemd there is nothing to install.

**`:30-43`** — `render_clocks_unit()` returns the unit text as a `String`, which makes it a pure
function a test asserts on directly. `jetson_clocks` resets every boot, so it runs from a oneshot
unit rather than once at install time.

**`:46-59`** — `static_tls_note` and `thermal_note`. Both return `Option<String>` and neither plans
an action: they are facts the operator needs that no command can fix. `hardware.rs::notes` collects
them into `Plan.notes`.

**`:61-70`** — the two predicates, each one expression. `clocks_unit_enabled` trims `.service`
because `probe::parse_unit_files` keeps the suffix `systemctl list-unit-files` prints.

**`:72-76`** — `install_clocks_unit` folds `sysconfig::unit_actions` into the stage: write the unit
as root, `daemon-reload`, `enable --now`. The same three actions the multicast unit uses, so a unit
is installed one way in the crate.

**`:78+`** — the tests. Two `nvpmodel -q` fixture strings (JetPack 6 MAXN, JetPack 5.1.1 at 15 W)
parsed by the same `probe::nvpmodel_is_maxn` the runtime uses, so the test and the binary cannot
disagree about what the output means.

Then read `src/setup/g1.rs` for the large case: an `observe` with six probes, a source build, a
`--no-deps -e` editable install, an rc block, and a `.env` merge that keeps every line it did not
write.

---

## 7. Runbook lines for `context.md`

Add these three, filled in, when the wizard lands:

```
# plan only, on any machine — must exit 0 and print every stage
dimos hardware <robot> setup --dry-run --non-interactive

# in a container, twice — the second run must report `already` for every stage
docker run --rm -v "$PWD/target/<target>/release/dimos:/usr/local/bin/dimos" ubuntu:22.04 \
  dimos hardware <robot> setup --non-interactive --yes

# on the robot — record the exact command, the exit code, and what `dimos list` showed
ssh <user>@<ip> 'dimos hardware <robot> setup --robot-ip <ip> --interface <iface>'
```

Hardware is the last rung, never the first: sim → container → robot, each proving strictly more
than the last.

---

## 8. What not to do

- No cliclack, dialoguer, indicatif, spinner, banner, or box drawing — v1 is plain text.
- No shell strings. `Action::Run` takes an argv vector.
- No `unwrap()`/`expect()` on an I/O path.
- No TODO, no FIXME, no compat shim for a caller that does not exist.
- No `std::process::Command` outside `plan.rs`, `sudo.rs` and `probe::capture`; an `observe` calls
  `capture`, so every probe has a deadline.
- No second definition of a computation. Grep before adding a helper: `plan::text`, `plan::owned`,
  `Action::run_owned` and `probe::capture` already exist.
