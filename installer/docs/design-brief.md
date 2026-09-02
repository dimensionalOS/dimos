# DimOS installer — design brief (fixed by the orchestrator, 2026-09-01)

Goal (Aaryan): a fully cleaned-up, refactored, nicely structured DimOS installer living entirely inside
the dimos repo. `curl | bash` installs the `dimos` CLI, which installs DimOS and brings up hardware
(G1 today, Jetson Orin NX today, Go2 later). Wizards live inside dimos. Rust where it earns it. Very
robust. Tested today on macOS arm64, Linux x86_64 (10.0.0.73), a Unitree G1 (Jetson Orin NX, Ubuntu
20.04, glibc 2.31, JetPack 5.1.1), and a standalone Jetson Orin NX. Branch `aaryan/installer` on
dimensionalOS/dimos, worktree at /Users/aaryan/Files/Dimensional/dimos/.claude/worktrees/installer.

Source material (verified maps from seven readers; cite them, do not re-derive):
- /Users/aaryan/.claude/projects/-Users-aaryan-Files-Dimensional-dimos--claude-worktrees-installer/08d344ec-1adb-4084-889f-8a3f8057bc05/tool-results/bmc9pm5h9.txt  (G1 bring-up steps + traps; dim build/release/install.sh/update; hermes UX patterns; learnings/traps/house rules)
- .../tool-results/bj2icjhr2.txt  (dim Rust file-by-file: setup flow, state files, verdicts)
- .../tool-results/bxhjjf6br.txt  (dimos: CLI, pyproject/extras, scripts/install.sh, docs, CI, rust workspace, uv)
- /private/tmp/claude-501/-Users-aaryan-Files-Dimensional/08d344ec-1adb-4084-889f-8a3f8057bc05/tasks/whtq75ycc.output  (full JSON; key `result.hw` = existing dimos hardware/config code to call vs replace)
- dim source: /Users/aaryan/.claude/jobs/08d344ec/tmp/dios/src   (read specific files when a contract needs exact behaviour)
- dimos worktree: /Users/aaryan/Files/Dimensional/dimos/.claude/worktrees/installer
- House rules: /Users/aaryan/Files/Dimensional/.claude/worktrees/dimos-installer-spec/CLAUDE.md (§ Hard rule, § Writing code the dimos way) and AGENTS.md (§5 House style, §6 Writing)

## Decisions already made — do not relitigate

1. **One Rust crate `installer/` in the dimos repo, package `dimos-installer`, binary `dimos`.** It joins the
   root Cargo workspace as a member (free fmt/clippy/test in CI, shared lock, release profile lto=thin).
   Static musl for linux x86_64 + aarch64, plain cargo for aarch64-apple-darwin. Cross-build locally with
   `cargo zigbuild` (installed on this Mac); CI job copies dim's release.yml matrix and ADDS sha256 files.
2. **Rust owns what must work before Python exists**: bootstrap/self-install/PATH, platform detection
   (OS, arch, Jetson via /etc/nv_tegra_release + /proc/device-tree, glibc, GPU incl. Tegra without
   nvidia-smi), system packages (apt/brew, consent-gated), uv + `uv venv --python 3.12` + DimOS install,
   machine config PERSISTENCE (sysctl rmem 64 MiB, multicast systemd unit, memlock limits.d, Jetson
   nvpmodel/jetson_clocks), update (decision 19: doctor + update in one command), service (systemd unit
   for a blueprint), uninstall, `robot scan`, and
   `hardware <target> setup` for g1 / jetson (go2 later). Unknown subcommands are forwarded to
   `<venv>/bin/dimos` (the Python CLI). **There is no `doctor` command (Aaryan, 2026-09-01: "i dont need
   doctor").** Verification is the final, critical stage of `setup` and of every `hardware <t> setup`,
   and it fails the run (dim's warn-only verify is the bug that produced "green install, dead robot").
3. **Verification runs from Rust through the venv's Python, so the Python side stays tiny.** The verify
   stage shells `<venv>/bin/python -c '<one-line check>'` under `bash -lc` (a fresh login shell, so
   CYCLONEDDS_HOME and PATH edits are what a user gets) for each check: `import dimos`;
   `<venv>/bin/dimos list` contains the target blueprint; for g1: `import cyclonedds, unitree_sdk2py`,
   `numpy.__version__` major < 2, `.env` carries ROBOT_IP; for jetson: `/etc/nv_tegra_release` parsed,
   `import torch` in a subprocess with the static-TLS message turned into a WARN line carrying the exact
   LD_PRELOAD fix (never a failure). No new Python commands for this. The only Python change is the
   forwarder so `dimos setup|update|service|robot|uninstall` inside an activated venv exec the Rust
   binary at `~/.local/bin/dimos` by absolute path (never by name — recursion guard via DIMOS_FORWARDED).
   The runtime SystemConfigurators in dimos/protocol/service/system_configurator/ are NOT touched.
4. **Install source = pinned by construction**: library mode installs `dimos[<extras>]==<DIMOS_VERSION>` (the pyproject version exported by build.rs; Cargo's own version is 0.0.0)
   from PyPI because installer and package ship on the same tag; dev mode = `git clone -b <branch>` +
   `uv sync --extra <extras>` (GIT_LFS_SKIP_SMUDGE=1). `--branch` / `DIMOS_BRANCH`. Today's tests use
   `--mode dev --branch aaryan/installer` because PyPI has no build with the Python-side changes yet.
5. **Extras come from pyproject.toml at build time** (build.rs parses `[project.optional-dependencies]`
   names into a const; no hand-mirrored lists). System packages + sysctl live in `installer/platforms.toml`
   keyed by extra name and platform, `include_str!`'d. Adding a dependency to a module is a pyproject edit.
6. **Plan-then-exec is structural**: every stage builds a `Vec<Action>` (Run{cmd,sudo}, WriteFile{path,
   mode, contents}, EnsureLine{file, marker, line}, ...) from probes; `--dry-run` prints the plan and never
   spawns anything that mutates; `exec` is the only place that touches the world. This is how the
   prototype made dry-run trustworthy and it is required.
7. **Modes**: interactive (failed non-critical stage → continue / shell / help), `--non-interactive`
   (also auto when stdin is not a TTY: critical stops, others logged and skipped, safe defaults, and
   a bare non-interactive run DOES install DimOS with extras=base — dim's "installs nothing" default
   is a bug), `--yes` grants sudo consent, `--agent` prints a machine-readable JSON plan/result.
   Exit codes mean something: 0 ok, 1 failed critical, 2 needs a human (lists what), 130 interrupted.
8. **sudo without a TTY** (the #1 real-G1 failure): honor `SUDO_ASKPASS`; accept `DIMOS_SUDO_PASSWORD`
   env (never argv) and feed `sudo -S` via stdin; `sudo -n true` probe first; clear exit-2 message when
   root is needed and unavailable. Never echo the password; never write it to the action log.
9. **Bootstrap `scripts/install.sh` ≤120 lines**: detect os/arch → resolve version (`DIMOS_VERSION`
   default latest) → download `dimos-<target>` + `dimos-<target>.sha256` from
   `${DIMOS_INSTALLER_URL:-https://github.com/dimensionalOS/dimos/releases/download/<tag>}` → verify sha256
   (shasum -a 256 / sha256sum) → exec `dimos setup "$@"`. `DIMOS_INSTALLER_URL` override lets today's
   LAN test serve locally built binaries with `python3 -m http.server`. Rewrite the existing 1000-line
   gum installer in place (same path) so docs/README curl lines keep working; it is deleted as an
   installer.
10. **Naming and state**: state at `~/.config/dimos/installer.json` (installer version, DimOS version,
    mode, dir, extras, last result) — NOT `~/.dimos` (dimos already owns ~/.config/dimos via
    dimos/constants.py CONFIG_DIR, and ~/.dimos was dim's). Action log at
    `~/.local/state/dimos/installer.jsonl` (STATE_DIR). No secret ever reaches either. Systemd unit
    `dimos-<blueprint>.service`. PATH edits only in per-user rc files with a `# DIMOS-ADDED` guarded block
    (never /etc/* with sudo).
11. **Drop from dim**: desktop, appstore, webserver, docker payload (DIMOSPAK), compat checker + the stale
    generated_platform_reqs.json, init/new_app, nearby-dimos BFS scan, banner animation, Deno prebuild,
    "record entire environment into service.json", and every TUI widget (decision 18). Remaining deps
    target: anyhow, clap(derive), dirs, serde, serde_json, toml, which, xshell, sha2, hex, chrono(clock).
    Downloads shell out to system curl (as dim does) — no TLS stack in the binary.
12. **Nix is opt-in** (`--with-nix`), default off, like dimos's own install.sh. Native Rust modules are
    cargo-built at first run from a checkout; that is dimos's existing behaviour, not the installer's.
13. **G1 onboard path** (`dimos hardware g1 setup`, run ON the robot after `dimos setup --mode dev
    --extras unitree`; the workstation-side wired/wifi reach is `dimos hardware g1 reach` and is optional
    for today because Aaryan can ssh in; both write the robot record of decision 17): cmake; CycloneDDS releases/0.10.x source build to
    ~/cyclonedds/install; clone unitree_sdk2_python (SDK2_PATH | /opt | ~); `uv pip install
    cyclonedds==0.10.2` with CYCLONEDDS_HOME; numpy<2 pin + `--no-deps -e sdk`; persist CYCLONEDDS_HOME
    in rc; nvpmodel -m 0 + jetson_clocks; `git config --global url.https://github.com/.insteadOf
    ssh://git@github.com/`; write `.env` in the project dir with ROBOT_IP / DIMOS_TRANSPORT; then the
    verify stage of decision 3 must pass. Every fact/trap from the G1 map applies (login selector does not
    hang BatchMode ssh; wpa-psk profile; 192.168.123.164; vendor dirs never touched).
14. **Jetson path** (`dimos hardware jetson setup`, Orin NX): detect L4T/JetPack from /etc/nv_tegra_release;
    nvpmodel/jetson_clocks; refuse the `cuda` extra on aarch64 with the reason (pyproject gates it to
    x86_64); warn (never block) on glibc<2.34 static-TLS with the exact LD_PRELOAD line; memlock+sysctl
    persistence; then the verify stage of decision 3 must pass.
15. **Tests are the docs**: cargo unit tests for every pure function (plan builders from fixture probe
    results, platform detection from fixture strings, sha256, version compare, extras validation, sysctl
    need computation, .env writer, unit-file rendering with escaping, verify-check snippet rendering)
    with invariant-docstring names; one Python test co-located (test_forward.py: both-directions
    invariant + recursion guard). Integration: the musl binary run in ubuntu:22.04 containers (amd64 + arm64) doing a full
    non-interactive dev install and a second idempotent run — the orchestrator runs this after build.
16. **House rules apply to Rust too**: smallest diff, no compat shims for callers that do not exist, logic
    in small named functions (≤25 lines; entry points read as lists of calls), one-line docstrings,
    comments are WHY only, no TODO/FIXME, units in names (`_bytes`, `_s`), errors say got-vs-want or the
    next command, `raise loudly / catch narrowly`, no bare `except`/`unwrap()` on I/O paths in the
    binary (use `?` with context), structured action log not f-string prose. Copy Jeff's good parts
    (handle_stage, fresh-login-shell PATH probe, guarded rc blocks, sysconfig persistence, update swap)
    rather than re-inventing them — but rewrite, do not paste, and cite the origin in the PR not the code.

17. **Phases (Aaryan, 2026-09-01: "maintain v1 and v2 etc"). Build v1 today; keep v2/v3 written down
    so nothing is designed in a way that blocks them, but do not build them.**
    - **v1 (today): find robots and install DimOS on hardware.** `dimos robot scan [--lan|--wired|--ble]
      [--timeout S]` prints what it finds and exits — it saves nothing. Rust implements the two probes that
      must work before Python exists: the Unitree LAN multicast probe (send `{"name":"unitree_dapengche"}` to
      231.1.1.1:10131, listen on 10134, per non-VPN interface — mirror dimos/robot/unitree/go2/cli/
      landiscovery.py exactly) and the wired G1 probe (ping/ssh 192.168.123.164 on any interface carrying a
      192.168.123.0/24 address). BLE stays in Python (bleak, dimos ble.py) and is invoked through the venv
      when installed, else reported as "install first". `hardware g1|jetson setup` take `--robot-ip`; the
      operator copies the address from the scan output. Output is a plain table: `kind  model  serial/mac  ip`.
    - **v2 (next phase, NOT built now): robot saving and tagging.** `~/.config/dimos/robots.json` keyed by
      serial or MAC, `dimos robot list|show|rename|rm`, tags, `--robot <name>` resolution, and the invariant
      that a re-scan finding a known robot at a new IP updates the entry in place (test
      `registry_rescan_moves_address_keeps_identity`). v1 must not preclude it: the scan result type carries
      identity (serial/MAC) and addresses as separate fields from day one.
    - **v3 (later): Go2 offboard bring-up, `hardware g1 reach` (workstation-side wired/wifi provisioning),
      and any richer terminal UI.**
18. **v1 has no TUI — plain text in the terminal.** No cliclack, dialoguer, indicatif, spinners, boxes, or
    banner. Output is plain lines with four fixed prefixes (`->` info, `ok` done, `!!` warn, `xx` fail) so
    it reads the same in a log file, over ssh, and in CI; honor `NO_COLOR` (colour is off by default and
    only ever on for the four prefixes when stdout is a TTY and NO_COLOR is unset). Prompts are single-line
    `question [y/N]: ` reads from stdin with a default; a multiple choice prompt lists numbered options.
    Failure recovery in interactive mode is the same three choices as dim's handle_stage, offered as
    `[c]ontinue / [s]hell / [h]elp`. Remaining crate deps: anyhow, clap(derive), dirs, serde, serde_json,
    toml, which, xshell, sha2, hex, chrono(clock) — nothing that draws.

19. **`dimos update` is doctor and update in one command (Aaryan, 2026-09-01).** It reads
    `installer.json` (mode, dir, extras, branch) and runs, in order, each as a stage with a probe so a
    no-op is reported as `ok already`: (1) self-update the binary when a newer release exists or
    `--version <tag>` is given: download `dimos-<target>` + `.sha256` → verify → run `--version` on the
    new file → atomic swap → keep `.bak` until the next exit-0 run; (2) update DimOS in the recorded dir:
    library `uv pip install --python <venv> "dimos[<extras>]==<new>"`, dev `git -C <dir> pull --ff-only`
    then `uv sync --extra <extras> --inexact`; (3) re-run the machine-config stages of `setup`
    (packages, sysctl/multicast/memlock, jetson perf) idempotently — this is the repair; (4) the verify
    stage. It prints one line per stage: `ok already`, `ok applied <what>`, `!! <warn>`, `xx <fail>`, then
    a summary. `dimos update --dry-run` is the read-only report (what doctor used to be): every probe
    runs, nothing is applied, exit 0 if nothing is needed, 1 if something would be applied or is broken,
    plus `--agent` JSON. No `installer.json` yet → exit 2 `run dimos setup first`. A hardware target's
    stages are included when `installer.json.hardware` records one (v1: g1 or jetson).

20. **A wizard-maker guide ships in the crate: `installer/WIZARDS.md` (Aaryan, 2026-09-01).** It is
    written for a coding agent (and an FDE driving one) integrating a new robot: how a hardware wizard is
    one Rust file `installer/src/setup/<robot>.rs` of stage builders (`fn(&Observed) -> Stage`) composed
    by `installer/src/hardware.rs`, beside the shared `setup/jetson.rs` and `setup/sysconfig.rs`; the
    Stage/Action/Probe contract with the exact types (copied from the code, not paraphrased); the rules —
    every stage has a probe so re-runs are no-ops, `--dry-run` must render the whole plan without touching
    the machine, a critical verify stage last, sudo only through the `Sudo` wrapper, secrets never in argv
    or logs, plain-text output prefixes; where to register the subcommand (`hardware/mod.rs` dispatch +
    `cli.rs` enum); the `platforms.toml` entry for its extra; the tests it must add (fixture-driven plan
    tests: fresh machine → full plan, configured machine → empty plan) and the container/hardware runbook
    lines; and a worked example that walks through `hardware/jetson.rs` line by line. One line each on
    what NOT to do (no cliclack, no shell strings, no `unwrap` on I/O, no TODO). Lean: link to code, do
    not duplicate it. **Acceptance test:** an agent that has only read WIZARDS.md adds a throwaway
    `hardware demo` wizard (two stages: write a marker file, verify it) in a scratch worktree and it
    compiles, passes `--dry-run`, and its plan tests run green — the orchestrator runs this in the verify
    phase and fixes the doc, not the agent, when it fails.

## Things Aaryan asked for today — the build must make each one visibly true

1. curl one-liner installs the `dimos` CLI, which installs DimOS and brings up hardware (G1, Jetson Orin NX).
2. Everything lives inside the dimos repo, on branch `aaryan/installer` at dimensionalOS/dimos; Rust where it earns it.
3. Cleaned up, refactored, nicely structured — small named functions, one home per computation, house rules.
4. Very robust — plan-then-exec, probes, idempotent re-runs, sudo without a TTY handled, checksum on download, exit codes that mean something.
5. Tested today on macOS, Linux (10.0.0.73), G1, Jetson Orin NX — runbook per platform, container runs before hardware.
6. v1 finds robots (`robot scan`) and installs on hardware; saving/tagging robots is v2; roadmap v1/v2/v3 maintained in docs + context.md.
7. v1 has no TUI: plain terminal text.
8. No `doctor`; `update` = doctor + update in one command, `update --dry-run` is the report.
9. `installer/WIZARDS.md` teaches agents to build their own wizard for a new robot; tested by having an agent do it.
10. `context.md` on the branch carries every workflow and test result so another window can continue.

## What the build plan must contain

A JSON object (schema given by the workflow) with: the crate/module tree and every file's contract
(public fns with signatures + one-line purpose, tests it owns), the Python-side files and contracts,
the exact command surface with flags, the Action/Plan/Stage types, the platforms.toml shape, the
state file schema, install.sh contents outline, the CI workflow, the ordered implementation steps
with which files can be written in parallel vs must be sequential, and the test plan per platform
for today (exact commands and expected observations).
