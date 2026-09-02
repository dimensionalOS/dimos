# Deltas to apply on top of merged-plan.json (Aaryan's decisions after the plan was drafted, 2026-09-01)

The merged plan (/Users/aaryan/.claude/jobs/08d344ec/tmp/merged-plan.json) is the file-level contract.
Where this file and the plan disagree, THIS FILE WINS. Brief: /Users/aaryan/.claude/jobs/08d344ec/tmp/DESIGN-BRIEF.md
(decisions 17–20 and the ten-point list are the source of these deltas).

## D1 — No `doctor` anywhere
- Delete from the plan: `installer/src/doctor.rs`, `dimos doctor` in cli.rs, and ALL Python doctor files:
  `dimos/cli/hardware/checks.py`, `g1.py`, `jetson.py`, `test_g1.py`, `test_jetson.py`, and the
  `hardware_cli.py` edit. The Python side is ONLY `dimos/cli/installer_cli.py` + `test_installer_cli.py` + the two-line
  registration in `dimos/cli/dimos.py`.
- Verification lives in Rust: new file `installer/src/setup/verify.rs` builds the critical verify Stage
  for a target (`Target::Host | Target::G1 | Target::Jetson`) as `Action::Run` items that shell
  `bash -lc '<venv>/bin/python -c "<check>"'` (login shell so CYCLONEDDS_HOME / PATH edits apply):
  host: `import dimos`; `<venv>/bin/dimos list` output contains the expected blueprint (unitree-g1 / unitree-go2
  / whatever `--blueprint` says, default by target); g1 adds `import cyclonedds, unitree_sdk2py`,
  `import numpy,sys; sys.exit(int(numpy.__version__.split('.')[0])>=2)`, `.env` in <dir> contains `ROBOT_IP=`;
  jetson adds a parse of /etc/nv_tegra_release (pure fn, fixture-tested) and a torch import in a subprocess
  whose "static TLS" stderr becomes a WARN line with the LD_PRELOAD fix (never a failure; torch absent =
  skipped line). Each check is one `Action::Run`; the stage is `critical: true`; its plan text shows the
  exact python one-liners so `--dry-run` documents what "verified" means.
- `wizards/mod.rs` ends every target with `verify::stage(target, ...)` instead of a Python doctor call.

## D2 — `dimos update` is doctor + update in one command (brief decision 19)
- `update.rs` becomes: read installer.json (exit 2 `run dimos setup first` when absent) → stages:
  `self-update` (existing plan: download beside binary, VerifySha256, `--version` probe, rename swap, .bak kept,
  `--rollback`), `dimos-update` (library: `uv pip install --python <venv>/bin/python "dimos[<extras>]==<new>"`;
  dev: `git -C <dir> pull --ff-only` then `uv sync --extra <each> --inexact`; probe = already at that
  version/commit), then the machine-config stages reused from setup (`deps::system_packages`,
  `sysconfig::stage`, `jetson::stage` when jetson, plus `g1` stages when installer.json.hardware has g1),
  then `verify::stage`. Flags: `[--version TAG] [--force] [--rollback] [--dry-run]`. `--dry-run` runs every
  probe, applies nothing, prints `ok already` / `would apply <what>` / `!!` / `xx` lines, exit 0 when nothing
  is needed, 1 when something would be applied or is broken. Reuse the stage runner; do not write a second one.
- installer.json gains `hardware: {"g1"|"jetson": {"at","result"}}` written by `hardware <t> setup`.

## D3 — Plain text, no TUI (brief decision 18)
- Remove cliclack/console/indicatif from Cargo.toml and from every file contract. `Ui` in plan.rs becomes a
  20-line `say` module in `installer/src/say.rs`: `info(msg)` prints `-> msg`,
  `ok(msg)` prints `ok msg`, `warn(msg)` prints `!! msg`, `fail(msg)` prints `xx msg`; colour only when stdout
  is a TTY and NO_COLOR is unset, and only on the prefix. `confirm(question, default) -> bool` prints
  `question [Y/n]: ` and reads one stdin line (empty = default; non-interactive → default without printing
  the question; `--yes` → true). `choose(question, options) -> usize` prints numbered options and reads a
  number. Failure recovery menu is `[c]ontinue / [s]hell / [h]elp`. No banner, no spinner, no box drawing.
  Long-running actions print `-> running: <argv>` before and `ok (<secs>s)` after.
- The plan's "grep test that only Ui uses cliclack" becomes: a test that no file other than run.rs
  reads stdin or calls `print!`/`println!` for prompts (`grep -rn "stdin()" src/ | grep -v run.rs` is empty).

## D4 — `dimos robot scan` (v1, brief decision 17)
- New file `installer/src/robot_scan.rs`: `scan(opts) -> Vec<Found>` where
  `Found { kind: Lan|Wired|Ble, vendor: String, model: Option<String>, identity: Identity{Serial(String)|Mac(String)|Unknown}, addr: String, iface: String }`
  (identity and addr are separate fields on purpose — v2 keys a registry by identity).
  - LAN: mirror dimos/robot/unitree/go2/cli/landiscovery.py:129-166 — for every non-loopback IPv4
    interface except tailscale/wg/tun/docker/br-/veth (enumerated by one fixture-tested parser over
    `ip -o -4 addr` on Linux / `ifconfig -a` on macOS): bind `0.0.0.0:10134` with SO_REUSEADDR, set
    IP_MULTICAST_IF + IP_ADD_MEMBERSHIP to that interface's IPv4 (needs `socket2`, pure Rust), send
    `{"name":"unitree_dapengche"}` to 231.1.1.1:10131, listen for `--timeout-s` seconds (default 3), parse
    the reply JSON: `sn` (required → Identity::Serial) and `ip` (default: the datagram's source address).
  - Wired: for every interface holding a 192.168.123.0/24 address, `ping -c1 -W1 192.168.123.164` (the G1
    Jetson) and `192.168.123.161` (control computer / Go2 default) → `Found{kind:Wired, vendor:"unitree",
    model:None, identity:Unknown, addr}`; then `ssh -o BatchMode=yes -o ConnectTimeout=3 unitree@<addr> true`
    only to report `ssh: key ok | password needed`.
  - BLE: if installer.json exists and `<venv>/bin/dimos` exists, run `<venv>/bin/dimos go2tool discover --ble
    --timeout <t>` and reprint its rows tagged `ble`; else print `!! ble: install DimOS first (dimos setup)`.
  - Output: plain table `kind  vendor  model  identity  address  iface  note`, one line per robot,
    `-> no robots found` otherwise; `--agent` prints the Vec<Found> as JSON. Saves nothing (v2 will).
  - Tests: reply-JSON parsing from fixture strings; interface filter; wired subnet detection from fixture
    `ip -o -4 addr` output; a fake UDP responder on localhost for the LAN probe round trip.
- cli.rs: `Robot { Scan { lan: bool, wired: bool, ble: bool, timeout_s: u64 } }` (all false = all kinds).
- installer_cli.py FORWARDED = ("setup", "update", "service", "uninstall", "robot"); `hardware g1|jetson setup`
  still forwarded as in the plan (they are external subcommands on the Rust side).

## D5 — `installer/WIZARDS.md` (brief decision 20)
- Written LAST, by the agent that wrote wizards/mod.rs, after the code is final, so every type and path in it is
  real. Structure: 1) what a wizard is (one file, `fn stages(obs: &Observed, opts) -> Vec<Stage>`), 2) the
  contract (paste the real `Action`, `Stage`, `Probe`/`Observed`, `Sudo::wrap` signatures from plan.rs /
  probe.rs / sudo.rs), 3) the rules (probe per stage, dry-run must render everything, critical verify last,
  sudo only via Ctx, secrets never in argv/log, plain-text prefixes, ≤25-line fns, no unwrap on I/O, no TODO),
  4) registering it (cli.rs enum + wizards/mod.rs dispatch + platforms.toml extra), 5) the tests it must add
  (fresh → full plan; configured → empty plan; verify snippet rendering), 6) worked example: `wizards/nvidia/jetson.rs`
  explained line by line, 7) the runbook lines to add to context.md. Lean; link, don't duplicate.
- Acceptance (verify phase): an agent that has read ONLY WIZARDS.md adds `hardware demo setup` (two stages:
  WriteFile a marker under $TMPDIR, verify it exists) in a scratch worktree; it must compile, pass
  `--dry-run`, and its plan tests must run green. Failures fix the doc.

## D6 — Roadmap in code and docs
- `installer/README.md` (short) carries the v1 / v2 / v3 phase list from the brief and the command surface.
- `context.md` at the dimos repo root (branch aaryan/installer) is the handoff file (task 5).

## Unchanged from the plan
Everything else: crate layout, plan-then-exec, Sudo ladder, probes, record files, system_config/jetson/g1
stages, systemd_service, uninstall, venv_forward.rs, install.sh, installer.yml, container.sh, build order, version single-homed in
pyproject via build.rs, tests per file.
