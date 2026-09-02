# DimOS installer — branch `aaryan/installer` — handoff

Read this first. It has everything another window or machine needs to continue or test. Last
updated 2026-09-01 (see the log at the bottom).

## What this branch is

One installer, inside the dimos repo, in Rust. `curl | bash` fetches a static `dimos` binary that
installs DimOS on the current machine and brings up hardware. Wizards live in the crate. The old
1000-line `scripts/install.sh` is replaced in place by an 80-line bootstrapper. Jeff's private
`dimensionalOS/dios` (`dim` v0.3.97) was the source of the reusable parts; nothing was copied
verbatim.

```
installer/                 Rust crate `dimos-installer`, binary `dimos`, workspace member
  src/plan.rs              Action / Stage / Plan; `run` is the ONLY fn that spawns, writes, prompts
  src/probe.rs             read the machine once into Observed; every parser pure over &str
  src/sudo.rs              Root | Passwordless | Askpass | Stdin(DIMOS_SUDO_PASSWORD) | Tty | Unavailable
  src/state.rs             ~/.config/dimos/installer.json + ~/.local/state/dimos/installer.jsonl
  src/pkgs.rs              platforms.toml + extras from pyproject (build.rs); cuda refused on aarch64
  src/cli.rs               clap surface
  src/setup/{mod,self_install,deps,install,sysconfig,jetson,g1,verify}.rs
  src/{update,service,uninstall,robot,hardware,forward}.rs
  platforms.toml           apt/brew packages per extra; sysctl + memlock values, with the WHY
  WIZARDS.md               how an agent adds a wizard for a new robot
  README.md                command surface + v1/v2/v3 roadmap
  context.md               this handoff (the repo root already has an upstream CONTEXT.md, and macOS is case-insensitive)
  docs/design-brief.md     the fixed design decisions (1–20) this was built from
  docs/build-deltas.md     Aaryan's same-day changes layered on the plan (D1–D6)
  tests/container.sh       ubuntu:22.04 amd64/arm64 harness (not run yet — see Testing)
scripts/install.sh         the bootstrapper (rewritten in place; README/quickstart curl lines unchanged)
.github/workflows/installer.yml   3 targets + sha256 on v* tags; crate checks on PRs
dimos/cli/forward.py       inside the venv, `dimos setup|update|service|uninstall|robot` exec the binary
```

## Decisions that shape it (full text: installer/docs/design-brief.md)

- **Probe → plan → exec.** Every stage builder is `fn(&Observed) -> Stage`. Empty actions means
  "already done", so a re-run is a no-op and `--dry-run` prints the entire plan without touching disk.
- **Verify is the last, critical stage** of `setup` and of every `hardware <t> setup`. It runs
  `bash -lc '<venv>/bin/python -c ...'` checks and fails the run. A green install that cannot start a
  blueprint was the real G1 failure in August.
- **No `doctor`.** `dimos update` is doctor and update in one: self-update → DimOS update →
  re-run the machine-config stages (the repair) → verify. `update --dry-run` is the read-only report.
- **Plain terminal text in v1.** Prefixes `->` `ok` `!!` `xx`, `[y/N]` prompts, `NO_COLOR` honored.
  No TUI crate in the dependency list.
- **v1 finds robots and installs. v2 remembers them. v3 is Go2 offboard + the reach wizard.**
  `robot scan` prints a table and saves nothing; its `Found` type keeps identity (serial/MAC) and
  address as separate fields so v2's registry is additive. Not deferred to Zenoh autodiscovery.
- **sudo with no TTY** (the real on-robot case): `DIMOS_SUDO_PASSWORD` env → `sudo -S` stdin, or
  `SUDO_ASKPASS`, or `sudo -n`; never argv, never logged. Exit 2 with the fix when none works.
- **Exit codes:** 0 ok · 1 a critical stage failed · 2 needs a human (lists what) · 130 interrupted.
- **Version has one home:** pyproject `[project].version` → `DIMOS_VERSION` at build time. Extras
  likewise. Cargo.toml version is `0.0.0` on purpose.

## Command surface (v1)

```
dimos setup    [--mode library|dev] [--extras a,b] [--branch B] [--dir PATH] [--with-nix]
dimos update   [--version TAG] [--force] [--rollback] [--dry-run]
dimos service  setup <blueprint> [--env K=V] | start|stop|restart|status|remove <blueprint>
dimos uninstall [--yes]
dimos robot scan [--lan] [--wired] [--ble] [--timeout N]
dimos hardware g1 setup     [--robot-ip IP] [--interface eth0] [--transport lcm|zenoh] [--sdk-path P]
dimos hardware jetson setup
dimos <anything else>       forwarded to the Python CLI in the venv (dimos run, list, ...)
global: --dry-run --non-interactive --yes --agent --verbose
```

Library mode pins `dimos[extras]==<DIMOS_VERSION>` from PyPI, so it only works once a release with
this branch's Python-side change exists. **Until then test in dev mode from this branch:**
`--mode dev --branch aaryan/installer`.

## Testing — where things stand

Done on this Mac (macOS 14, arm64): `cargo fmt`, `cargo clippy -D warnings`, `cargo test`,
`pytest dimos/cli/test_forward.py`, `bash -n` + shellcheck on install.sh, cross-builds for
`x86_64-unknown-linux-musl` and `aarch64-unknown-linux-musl` via `cargo zigbuild`, and the local
dry-run sanity. Results are in the log below.

**Not run:** the container harness (`installer/tests/container.sh`), any real install on Linux, the
G1, or the Jetson. That is the next step, on another machine.

### Getting a binary onto a test machine

Two ways, since there is no GitHub release of this branch yet.

**A. Build from source on the test machine** (needs Rust; Linux x86_64 laptop or a Mac):

```bash
git clone -b aaryan/installer https://github.com/dimensionalOS/dimos.git ~/dimos
cd ~/dimos
cargo build --release -p dimos-installer
./target/release/dimos --version                 # dimos 0.0.14b1
```

**B. Cross-build here, serve over the LAN, use the real curl path** (for the G1 and Jetson, which
have no Rust and are aarch64):

```bash
# on the Mac, in the checkout
cargo zigbuild --release -p dimos-installer --target aarch64-unknown-linux-musl
cargo zigbuild --release -p dimos-installer --target x86_64-unknown-linux-musl
mkdir -p /tmp/dimos-dist && cd /tmp/dimos-dist
cp ~/Files/Dimensional/dimos/.claude/worktrees/installer/target/aarch64-unknown-linux-musl/release/dimos dimos-aarch64-linux-musl
cp ~/Files/Dimensional/dimos/.claude/worktrees/installer/target/x86_64-unknown-linux-musl/release/dimos  dimos-x86_64-linux-musl
shasum -a 256 dimos-aarch64-linux-musl > dimos-aarch64-linux-musl.sha256
shasum -a 256 dimos-x86_64-linux-musl  > dimos-x86_64-linux-musl.sha256
python3 -m http.server 8000          # leave running; note the Mac's LAN IP (ipconfig getifaddr en0)

# on the test machine
export DIMOS_INSTALLER_URL=http://<mac-ip>:8000
bash <(curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/aaryan/installer/scripts/install.sh) \
  --mode dev --branch aaryan/installer --extras base --dir ~/dimos
```

### Runbook per platform

**Every platform, first:** dry-run, then the real thing, then the idempotence check.

```bash
dimos setup --dry-run --non-interactive --mode dev --branch aaryan/installer --extras base --dir ~/dimos
#   expect: the full plan printed, nothing created (ls ~/dimos → no such file), exit 0
dimos setup --mode dev --branch aaryan/installer --extras base --dir ~/dimos
#   expect: "ok" lines per stage, verify passes, exit 0; ~/.local/bin/dimos exists; ~/.config/dimos/installer.json written
dimos setup --non-interactive --yes --mode dev --branch aaryan/installer --extras base --dir ~/dimos
#   expect: every stage "ok already", exit 0; grep -c '"outcome":"applied"' ~/.local/state/dimos/installer.jsonl unchanged
dimos update --dry-run          # expect: exit 0, nothing to apply
dimos list                      # forwarded to the Python CLI; prints blueprints
cd ~/dimos && source .venv/bin/activate && dimos update --dry-run   # the venv's dimos forwards back to the binary
```

**macOS arm64.** As above. Expect brew packages offered with consent, no sysctl stage (macOS
buffers are set by dimos at run time), no systemd.

**Linux x86_64 laptop (Ubuntu 22.04/24.04).** As above with apt. Then:

```bash
dimos robot scan --timeout 3              # table of robots on the LAN, or "no robots found"
dimos service setup unitree-go2           # writes /etc/systemd/system/dimos-unitree-go2.service; needs sudo
dimos service status unitree-go2
```

**Jetson Orin NX (standalone, JetPack 5.x/6.x, aarch64).** Serve the aarch64 binary (path B). Then:

```bash
dimos setup --mode dev --branch aaryan/installer --extras base --dir ~/dimos
dimos hardware jetson setup               # nvpmodel -m 0, jetson_clocks unit, sysctl + memlock persisted, verify
#   expect: `cuda` extra refused with the reason if requested; glibc<2.34 gives a static-TLS WARN line with the LD_PRELOAD fix, never a failure
sudo reboot && dimos update --dry-run     # persistence survived the reboot → nothing to apply
```

**Unitree G1 (on the robot's Jetson, Ubuntu 20.04, glibc 2.31).** Wired: laptop at 192.168.123.100/24,
`ssh unitree@192.168.123.164` (factory password `123`; some units have password auth off). The login
selector `ros:foxy(1) noetic(2)` does not block non-interactive ssh. Serve the aarch64 binary (path B).

```bash
export DIMOS_SUDO_PASSWORD=123            # sudo over ssh has no TTY; env → sudo -S stdin, never argv
export DIMOS_INSTALLER_URL=http://<mac-ip>:8000
bash <(curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/aaryan/installer/scripts/install.sh) \
  --mode dev --branch aaryan/installer --extras unitree --dir /home/unitree/dimos
# hardware commands are the Rust binary's; run them with the venv NOT activated (or by absolute path)
~/.local/bin/dimos hardware g1 setup --robot-ip 192.168.123.161 --interface eth0
#   stages: cmake · CycloneDDS releases/0.10.x source build (~10 min) · unitree_sdk2_python clone ·
#           cyclonedds==0.10.2 · numpy<2 + sdk · CYCLONEDDS_HOME rc block · git insteadOf https ·
#           .env ROBOT_IP · nvpmodel/jetson_clocks · sysctl/memlock · verify (imports, numpy<2, ROBOT_IP)
#   --robot-ip is the CONTROL computer (.161): G1Config.ip is what the WebRTC driver dials from the Jetson
#   why the C-library build exists: PyPI has no manylinux aarch64 wheel for ANY cyclonedds version
#   (checked 2026-09-01), so on the G1 the python package builds from sdist against CYCLONEDDS_HOME.
#   That is also why `--extras unitree-dds` is refused on the G1 path: it would pull cyclonedds 11.x
#   against the 0.10.x library. Use `--extras unitree` and let the wizard own the DDS stack.
~/.local/bin/dimos hardware g1 setup --robot-ip 192.168.123.161 --interface eth0   # second run: all "ok already"
cd ~/dimos && source .venv/bin/activate
dimos --rerun-open none --rerun-host 0.0.0.0 run unitree-g1            # the human step; robot on a gantry, controller in hand
```

Traps that bit before (all handled in code, listed so nobody re-learns them): the Unitree app's IP is
the control computer, not the Jetson; `nmcli device wifi connect` fails on the mixed WPA2/WPA3
office network (use an explicit wpa-psk profile); prove WiFi ssh before unplugging Ethernet; never
delete `~/unitree_sdk2-main` or `~/*_unitree_install`; never open port 80 on the robot (that page is
Unitree's firmware updater and its Factory Reset acts on the vendor stack); plug the shipped USB-C
head-camera cable or `/dev/video*` never appears; the native viewer OOMs on the headless robot, so
always `--rerun-open none`; MAXN mode runs hot, so keep the robot on the charger for long sessions.

## What is open

- Container harness and every hardware rung above: not run. First real run will find things; fix
  in the stage that owns them and keep the probe/plan pair in sync so idempotence holds.
- Library mode (`--mode library`) needs a PyPI release built from a branch that includes
  `dimos/cli/forward.py`; until then it is unit-tested only.
- `installer/WIZARDS.md` was written but not yet proven by having a fresh agent follow it
  (planned acceptance test, cut for time).
- v2: `~/.config/dimos/robots.json`, `robot list|show|rename|rm`, tags, `--robot <name>`, and the
  invariant that a re-scan finding a known robot at a new IP updates the entry in place.
- v3: `hardware go2 *` (offboard, WebRTC/BLE), `hardware g1 reach` (workstation-side wired/WiFi
  provisioning), richer terminal UI.
- Docs in the knowledge-base repo (`temp/dios/DIOS-PRD.md`, `ARCHITECTURE.md`,
  `review-dispositions.md`) still say discovery is deferred to Zenoh autodiscovery; they need the
  v1/v2 phase split written in.
- Not yet rebased on `origin/main` after the final commits (do `git pull --rebase origin main` before
  opening the PR; LFS smudge fails on this Mac — use `GIT_LFS_SKIP_SMUDGE=1`).
- **Merge order matters.** `README.md:143` and `docs/quickstart.md:24` curl `scripts/install.sh` from
  `main`, and the new install.sh defaults to the latest GitHub release. The moment this lands on
  main, the public one-liner 404s until a `v*` tag carries `dimos-<target>` + `.sha256` assets. So:
  merge → dispatch `release.yml` → confirm `installer.yml` uploaded the three binaries → then the
  README line is live. Until then `DIMOS_INSTALLER_URL` is the override.
- Installer flags go AFTER the verb inside the venv (`dimos setup --agent`, not `dimos --agent setup`):
  the Python CLI's root options swallow the global flags before the forwarder sees them.

## Provenance

Built 2026-09-01 by an orchestrated set of agents: seven readers mapped `dim`, the dimos surface,
the G1 bring-up guides, prior prototype learnings, and the hermes installer pattern; three planners
plus a judge produced per-file contracts; a critic checked them against the code maps; one agent
wrote the core, one per file wrote the stages, one integrated, one reviewed, one fixed. Aaryan set
the phase split, no-doctor, plain-text, and wizard-guide decisions the same day. Review comments on
the earlier PRD (Paul, Ivan, Jeff, Stash, Jetson Wu) are dispositioned in the knowledge-base repo.

## Log

- 2026-08-31 — branch created from origin/main 1fa7cabfd; v2 PRD/ARCHITECTURE written in the KB repo.
- 2026-09-01 — scaffold committed (workspace member, build.rs version+extras from pyproject, platforms.toml).
- 2026-09-01 — full crate + bootstrapper + forwarder built, integrated and committed. On this Mac
  (arm64): `cargo fmt --check`, `cargo clippy --all-targets -- -D warnings`, `cargo test` (245 unit +
  7 build-script tests) green; `pytest dimos/cli/test_forward.py dimos/cli/test_cli_startup.py`
  14 passed; `bash -n` + shellcheck clean on `scripts/install.sh` and `installer/tests/container.sh`;
  `cargo zigbuild --release` for `aarch64-unknown-linux-musl` and `x86_64-unknown-linux-musl` both
  `statically linked`; `cargo build --release` for the Mac. Sanity: `dimos --version` → `dimos 0.0.14b1`;
  `robot scan --timeout 2` → `no robots found` plus the BLE install-first warning; `update --dry-run`
  → exit 2 `run dimos setup first`; `uninstall --dry-run` → exit 0, three stages; `hardware jetson
  setup --dry-run` → exit 1 `run dimos setup first`. `setup --dry-run --dir /tmp/dimos-dryrun` stopped
  at the preflight disk gate (8 GiB free, 12 GiB needed) — the gate working, not a defect; the dry-run
  plan itself is the CI smoke step and the container harness. Containers and hardware: not run from
  this seat.
