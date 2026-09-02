# dimos-installer

The `dimos` binary: it installs DimOS and brings up robot hardware. One static musl binary per
target, no runtime, so it works before Python exists on the machine.

```sh
curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/main/scripts/install.sh | bash
```

`scripts/install.sh` downloads `dimos-<target>` plus its `.sha256`, verifies it, and execs
`dimos setup`. Everything after that is this crate.

## Commands

| Command | Does |
|---|---|
| `dimos setup` | uv, a Python 3.12 venv, DimOS, and this machine's configuration |
| `dimos update` | self-update, update DimOS, re-apply the machine config, verify — `--dry-run` is the read-only report |
| `dimos hardware g1\|jetson setup` | bring up the robot from the machine it runs on |
| `dimos robot scan` | print what is on the network (`--lan --wired --ble --timeout`) |
| `dimos service <verb> <blueprint>` | run a blueprint as a systemd unit |
| `dimos uninstall` | remove what setup installed, listing what it leaves |
| anything else | the DimOS Python CLI, exec'd from the venv |

Global flags: `--dry-run` prints the plan and touches nothing, `--non-interactive` never prompts
(also implied when stdin is not a terminal), `--yes` grants sudo and package consent, `--agent`
prints one JSON plan and one JSON report on stdout, `--verbose` echoes every child line.

Exit codes: `0` ok, `1` a critical stage failed, `2` a human is needed and the run prints what.

## How it is built

Every command builds a `Plan` of `Stage`s of `Action`s from `probe::Probes`; `plan::run` is the only
function that mutates the machine or reads stdin, and every probe spawns through `probe::capture`
with a deadline. A stage with no actions means the machine is already there, so a second run is
free and `--dry-run` is trustworthy. Adding a robot: [WIZARDS.md](WIZARDS.md).

## Roadmap

- **v1 (this)** — find robots and install DimOS on hardware. `robot scan` prints and exits; it saves
  nothing. `hardware g1|jetson setup` takes `--robot-ip` copied from the scan output.
- **v2** — a robot registry at `~/.config/dimos/robots.json` keyed by serial or MAC, with
  `robot list|show|rename|rm`, tags, and `--robot <name>`. A re-scan that finds a known robot at a
  new address updates it in place, which is why v1's scan result already keeps identity and address
  as separate fields.
- **v3** — Go2 offboard bring-up, `hardware g1 reach` for workstation-side wired/wifi provisioning,
  and a richer terminal UI.
