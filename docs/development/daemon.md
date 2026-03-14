# DimOS Daemon Architecture

How blueprint processes are launched, daemonized, logged, and stopped.

## Overview

When a blueprint is launched (every `dimos run` daemonizes by default), the process **forks before `build()`** so no threads are killed. The daemon grandchild runs the build, sends a BUILD_OK sentinel through a pipe to the parent, and then loops forever.

```
User action                   What happens
─────────────                 ────────────
DIO "launch" tab              launcher.py spawns subprocess
  or `dimos run --daemon`
                              ─────────────────────────────────────────
                              │ CLI parent:                            │
                              │   Create run_dir, write config.json   │
                              │   Create os.pipe() for startup output │
                              │   fork()                              │
                              ─────────────────────────────────────────
                                        │
                              ┌─────────┴──────────────────────────────┐
                              │  DAEMON (grandchild after double-fork) │
                              │  OutputTee: fd 1+2 → pipe + log files │
                              │  blueprint.build()                     │
                              │  health_check()                        │
                              │  Write BUILD_OK to pipe, close pipe    │
                              │  Update current.json with final PID    │
                              │  install_signal_handlers()             │
                              │  coordinator.loop()  ← blocks forever  │
                              └────────────────────────────────────────┘
                                        │
                              ┌─────────┴─────────────────────────────┐
                              │  CLI PARENT: reads pipe, echoes        │
                              │    If --detach: exit 0 after BUILD_OK  │
                              │    If attached: tail stdout.log,       │
                              │      forward ctrl+c → SIGTERM          │
                              └───────────────────────────────────────┘
```

## Directory Structure

```
~/.dimos/                              # DIMOS_HOME (env var override)
└── instances/
    └── <global_name>/                 # e.g., "unitree-go2"
        ├── current.json               # Metadata for running instance (exists while running)
        └── runs/
            └── <YYYYMMDD-HHMMSS>/     # e.g., "20260312-143005"
                ├── config.json        # Full GlobalConfig snapshot
                ├── stdout.log         # Combined stdout+stderr with ANSI colors
                ├── stdout.plain.log   # ANSI-stripped copy
                └── main.jsonl         # Structured JSON logs (structlog)
```

## Process Lifecycle

### 1. Launch (pre-fork)

**File:** `dimos/robot/cli/dimos.py` — `run()` command

```
dimos run --viewer rerun --n-workers 2 --daemon my-blueprint
```

Sequence:
1. Resolve instance name (default = blueprint name)
2. Check for name clash — prompt, `--force-replace`, or error
3. Create `run_dir` under `~/.dimos/instances/<name>/runs/<datetime>/`
4. `set_run_log_dir(run_dir)` — configures structlog to write `main.jsonl`
5. Dump `config.json` — full GlobalConfig snapshot
6. Create `os.pipe()` for startup output
7. `fork()` — parent reads pipe, child becomes daemon

### 2. Fork Before Build

**File:** `dimos/core/daemon.py` — `daemonize()`

**Key insight: fork BEFORE `blueprint.build()`** so no threads are killed.

```
CLI process (PID 100)
  │
  ├─ fork() ──→ Child (PID 101)
  │               │
  │               ├─ setsid()  ← become session leader
  │               │
  │               ├─ fork() ──→ Grandchild (PID 102)  ← THE DAEMON
  │               │               │
  │               │               ├─ stdin  → /dev/null
  │               │               ├─ OutputTee: fd 1+2 → pipe + log files
  │               │               ├─ blueprint.build()    ← ALL threads born here
  │               │               ├─ health_check()
  │               │               ├─ Write BUILD_OK to pipe
  │               │               ├─ register(InstanceInfo)
  │               │               └─ coordinator.loop()
  │               │
  │               └─ os._exit(0)
  │
  └─ Reads pipe, echoes to terminal
     └─ On BUILD_OK: detach or tail stdout.log
```

**No more `restart_daemon_threads()`** — all threads are born in the daemon.

### 3. Instance Registry

**File:** `dimos/core/instance_registry.py`

Each running instance writes `~/.dimos/instances/<name>/current.json`:
```json
{
  "name": "unitree-go2",
  "pid": 12345,
  "blueprint": "unitree-go2",
  "started_at": "2026-03-12T14:30:05+00:00",
  "run_dir": "/Users/me/.dimos/instances/unitree-go2/runs/20260312-143005",
  "grpc_port": 9877,
  "original_argv": ["dimos", "run", "--daemon", "unitree-go2"],
  "config_overrides": {"dtop": true}
}
```

Key operations: `register()`, `unregister()`, `get()`, `list_running()`, `get_sole_running()`, `stop()`, `make_run_dir()`

### 4. OutputTee

**File:** `dimos/core/output_tee.py`

Replaces the old `os.write` monkey-patch. Clean fd-level tee:
1. Creates an internal `os.pipe()`
2. `dup2`s stdout/stderr to the write end
3. Reader thread fans out bytes to: parent pipe fd, `stdout.log`, `stdout.plain.log` (ANSI-stripped)
4. `detach_parent()` stops writing to parent pipe after BUILD_OK

### 5. Shutdown

When `dimos stop` sends SIGTERM (or SIGINT):

```
Signal received
  │
  ├─ coordinator.stop()
  │    ├─ Stop StatsMonitor
  │    ├─ For each module (reverse order): module.stop()
  │    └─ WorkerManager.close_all()
  │
  ├─ unregister(name)  — deletes current.json
  ├─ tee.close()
  └─ sys.exit(0)
```

## CLI Commands

| Command | Description |
|---------|-------------|
| `dimos run <bp> [-d] [--detach] [--name N] [--force-replace]` | Launch (always daemonizes with -d) |
| `dimos stop [NAME] [-f]` | Stop instance (sole if no name) |
| `dimos restart [NAME] [-f]` | Re-exec with same argv |
| `dimos status [NAME]` | Show instance(s) details |
| `dimos log [NAME] [-f] [-n N] [--json] [--run DT]` | View logs |

### Name Resolution

- If NAME given: use that instance
- If omitted and 1 running: use it
- If omitted and 0: "No running instances"
- If omitted and 2+: list them and ask

### Name Clash Handling

- **Interactive:** Prompt "Stop existing? [y/N]"
- **DIO:** Uses `--force-replace` (shows TUI confirmation first)
- **Scripting:** `--force-replace` flag

## Log Flow

```
                          ┌─────────────────────────────────────────────┐
                          │            structlog pipeline               │
                          │                                             │
  logger.info("...")  ──→ │  filter → add_level → timestamp → callsite │
                          │                                             │
                          │     ┌──────────┐       ┌──────────────┐    │
                          │     │ Console  │       │    File      │    │
                          │     │ Handler  │       │   Handler    │    │
                          │     └────┬─────┘       └──────┬───────┘    │
                          └──────────┼────────────────────┼────────────┘
                                     │                    │
                                     ▼                    ▼
                              stdout (fd 1)         main.jsonl
                                     │              (RotatingFileHandler
                                     │               10 MiB, 20 backups)
                           ┌─────────┴─────────┐
                           │    OutputTee       │
                           │  (reader thread)   │
                           └──┬──────┬──────┬───┘
                              │      │      │
                              ▼      ▼      ▼
                         parent   stdout  stdout.plain
                          pipe     .log      .log
                        (until   (ANSI)   (stripped)
                        BUILD_OK)
```

## DIO Integration

### Launcher (`launcher.py`)
Uses `--detach --force-replace`. The CLI process exits after BUILD_OK, and the launcher detects completion.

### Status/Runner (`runner.py`)
Tails `stdout.log` from the instance's run directory. Polls `instance_registry.list_running()` every second to detect state changes.

### Chat (`humancli.py`)
Connects via LCM transports — completely separate from the log pipeline.
