# DimOS · Hogwarts Hackathon Working Tree (`jamjam_ui` branch)

> Branched from [`jamjamDimos/dimos:jamjam`](https://github.com/jamjamDimos/dimos/tree/jamjam),
> which itself layers YOLO-E open-vocab tracking + target-lock + bbox-distance
> follow on top of [`dimensionalOS/dimos`](https://github.com/dimensionalOS/dimos).
>
> This file documents **what `jamjam_ui` adds on top of `jamjam`** and how to
> run it. The upstream-only docs live in [`README.md`](./README.md); the
> jamjam control stack docs live in
> [`dimos/robot/custom/README.md`](./dimos/robot/custom/README.md).

---

## TL;DR

```bash
# MuJoCo simulation, no hardware needed — best for trying the UI
SIM=mujoco scripts/run-blueprint.sh
open http://localhost:7782/

# Real Go2 on its own AP (dimair10 / 192.168.12.1):
sudo route add -net 224.0.0.0/4 -interface lo0     # LCM multicast onto lo0
scripts/run-blueprint.sh                            # default ROBOT_IP=192.168.12.1
open http://localhost:7782/
```

Logs land in `/tmp/dimos-logs/` with a `latest.log` symlink (auto-rotated; keeps newest 20).

---

## Two-sentence overview of `jamjam_ui`

`jamjam_ui` adds **a Harry-Potter "Marauder's Map" web app** (`dimos/apps/marauders_map/`)
on top of jamjam's perception/control stack. The app composes upstream
navigation + jamjam's tracking + the WebRTC connection into a single
blueprint, then renders a parchment-styled web UI on port 7782 that clients
can click to track people, click to navigate, and drive with WASD.

Everything new lives **under `dimos/apps/`** so jamjam upstream can fast-forward
with zero merge conflict.

---

## Project structure (after this branch's additions)

```
DimOS/                                  ← origin   = dimensionalOS/dimos
│                                       ← jamjam   = jamjamDimos/dimos
│                                       ← branch   = jamjam_ui
│
├── HACKATHON.md                        ★ this file
├── README.md                              upstream DimOS README — untouched
│
├── dimos/apps/                         ★ NEW NAMESPACE — application code lives here
│   └── marauders_map/                  ★ The Wrath of Filch
│       ├── blueprint.py                   autoconnect composition of the full closed loop
│       ├── module.py                      ReidMapModule — Starlette+socket.io on :7782
│       └── templates/
│           ├── marauders_map.html         parchment UI + intro overlay + quest chip + BGM
│           ├── hp_characters.js           100 HP characters × 10 lines, baked offline
│           ├── socketio.min.js
│           └── potion_latch.mp3           looped background music
│
├── dimos/robot/custom/                    jamjam's open-vocab control stack
│   │                                      (inherited from jamjam/jamjam — see custom/README.md)
│   ├── modules/
│   │   ├── yoloe_tracking_module.py       open-vocab YOLO-E + BoT-SORT tracking
│   │   ├── bbox_selection_module.py       sole writer of /user_selected_bbox
│   │   ├── target_lock_module.py          debouncing + spatial re-association
│   │   └── go2_startup_self_check_module.py
│   ├── tasks/
│   │   └── bbox_distance_behavior_module.py  pinhole bbox → Twist follower
│   ├── visualization/
│   │   └── detection2d_overlay.py            Detection2DArray → Rerun overlay
│   ├── blueprints/                           autoconnect compositions
│   │   ├── yoloe_target_lock_distance_follow.py    full closed-loop demo
│   │   ├── bbox_distance_follow.py                 minimal task test
│   │   ├── yoloe_keyboard_teleop.py
│   │   ├── yoloe_tracking_test.py
│   │   └── go2_startup_self_check.py
│   └── tests/                                pytest-able module + task tests
│
├── dimos/robot/unitree/
│   ├── connection.py                   ★ FIX: free_walk() now also sends
│   │                                        SwitchJoystick(1027) — fixes the
│   │                                        "dog sways in place" bug.
│   └── go2/
│       └── connection.py               ★ FIX: 2 FreeWalk publishes on init +
│                                        15s heartbeat thread; hasattr-guarded
│                                        so MuJoCo/DimSim/Replay don't crash.
│
├── examples/                           ★ HACKATHON SAMPLES
│   ├── go2_phone_control/                  Phone-as-controller webapp (mock+real)
│   │   ├── server.py                       FastAPI + WebRTC + YOLO+OSNet
│   │   ├── index.html                      GameBoy keypad page
│   │   ├── people.html                     tracked-people map page
│   │   └── halt.py                         standalone E-STOP
│   ├── go2_walk_forward.py             ★ minimal direct-WebRTC scripts
│   ├── go2_walk_backward.py
│   └── go2_person_aware_walk.py
│
├── scripts/
│   └── run-blueprint.sh                ★ auto-logged runner; SIM/REPLAY/ROBOT_IP envs;
│                                          picks mjpython on macOS for MuJoCo's
│                                          main-thread GL context
│
└── docs/research/
    ├── go2_marauders_map_design.md         design rationale for the app
    └── reid_capability_summary.md          re-ID architecture notes
```

`★` marks paths added or modified on this branch. Everything else either came
from `jamjam/jamjam` or `dimensionalOS/dimos` upstream.

---

## The Marauder's Map app — what the page actually does

Open `http://localhost:7782/` after starting a blueprint. The page shows:

| UI element | Behavior |
|---|---|
| **Opening parchment scroll** | Unfurls + per-character "magick-in" reveal of the story beats (~11s, skippable with Esc / corner link) |
| **Live floor plan** | Detected people pinned by stable HP character names; footprint trails |
| **Click a face on the map** | Confirm dialog → BBoxSelectionModule → TargetLockModule → bbox-distance follower drives `nav_cmd_vel` |
| **Click free map space** | Publishes `goal_request` → ReplanningAStarPlanner drives `nav_cmd_vel` toward that point |
| **Roster (right column) click** | Direct select-and-track (no confirm — list is a deliberate picker) |
| **`View all` gallery** | Full-screen face grid; click direct-tracks, interrupting any prior track or nav |
| **Teleop pad (bottom)** | Q/W/E/A/S/D + STOP buttons; `<space>` = STOP; keyboard drives the same handlers as buttons |
| **Quest chip (top-left)** | Always-visible mini brief; click to expand the win conditions |
| **🔊/🔇 (top-right)** | Toggles `Potion Latch.mp3` looped BGM. Muted-autoplay on load → unmutes on first user interaction (the only autoplay shape every browser permits) |

**Exclusivity rule.** The planner and the bbox-follower both publish to
`nav_cmd_vel`. The server-side `select` event publishes `stop_movement=True`
before adopting the new selection, and `navigate` clears any active bbox lock
first. So picking a face cancels a nav goal, and clicking a nav point cancels
a follow — both via a single user gesture.

---

## What this branch adds on top of `jamjam/jamjam`

### A. The app — `dimos/apps/marauders_map/`

A new top-level namespace `dimos/apps/` for hackathon-flavored full-stack
applications. The marauders_map app composes:

- `unitree_go2_basic` (connection + camera + lidar)
- `VoxelGridMapper` + `CostMapper` (world voxel map + 2D parchment walls)
- `YoloeTrackingModule` + `BBoxSelectionModule` + `TargetLockModule` (jamjam)
- `BBoxDistanceBehaviorModule` (jamjam follower)
- `MovementManager` (priority mux: nav vs teleop)
- `ReplanningAStarPlanner` (planner — consumes `goal_request`)
- `ReidMapModule` (this app's web server)

…via `autoconnect(...)` with remappings, plus an LCM transport map. See
`dimos/apps/marauders_map/blueprint.py` for the wiring detail.

### B. Hardware fixes — `dimos/robot/unitree/`

1. **`free_walk()` + `SwitchJoystick(1027)`** in `connection.py`. Without the
   second command, lx/ly are interpreted as body-pose lean (BalanceStand
   semantics) and the dog only sways — even after FreeWalk has been issued.
   The fix is modeled on `enable_rage_mode()` which already does both.
2. **FreeWalk init hardening** in `go2/connection.py`. Two FreeWalk publishes
   on init (with a settle gap) plus a 15-second background heartbeat that
   re-asserts FreeWalk. Guarded by `hasattr(connection, "free_walk")` so the
   simulation backends (`MujocoConnection`, `DimSimConnection`,
   `ReplayConnection`) don't crash.

### C. The runner — `scripts/run-blueprint.sh`

A thin wrapper around `dimos` that:

- Resolves run mode by env var. Priority: **`SIM` > `REPLAY` > `ROBOT_IP`**.
- Uses `mjpython` on macOS when `SIM=mujoco` (MuJoCo's GL window has to own
  the Cocoa main thread).
- Writes every run to `/tmp/dimos-logs/<blueprint>-<timestamp>.log` with a
  header (git HEAD, uncommitted file count, full command). Auto-rotates to
  the newest 20.
- Refreshes `/tmp/dimos-logs/latest.log` symlink on exit.

### D. Examples — `examples/`

- `go2_phone_control/` — standalone phone-as-controller webapp. The Mac
  holds one persistent WebRTC session to the dog; a mobile-friendly page
  posts button taps over HTTP. Two UIs (GameBoy keypad + people tracker)
  plus an E-STOP script (`halt.py`) that opens a fresh WebRTC and forces
  the dog to lie down.
- `go2_walk_forward.py`, `go2_walk_backward.py`, `go2_person_aware_walk.py`
  — minimal direct-WebRTC scripts. No blueprint, no LCM. Useful as the
  smallest possible reproducers for movement debugging.

### E. Research notes — `docs/research/`

- `go2_marauders_map_design.md` — design rationale for the app's web/control split.
- `reid_capability_summary.md` — re-ID architecture (OSNet/TorchReID).

---

## Running it — picking the right mode

| Goal | Command |
|---|---|
| MuJoCo physics simulation | `SIM=mujoco scripts/run-blueprint.sh` |
| DimSim simulation | `SIM=dimsim scripts/run-blueprint.sh` |
| Replay a recorded dataset | `REPLAY=1 scripts/run-blueprint.sh` |
| Real Go2, default IP `192.168.12.1` | `scripts/run-blueprint.sh` |
| Real Go2 elsewhere | `ROBOT_IP=192.168.123.18 scripts/run-blueprint.sh` |
| Run a different blueprint | `scripts/run-blueprint.sh yoloe-target-lock-distance-follow` |

Anything after the blueprint name is forwarded straight to `dimos`.

---

## Data flow (text wiring diagram)

```
              ┌──────────────────────┐
   camera ───▶│ YoloeTrackingModule  │── /color_image/yoloe_detections ─┐
              └──────────────────────┘                                   │
                                                                          ▼
              ┌──────────────────────┐                       ┌────────────────────────┐
 Rerun click ▶│  BBoxSelectionModule │── /color_image/        │   TargetLockModule     │
 Web click ──▶│  (sole writer of     │   selected_bbox      ─▶│  (debounce + reassoc.) │
              │   /user_selected_bbox)│                       └────────────────────────┘
              └──────────────────────┘                                   │
                                                                          │ /color_image/locked_bbox
                                                                          ▼
                                                            ┌──────────────────────────────┐
                                                            │ BBoxDistanceBehaviorModule   │
                                                            │   (bbox → Twist follower)    │
                                                            └──────────────────────────────┘
                                                                          │
              ┌──────────────────────┐                                    │ nav_cmd_vel
  Map click ▶│   ReidMapModule       │── goal_request                     ▼
  (free pt)  │   (web :7782)         │──────────────────▶ ReplanningA*Planner ──┐
              │                       │── tele_cmd_vel ───────────────────┐    │
              └──────────────────────┘                                    ▼    ▼
                                                                ┌──────────────────┐
                                                                │ MovementManager  │ ← priority mux
                                                                │   (nav | tele)   │
                                                                └──────────────────┘
                                                                          │
                                                                          │ cmd_vel
                                                                          ▼
                                                                ┌──────────────────┐
                                                                │  GO2Connection   │ ← WebRTC → real / MuJoCo
                                                                └──────────────────┘
```

Both human-facing selectors (Rerun camera click and Marauder's Map web click)
feed the **same** `BBoxSelectionModule`. The planner and the follower share
`nav_cmd_vel`; exclusivity is enforced in `ReidMapModule`'s socket handlers.

---

## E-STOP — the layered safety story

Order of preference when the dog goes wrong:

1. **Physical remote — `L2 + B`** (damp). Always works.
2. **Web `STOP` button or `<space>` key** on the Marauder's Map page.
3. **`POST /api/halt`** on the phone-control server (`examples/go2_phone_control/server.py`).
4. **`halt.py`** — opens a fresh WebRTC session, sends 5× zero velocity, then `liedown`.
   Survives blueprint hang/crash:
   ```bash
   PATH="$PWD/.venv/bin:$PATH" .venv/bin/python examples/go2_phone_control/halt.py
   ```

---

## License

Apache-2.0, inherited from upstream DimOS.
