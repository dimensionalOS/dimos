# DimOS Mixed-Fleet Simulator

Quadrotors running **real, unmodified PX4 firmware** and legged robots, sharing
**one MuJoCo physics world**, commanded through DimOS.

```
./dimos/simulation/px4_hil/sim.sh start 3 1     # 3 drones + 1 dog
dimos mcp call takeoff_all --arg altitude=6
dimos mcp call rtl_all
./dimos/simulation/px4_hil/sim.sh stop
```

---

## Quickstart on a fresh machine

Tested on Ubuntu x86_64 with Python 3.12 and PX4 v1.16.2. Everything below is
copy-pasteable; total setup is ~15 minutes plus downloads.

**1. Clone and install** (skip the repo's large LFS assets -- this simulator
does not need them):

```bash
GIT_LFS_SKIP_SMUDGE=1 git clone https://github.com/dimensionalOS/dimos.git
cd dimos
uv sync --extra sim --extra drone --extra cpu
echo "DIMOS_TRANSPORT=zenoh" > .env      # the CLI tools read this
```

**2. Build PX4 SITL** (the real autopilot firmware the drones run):

```bash
git clone --branch v1.16.2 --recursive https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot
make -C ~/PX4-Autopilot px4_sitl none_iris
# the build finishes by LAUNCHING one PX4 instance -- Ctrl-C it; sim.sh
# manages its own instances. A different checkout location works via
# PX4_DIR=/path sim.sh start ...
```

**3. Optional but recommended -- the good-looking assets.** Without them the
simulator falls back to primitive shapes that fly and walk identically:

```bash
# real Holybro X500 drone meshes (BSD-3, fetched from PX4-gazebo-models):
.venv/bin/python tools/fetch_x500_meshes.py
# real Unitree Go1 + its trained walking policy (~2 GB Menagerie + git-lfs):
git lfs pull --include "data/.lfs/mujoco_sim.tar.gz"
.venv/bin/python -c "from mujoco_playground._src import mjx_env; mjx_env.ensure_menagerie_exists()"
```

**4. Fly:**

```bash
./dimos/simulation/px4_hil/sim.sh start 2 1 --viewer    # 2 drones + 1 dog
.venv/bin/dimos mcp call takeoff_all --arg altitude=6
.venv/bin/dimos mcp call walk --arg speed_mps=0.5
.venv/bin/dimos mcp call rtl_all
./dimos/simulation/px4_hil/sim.sh stop
```

The bridge logs which asset tier it chose at startup, and `sim.sh` refuses to
say "ready" unless every robot is actually connected and physics is stepping.
On laptops, set the CPU governor to performance first (see Troubleshooting) --
it is worth ~2.4x realtime.

---

## Why this exists

DimOS previously ran two incompatible simulators: drones in **Gazebo** with PX4,
legged robots in **MuJoCo** inside the DimOS process. Nothing could interact
across that boundary, and on a normal laptop Gazebo could not fly three drones
at all — its physics loop is single-threaded, and starved PX4's sensor stream
until the EKF diverged and refused to arm.

This replaces the drone half with MuJoCo while **keeping PX4**. That matters:
PX4 SITL is the same firmware that runs on a Pixhawk, so what flies here is what
flies on real hardware. Losing it would make the simulator a toy.

| | Gazebo | This |
|---|---|---|
| 3 drones actually fly | ✗ | ✓ |
| 3 drones + 1 dog | ~1× realtime | **~10×** |
| drones and legged robots in one world | ✗ | ✓ |
| real PX4 firmware | ✓ | ✓ |
| real Unitree Go1 + trained policy | ✗ | ✓ |

### Speed

Physics-only ceilings, measured 2026-08-21 on the `performance` governor with
real X500 meshes and real Go1s. End-to-end throughput is far lower because PX4
runs in lockstep as a separate process per drone — these are the headroom, not
the number you will see.

| Fleet | Physics ceiling | µs/step |
|---|---|---|
| 1 drone | 313× | 12.8 |
| 2 drones | 192× | 20.9 |
| 3 drones | 162× | 24.8 |
| 5 drones | 109× | 36.6 |
| 1 dog | 82× | 48.5 |
| 2 dogs | 38× | 106.7 |
| 3 dogs | 22× | 181.0 |
| 3 drones + 1 dog | 61× | 65.5 |
| 3 drones + 2 dogs | 33× | 119.9 |
| 5 drones + 2 dogs | 30× | 133.3 |

Two things dominate, and they are different for each robot class. **Drones are
PX4-bound**: each is a separate OS process in lockstep, so the gap between the
ceiling and reality is round-trip latency, not physics. **Legged robots are
physics-bound**: no PX4 at all, but 12 actuated joints with mesh inertias each.

**The CPU governor is worth roughly 2.4×** and does not survive a reboot — see
Troubleshooting. Every number here assumes `performance`.

**`--viewer` costs roughly 20-25%** at full mesh detail. Measured on 2 drones +
1 Go1: **7.7-10.4x headless**, **6.4-7.4x with the window open**. Both bounce
around by a couple of x between samples, so treat these as ranges rather than
figures. Drop the viewer for batch runs; the physics is identical either way.

For reference, the same fleet ran ~10-12x before the full-detail X500 frame was
added -- 179k triangles per airframe is most of the difference, and it is all
rendering, not physics.

---

## How it works

```
                     DimOS  (namespaced: one module per robot)
   SwarmCoordinator ── drone_state / swarm_cmd ── drone1..N , dog1..M
            │                                          │
            │                              MAVLink udp 14540+i   UDP 15000+i
            ▼                                          ▼            ▼
      ┌────────────────────────────────────────────────────────────────┐
      │  PX4 SITL × N   (unmodified firmware, `none_iris` airframe)     │
      └────────────────────────────────────────────────────────────────┘
                     HIL / TCP 4560+i   (PX4 is the client)
      ┌────────────────────────────────────────────────────────────────┐
      │  fleet_bridge.py  —  ONE MuJoCo world                          │
      │    sensors out to all → actuator replies in → one physics step │
      └────────────────────────────────────────────────────────────────┘
```

### The PX4 link (HIL)

Gazebo talks to PX4 through `gz_bridge`, a module compiled into the firmware.
MuJoCo has no equivalent, so we use PX4's other supported path: the generic
**HIL** (Hardware-In-The-Loop) interface `simulator_mavlink`, the same one
jMAVSim and JSBSim use.

* **TCP on 4560 + i**, and *PX4 is the client* — the bridge must be listening first.
* Sim → PX4: `HIL_SENSOR` every step (accel, gyro, mag, baro), `HIL_GPS` at 10 Hz.
* PX4 → Sim: `HIL_ACTUATOR_CONTROLS`, normalised motor outputs.

**The simulator owns PX4's clock.** Every `HIL_SENSOR` calls
`px4_clock_settime()` inside PX4, so PX4 never runs on wall-clock time — it
advances exactly as fast as we feed it. A slow machine makes the world run
*slower*, it does not starve the sensor stream. That single property is why this
scales past the one flyable drone Gazebo manages here.

### Namespaces

Every robot is one module instance under its own namespace, so it gets its own
RPC surface, topics, TF frames and config keys:

```
drone2/px4dronemodule/takeoff      RPC and MCP tool
/drone2/odom                       topic
-o drone2/px4dronemodule.max_altitude_m=15
```

Only two streams are **exposed** (left global) so data crosses the boundary:
`drone_state` (every robot publishes its own snapshot) and `swarm_cmd` (the
coordinator broadcasts; each robot acts only on its own key or `all`).
Everything else — notably `cmd_vel` — stays namespace-local, so one drone's
vision tracker can never fly another drone.

Drones and legged robots publish the **same** state message with a
`robot_class` field, which is what lets `fleet_state` and `count_within` cover
both without special-casing either.

### Frames

The classic way to break a bridge like this.

| | Body | World |
|---|---|---|
| MuJoCo | **FLU** (Forward-Left-Up) | **NWU** (North-West-Up) |
| PX4 | **FRD** (Forward-Right-Down) | **NED** (North-East-Down) |

Both conversions are `(x, -y, -z)`. NWU rather than the more common ENU is
deliberate: PX4 assumes a body at zero yaw points **North**, and a body at
identity in MuJoCo points along world +x. An ENU world puts "forward" at East,
which yaws the magnetometer 90° and diverges the attitude estimate.

---

## Running it

`sim.sh` handles the ordering that otherwise has to be remembered, and waits
until every robot has actually reported before saying `ready`.

```bash
SIM=./dimos/simulation/px4_hil/sim.sh

$SIM start 0 2      # 2 dogs, no drones
$SIM start 0 3      # 3 dogs
$SIM start 2 0      # 2 drones, no dogs
$SIM start 1 1      # 1 drone + 1 dog
$SIM start 3 1      # the full demo fleet

$SIM status
$SIM log bridge     # or: daemon, px4_0
$SIM stop
```

### Seeing it

The simulator runs **headless by default** — no window opens. Add `--viewer`
(anywhere in the arguments) to get the MuJoCo window:

```bash
$SIM start 3 1 --viewer
```

In the window: **Tab** cycles cameras (free -> `field` -> a chase cam on each
vehicle), **double-click** a vehicle then drag to orbit it, **S** toggles
shadows, **space** pauses. Closing the window now just drops the sim to
headless — physics, PX4 and the fleet keep running; use `$SIM stop` to shut
down.

Each drone carries an accent colour (drone1 red, drone2 cyan, drone3 amber) on
its beacon and stripe, and lands on a matching ringed pad, so you can tell which
vehicle is flying which lane of a sweep. Those markings are visual only
(`contype=0`, zero mass) and cannot affect the physics — verified by diffing the
compiled model against the unstyled scene: identical masses, inertias, DOF, and
collidable-geom counts, and under 5% on step cost.

The ground checker is 2 m per square; at a typical 8 m sweep altitude that is
what gives you usable motion parallax and a way to eyeball distance.

### Commands

Fleet-wide (no namespace — they act on the whole fleet):

```bash
dimos mcp call list_drones
dimos mcp call preflight_check
dimos mcp call fleet_state
dimos mcp call takeoff_all   --arg altitude=6
dimos mcp call grid_sweep    --arg corner_b_north=40 --arg corner_b_east=30
dimos mcp call investigate   --arg north=30 --arg east=10 --arg num_drones=2
dimos mcp call line_formation --arg altitude=7 --arg spacing_m=6
dimos mcp call count_within  --arg drone=drone1 --arg radius_m=100
dimos mcp call goto_drone    --arg drone=drone1 --arg north=25 --arg east=0 --arg altitude=6
dimos mcp call rtl_all
dimos mcp call emergency_land_all      # controlled descent
dimos mcp call kill_all                # last resort: motors off, they fall
```

Per robot. A skill offered by only one robot keeps its bare name; when several
robots offer it the name is qualified. Note `state`, `stop` and `goto` exist on
*both* robot classes, so in a mixed fleet they are always qualified:

```bash
dimos mcp call crouch                          # only one dog -> bare
dimos mcp call dog2/leggedsimmodule/crouch     # two or more dogs -> qualified
dimos mcp call drone2/px4dronemodule/takeoff --arg altitude=5
dimos mcp call dog1/leggedsimmodule/state      # `state` exists on both classes
dimos mcp list-tools | grep '"name"'           # see what exists right now
```

Legged robots:

```bash
dimos mcp call walk --arg speed_mps=0.3 --arg turn_rate_rads=0.1   # + turn = right
dimos mcp call halt          # stop walking (NOT `stop` -- see below)
dimos mcp call crouch      # and stand
```

Legged robots are **real Unitree Go1s driven by DimOS's trained ONNX policy**
when the assets are present (see below). Measured envelope of that policy in
this world: **0.94 m/s** forward (1.0 commanded), **0.31 m/s** reverse,
**0.55 rad/s** yaw, and it holds a turn and a cruise at the same time.

**It has a low-speed deadband: a commanded 0.18 m/s produces no motion at all.**
0.30 walks, 0.45 walks well. Anything under ~0.25 m/s is a stop, not a slow
walk — this is why `goto`'s speeds are set where they are, and it is the first
thing to check if a legged robot ignores a command.

`crouch` / `set_height` do nothing on a real Go1: the shipped policy is a
flat-ground velocity controller with no height input. The bridge logs that
rather than silently accepting the command. They still work on the primitive
fallback below.

### Legged assets

Without MuJoCo Menagerie and the trained policy, dogs fall back to a
primitive box-and-capsule quadruped with a hand-written trot (0.37 m/s,
0.12 rad/s, and it falls at closed-loop reversals). To get the real robot:

```bash
sudo apt install git-lfs && git lfs install
git lfs pull --include "data/.lfs/mujoco_sim.tar.gz"      # the trained policy
.venv/bin/python -c "from mujoco_playground._src import mjx_env; mjx_env.ensure_menagerie_exists()"
```

Menagerie is a plain 2 GB `git clone` (no LFS, no sudo) and supplies the mesh;
the LFS archive supplies `unitree_go1_policy.onnx`. **Both are required** —
the mesh alone gives a robot that cannot walk. The bridge logs which backend it
chose on startup (`legged robots: real Unitree Go1 + trained policy xN`).

### The operating envelope

**5 drones + 5 dogs, maximum.** `sim.sh` refuses more. That is a policy, not a
performance limit -- the validated demo ladder is 2 drones, then 2 dogs, then
2 drones + 1 dog, and everything beyond 5+5 is unvalidated territory.

### The staged mission

The one-command version of "the drones go in first, then the dog":

```bash
dimos mcp call takeoff_all --arg altitude=6
dimos mcp call sweep_then_ground --arg ground_delay_s=30
dimos mcp call mission_status              # [air sweep] / [ground transect] / [complete]
dimos mcp call rtl_all                     # when done -- drones hold at lane ends
```

Drones sweep the marked 40 x 30 m field immediately; after `ground_delay_s`
(wall-clock) the dog walks the centre transect -- entry, centre, far edge --
and returns to where it started. `emergency_land_all` and `kill_all` abort the
ground stage. The delay is wall-clock on purpose: the operator watches the wall
clock, not sim time.

### The world

The 40 x 30 m sweep field is marked with white lines (NED (0,0)..(40,30)) so a
grid sweep is legible in the viewer. Trees are visual-only and parked well away
from every mission path -- a trunk the dog could walk through must never sit
where a route goes. The one collidable obstacle cluster (three crates, NED
(12,-14), west of the field) exists so the world has at least one honest
obstacle; no default route goes through it.

### Natural-language control

```bash
export OPENAI_API_KEY=sk-...     # required BEFORE start; the daemon dies without it
SIM_BLUEPRINT=mixed-fleet-agentic ./dimos/simulation/px4_hil/sim.sh start 0 2 --viewer
dimos humancli
```

**Run `humancli` from the repo root** (`~/Work/dimos`) -- the `.env` there
pins `DIMOS_TRANSPORT=zenoh` for every process. Without it, humancli on Linux
defaults to LCM, connects to nobody, and shows "thinking..." forever while the
daemon (on zenoh) never hears it.

**Reopen `humancli` after every `sim.sh start`.** A humancli window from a
previous run still says "Connected" but talks to a dead session -- its
messages go nowhere and it shows "thinking..." forever. Ctrl-C, run
`dimos humancli` again, done. (`dimos agent-send "..."` is the quick way to
test the agent without a chat window.)

### Wind

```bash
SIM_WIND_N=3 SIM_WIND_E=0 SIM_GUST_STD=1 $SIM start 2 0
```

Constant mean wind (m/s, world NED) plus an Ornstein-Uhlenbeck gust process,
entering through the airspeed the rotor drag sees. Zero by default. Verified:
at 3 m/s mean + 1 m/s gusts the fleet holds position within ~0.1 m and lands
clean. The drone side also models motor spool lag, rotor H-force drag and
ground effect -- constants and their approximations are documented at the top
of `hil_bridge.py`.

### Control boundaries

Run `dimos mcp call boundaries` to see every active limit. Two layers, catching
different failures:

| Boundary | Layer | What it catches |
|---|---|---|
| Operating radius 250 m | dispatch-time (coordinator + dog `goto`) | a bad command, before anything moves |
| Altitude ceiling | dispatch-time | same |
| Separation floor 2 m | dispatch-time admission | a waypoint ending too close to another aircraft |
| PX4 geofence (hold at 260 m / 45 m) | **in-flight, inside the autopilot** | `set_velocity` drift and anything dispatch cannot see (aircraft only) |
| Ground fence (halt at 250 m) | **in-motion, in each legged module** | a `walk` command carrying a dog out of the operating area -- the ground analogue of the PX4 fence. Walking back in is allowed; continued escape re-trips it |
| Ground proximity (halt at 0.6 m) | **in-motion, in the simulator** | two ground robots converging -- including ones already moving, which no dispatch check can see. Re-arms at 1.2 m |

Legged robots have two motion commands with deliberately different semantics:
`move(forward_m, right_m)` is body-relative and **holds the heading** (the Go1
strafes), so "left 5 then forward 2" composes the way a person means it;
`goto(north, east)` turns toward its target -- efficient for distance, but it
changes what "forward" means afterwards. Both are closed-loop **inside the
simulator at physics rate**: control from the DimOS side on wall-clock ticks
acts on 6-12 sim-seconds of stale state at these realtime factors and is
unstable (measured: a 5 m strafe wandered 140 m).
| Airborne gate | dispatch-time | OFFBOARD to a grounded drone (trips a failsafe that blocks the next arm) |
| Datalink-loss failsafe (`NAV_DLL_ACT=2`) | **in-flight, inside the autopilot** | the laptop/DimOS dying mid-flight -- each drone returns and lands on its own authority |

With the datalink-loss failsafe configured, PX4 refuses to arm without a live
ground station -- so the SwarmCoordinator **is** the fleet's ground station: it
binds the shared GCS port (14550, the QGroundControl convention every PX4
instance targets), discovers each vehicle from its stream, and answers with
2 Hz heartbeats. Kill DimOS mid-flight and those heartbeats stop; every drone
holds 5 s, returns, lands and disarms entirely on its own authority — the
exact behaviour wanted on real hardware, exercised in sim. (Running a real
QGroundControl instead? Set `-o swarmcoordinator.gcs_port=0` so they do not
fight over the port.)

The fence radius (260 m) is deliberately just outside the dispatch radius
(250 m) so the polite refusal always fires first; the fence is the backstop,
not the interface. The separation floor is admission control, **not** in-flight
collision avoidance — two drones already converging are not re-checked.

**Order matters: `takeoff_all` before any position maneuver.** `grid_sweep`,
`line_formation`, `investigate` and `goto_drone` engage OFFBOARD; doing that to
a grounded, disarmed vehicle trips a PX4 failsafe that then blocks the *next*
arm. The simulator refuses and tells you, rather than failing later.

---

## Suggested first session

```bash
SIM=./dimos/simulation/px4_hil/sim.sh
D=.venv/bin/dimos

$SIM start 0 2                                     # two dogs
$D mcp call list_drones
$D mcp call dog1/leggedsimmodule/crouch
$D mcp call dog1/leggedsimmodule/stand

$SIM start 0 3                                     # three dogs
$D mcp call fleet_state

$SIM start 2 0                                     # two drones
$D mcp call preflight_check
$D mcp call takeoff_all --arg altitude=5
$D mcp call fleet_state
$D mcp call rtl_all

$SIM start 1 1                                     # one of each
$D mcp call takeoff_all --arg altitude=5
$D mcp call count_within --arg drone=drone1 --arg radius_m=100   # sees the dog
$D mcp call crouch
$D mcp call rtl_all
$SIM stop
```

---

## Troubleshooting

**Drones will not arm; PX4 logs `High Accelerometer Bias` or `ekf2 missing data`.**
Check the CPU governor first — it does **not** survive a reboot:

```bash
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor    # want: performance
sudo cpupower frequency-set -g performance
```

On `powersave` this machine pins every core near 1.5 GHz against a 4.7 GHz
capability, even on AC with cool temperatures. MuJoCo tolerates it far better
than Gazebo did — 3 drones plus a dog still run at ~3.3× — but performance mode
is free speed.

**`REJECTED: no aircraft are armed`.** Working as intended. Call `takeoff_all`
first; see the ordering note above.

**Nothing takes off after an RTL.** Fixed — `takeoff` now clears the latched
`AUTO.RTL` nav state before arming. If you see it again, `$SIM stop && $SIM start`.

**Drone flips immediately on arming.** A rotor yaw-torque sign is inverted.
PX4 computes `moment = ct * position.cross(axis) - ct * km * axis` with
`axis = (0,0,-1)` in FRD, giving `torque_z(FRD) = +km * thrust`; FRD z points
down, so that is *negative* about MuJoCo's z. See the comment in `scene.py`.

**Weird estimator behaviour after fiddling with parameters.** PX4 transports
INT32 parameters bit-cast into a float field — sending a literal `1.0` stores
`1065353216`. `sim.sh` wipes each instance's `parameters.bson` on start for
exactly this reason.

**Battery dies about a minute in.** `SIM_BAT_DRAIN` defaults to 60 s full to
empty. `sim.sh` runs `sim_params.py` automatically; check
`/tmp/dimos-sim/params.log` if a run ends early.

---

## Files

| File | What |
|---|---|
| `sim.sh` | Bring the whole stack up and down |
| `fleet_bridge.py` | The simulator: one MuJoCo world, N PX4 links, M legged links |
| `hil_bridge.py` | HIL protocol, frame conversions, and one vehicle's link |
| `scene.py` | Builds the world — single source of rotor geometry and the quadruped |
| `../../robot/drone/px4_drone_module.py` | DimOS module for one drone |
| `../../robot/legged/legged_sim_module.py` | DimOS module for one legged robot |
| `../../robot/drone/px4_swarm_coordinator.py` | Fleet state, guardrails, maneuvers |

---

## Known limits

* **The quadruped is solid with the trained policy, weak without it.** With a
  real Go1, `goto` reaches **6 of 6** spread targets (worst error 0.62 m) and
  the robot did not fall in any tested command, including reversals and targets
  directly behind it. On the primitive fallback it is the old hand-tuned trot:
  4 of 6 targets, and it topples on large sustained turns.
* **The Go1 policy is Go1-only.** DimOS ships trained policies for Go1 and G1.
  Menagerie also contains a Go2 mesh, but there is no Go2 policy — so this is a
  good simulated demo, not sim-to-real transfer for Go2 hardware.
* **A real Go1 costs about 2x the physics time** of the primitive dog per step
  (measured): fewer contacts, but 12 actuated joints with mesh inertias. At
  3 drones + 2 dogs the physics ceiling is ~53x, still far above what PX4
  lockstep delivers, so it is affordable.
* **The legged "stop walking" tool is `halt`, not `stop`.** `Module.stop()` is
  the framework's teardown RPC — it closes the module's RPC, tools and event
  loop. A `@skill` named `stop` shadowed it, so the module never tore down, and
  `_on_swarm_cmd` calling `self.stop()` on a `land`/`hold` broadcast would have
  killed the module outright once the shadowing was removed. Never add a skill
  named `stop` to a Module subclass.
* **No aerodynamics.** MuJoCo models no rotor drag, downwash or ground effect.
  Fine for separation and coordination work; not a flight-dynamics model.
* **The separation guardrail is a dispatch-time admission check**, not
  collision avoidance. It rejects a commanded waypoint that would end too close;
  it does not re-check two drones already converging in flight.
* **`SIM_DRONES` / `SIM_DOGS` must match** what the bridge was started with, or
  DimOS waits for robots that do not exist. `sim.sh` keeps them in step.
