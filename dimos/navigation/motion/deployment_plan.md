# Deploying the motion stack onto the robot

The time-critical half of the motion stack — local planner, trajectory
controller, cmd_vel mux — runs on the Go2 itself as one baked binary. Mapping
stays on the laptop. Quick start below; the reasoning, measurements and sharp
edges follow.

## Quick start: run it on your Go2

From the repo's dev shell (nix provides the whole toolchain — rust with the
aarch64 target, zig, cargo-zigbuild; nothing to install):

    dimos bake motion_planner trajectory_follower cmd_vel_mux go2_tf \
        -o motion-host --builder zigbuild --target aarch64-unknown-linux-gnu.2.31

(`go2_tf` publishes the go2's own tf tree robot-side; the host refuses to start
if the stdin config names a module it wasn't baked with, so keep the bake list
and `motion-host.json` in sync.)

    scp target/dimos-bake/motion-host/target/aarch64-unknown-linux-gnu/release/motion-host \
        root@<robot>:/root/motion-host/
    scp dimos/navigation/motion/motion-host.json root@<robot>:/root/motion-host/

`motion-host.json` (this directory) is the AUTHORITATIVE deployed config —
edit it here, push it with the binary, never hand-edit the robot's copy.
Three debugging sessions have burned time on config skew between the robot and
a local guess.

On the robot, install `dimos-motion-host.service` (this directory) — it carries
the load-bearing zenoh wiring: go2web runs as the zenoh ROUTER on 7447, and the
host connects to it as a client over loopback
(mode `client`, connect `tcp/127.0.0.1:7447` — that is where odometry comes
from). The host listens on nothing; the router does all forwarding. Those
settings live in the config's `session` block, NOT in the unit's environment:
the rust module reads only `DIMOS_TRANSPORT` from the env, and with no `session`
block it opens zenoh's own defaults, which makes it a peer with multicast
scouting — and a router forwards to clients, not to peers. Config arrives as one
JSON line on stdin (`/root/motion-host/motion-host.json` — see "Config is the
sharp edge" below before writing it).

    systemctl enable --now dimos-motion-host

Laptop side, one dial — the router forwards to everything behind it:

    dimos --robot-ip <robot> run go2-zenoh-motion-local

`-local` is the point: it is `go2-zenoh-motion` MINUS the three modules the host
now runs. Running plain `go2-zenoh-motion` against a live host gives you two
planners, two followers and two muxes publishing onto the same topics.

The `.2.31` in the bake triple pins the glibc floor to the Go2's Ubuntu 20.04
(glibc 2.31); the artifact links at ≤2.30, so it runs there regardless of how
new your build machine is.

## The cut

```
laptop                                   robot
------                                   -----
RayTracingVoxelMap  --- local_map ----->  MotionPlanner        (5 Hz)
MLSPlannerNative    --- planner_path -->  TrajectoryFollower   (10 Hz)
GoalRelay                                 CmdVelMux
MovementManager (click half)              go2web bridge
vis_module
                    <-- lidar, odom ----
                    --- tele_cmd_vel -->
```

Everything on the right is one baked host binary, plus the bridge the robot
already runs.

### Why not the whole stack

The raycaster is the expensive module and we have not yet proven the robot can
carry it. Until then it stays on the laptop and we accept that `local_map`
crosses the link.

### What this cut does and does not buy

It does **not** remove a wire crossing from the perception→action loop. Today
the loop crosses twice (lidar out, `cmd_vel` in); after the move it still
crosses twice (lidar out, `local_map` in), and the message got much bigger.

What it buys is **jitter immunity on the last stage**: the follower ticks at a
steady 10 Hz off a locally-held path instead of the robot receiving `cmd_vel`
in bursts whenever the link hiccups.

### The end state

Move the raycaster to the robot as well and the loop is fully local: the link
then carries lidar for visualization, goals, teleop and telemetry, none of it
in the control path. MLS can stay on the laptop — its path is small and slow,
and a stale global route is benign, because the local planner only takes a
carrot along it.

Notably this costs no extra bandwidth over the current cut: MLS consumes
`local_map` too (its `global_map` is remapped off), so the cloud crosses the
link either way — inbound now, outbound then. The blocker is purely whether
the robot's SBC can afford the raycaster. **It can, for short missions — see
below.**

## How much slower is the robot? (measured 2026-08-03)

The Go2 board is a 7-core Cortex-A55 at 1.42 GHz (1.8 GHz max), Ubuntu 20.04,
glibc 2.31, no rust toolchain. The laptop is a 20-thread Ryzen AI 9 365.

**A wall-time ratio between the two machines is not a usable measure.** The
raycaster's `update_map` is rayon-parallel, so wall time is a function of how
many cores the pool happened to get — 8.3 on the laptop, 2.9 on the robot —
and the ratio moves with the machine's width, not its speed. Two numbers are
portable and worth remembering:

|                                              | laptop             | robot                | factor             |
|----------------------------------------------|--------------------|----------------------|--------------------|
| single-thread CPU-s per 310 s of lidar       | 100.8              | 310.8                | **3.08x**          |
| single-thread per-frame p50 / p95 / p99 (ms) | 28.9 / 64.7 / 87.3 | 99.7 / 187.7 / 237.7 | 3.4x / 2.9x / 2.7x |

**Rule of thumb: one Go2 core ≈ one third of a laptop core on this workload.**
Use the single-thread CPU-time ratio when sizing anything else for the robot;
use per-frame p99 when the question is whether a loop closes on time.

Do not use the multi-threaded ratio (fps 112.7 vs 21.6, ≈5.2x). Rayon's spin
overhead more than doubles the CPU-seconds the job costs when the pool is
wide — the laptop burned 227.8 CPU-s to do the same work it does in 100.8
CPU-s on one thread — so that 5.2x is a statement about core counts, not speed.

### What the raycaster actually costs on the robot

Replaying `data/mid360_athens_stairs.db` (3093 clouds at 9.99 Hz, ~4.6k
points/cloud, 310 s) through the real `RayTracingVoxelMap` at wall-clock speed,
with the `go2_zenoh_raycaster` config (`voxel_size=0.08`, `emit_every=10`,
`global_emit_every=100`, `support_min=4`):

|                  | laptop  | robot (rayon=7) | robot (rayon=2) |
|------------------|---------|-----------------|-----------------|
| clouds processed | 99.9%   | 97.0%           | 91.8%           |
| sustained rate   | 9.98 Hz | 9.69 Hz         | 9.17 Hz         |
| cores consumed   | 0.83    | 1.46            | 1.02            |
| peak RSS         | 305 MB  | 304 MB          | 304 MB          |

Robot idle baseline at the time: **2.80 of 7 cores busy, 4.20 free** (30 s
`/proc/stat` sample), with the go2web bridge, `basic_service`, `mcf_main`,
`dimos-pointlio`, `videohub` and friends running. So the raycaster fits — it
adds ~1.5 cores to a machine with ~4.2 free — but three things bound the claim:

- **The tail exceeds the frame budget.** p99 is 140 ms against a 100 ms
  budget at 10 Hz. Nothing overflows the 128-deep input queue. The missing 3%
  (and 8% at rayon=2) was the cloud being dropped for want of a pose — but
  load was never the whole story, and blaming it hid how little margin there
  was. `select!` polls its ready arms in random order, so a cloud outruns the
  odometry queued beside it whatever the load; and the go2 sweep measures
  100.8 ms, not the nominal 100, which puts the previous sweep's pose 0.8 ms
  outside `POSE_MATCH_TOLERANCE_S`. There was no second chance to lose.
  Measured on 20260807-190044.mcap at 6.1% of clouds, on an idle laptop.
  The module now holds an unmatched cloud for its own sweep's pose instead of
  discarding it, so re-measure this row before quoting it.
- **Cost grows with the map, without bound.** Single-threaded per-frame mean
  went 73 ms → 161 ms over the 5-minute recording as the map reached 366k
  voxels; `emit_points` walks every voxel on each emit. A 5-minute run fits.
  A 20-minute one probably does not, at this `voxel_size`.
- **Throttling rayon does not pay.** rayon=2 saves 0.44 cores but drops 8% of
  clouds. If the raycaster moves to the robot, give it the full pool.

Verdict: **option C is affordable for short missions and needs a bound on map
growth before it is affordable for long ones.** Alongside a 5 Hz planner and a
10 Hz controller the projected total is ~5.3 of 7 cores — real but thin
headroom, and the growth curve is what will break first.

## Consequences we have to handle

### Staleness — the link was the deadman

Today a dropped link stops `cmd_vel` and the bridge watchdog halts the robot.
Once the loop runs on the robot, that accidental safety is gone: the planner
would keep replanning on a frozen map while the follower tracks the result at
cruise speed. Worse, the speed governor reads clearance from that same stale
cloud, so it stays confident.

Three guards:

| where                | rule                                                                             | status                        |
|----------------------|----------------------------------------------------------------------------------|-------------------------------|
| `MotionPlanner`      | `local_map` older than `max_map_age_s` (5 s) → publish the single-pose hold stub | **done** (`fd5c873a5`)        |
| `TrajectoryFollower` | `path` older than N → zero the twist                                             | todo                          |
| `CmdVelMux`          | `nav_cmd_vel` stale → zero `cmd_vel`                                             | todo, part of writing the mux |

Staleness is measured from **arrival**, not `msg.ts`: the mapper's clock is not
the robot's, and what these guard is how long since the producer was last heard
from.

### Splitting MovementManager

`MovementManager` is a click-to-goal relay *and* a velocity mux, and the two
halves land on opposite sides of the link. The seam is not clean, because
`_on_teleop` does two unrelated things: it preempts `cmd_vel` (robot side) and
calls `_cancel_goal()`, which publishes `stop_movement` (consumed by the
follower — robot side) and a NaN goal (consumed by GoalRelay/MLS — laptop
side).

So one keystroke has to land on both sides:

- **rust, robot**: in `nav_cmd_vel`, `tele_cmd_vel` → out `cmd_vel`,
  `stop_movement`
- **python, laptop**: in `clicked_point`, `tele_cmd_vel` → out `goal`,
  `way_point` (including the NaN cancel)

Both subscribe `tele_cmd_vel`. Teleop originates on the laptop, so the python
half gets it for free, and `stop_movement` stays co-located with its only
consumer. Do not route the cancel back from the rust half.

### tf on the robot

The planner and the follower both need tf. LIO stamps its odometry at the
SENSOR (`mid360_link`), so the pose it carries is the lidar's, not the robot's
— 0.30 m ahead of the body and 0.16 m above it on this rig. Both modules
resolve it through `tf_pose::OdomBasePose`, the twin of
`dimos/navigation/tf_pose.py`: look the static `mid360_link → base_link` leg up
once, cache it, compose it onto every message, and DROP messages until the leg
arrives.

The leg comes from `GO2Zenoh`, which stays on the laptop and publishes the
static mount tree (plus the live `odom → mid360_link` edge) onto tf. So the
baked host needs a `tf` topic in its map — `Builder::tf()` claims the port, and
the module refuses to start without it — and a link back to the laptop. It
still needs **no on-robot equivalent of GO2Zenoh**: the bridge publishes
`odometry`, the laptop publishes `tf`, and the host subscribes both.

The failure mode is quiet but not dangerous: a host that never sees tf holds
its pose at `None` and plans nothing, rather than planning off-heading. One
"dropping odometry" line per outage says so.

(Cleanup while we are here: `ControllerConfig.frame_id` and the
`TrajectoryController` docstring claim the controller does a live tf lookup of
that frame. It does not and never has. Delete it rather than port a lie.)

## What has to be built

Nothing on the robot side exists yet. `dimos bake --list` currently registers
only `ray_tracing` and `mls_planner` — which is to say, the only two bakeable
modules are the two that stay on the laptop.

Every crate must be a **standalone native module**, not just a bake input. The
registry format enforces this: `python` is a required key in the metadata
table, so a bake module must name a python `NativeModule` wrapper. Each crate
gets the raycaster's shape:

```
[[bin]] name = "..." path = "src/main.rs"   ← standalone shim over run_module_core
src/module.rs                                ← the struct, linkable into a host
[package.metadata.dimos.module.<id>]         ← registry entry
  python = "...:WrapperClass"                ← the standalone wrapper
```

Baking is then purely additive: develop and debug each module standalone
through its python wrapper, bake only when you want one binary on the robot.

### Build order

1. **`motion/profile/rust/`** — the z-band nearest-neighbour clearance query
   plus `encode_precision`/`decode_ceilings` (`control/profile.py`,
   `control/world.py:path_clearance`). The only real algorithm work, and both
   adapter modules need it. A uniform grid hash likely beats a KD-tree here
   given everything upstream already thinks in voxels. Same discipline as the
   existing law crates: pyo3-optional, parity-tested against the python.
2. **`motion/adapter/rust/`** — module structs wrapping `dimos-motion2-target`
   (planner) and `dimos-motion2-tc` (controller), with `main.rs` shims,
   `[[bin]]` entries, metadata, and python `NativeModule` wrappers.
   `MotionPlanner`/`TrajectoryFollower` are plain python `Module`s today, so
   even the wrapper classes are missing.
3. **`movement_manager/rust/`** — the mux, plus splitting the click half off in
   python.

The two existing law crates (`dimos-motion2-target`, `dimos-motion2-tc`) stay
**pure algorithm**. They are deliberately dependency-light and parity-locked to
python; do not drag `dimos-module`, `lcm-msgs` and tokio into them. The adapter
crate is the transport shell, mirroring the python layout exactly.

Open layout question: one adapter crate with two `[[bin]]` entries, or two
crates sharing the `profile` crate. Start with one and split if it chafes.

## Deployment mechanics

### The baked host runs standalone

`NativeModule` spawns a **local** subprocess (`subprocess.Popen`) — no ssh, no
remote exec. So `baked_host()` in a laptop blueprint would spawn the binary on
the laptop. The robot host therefore runs standalone: bake for aarch64, copy
it over, systemd unit, config from stdin. The laptop blueprint becomes a
separate blueprint that declares the robot's topics as external.

Cross-compilation is already supported: `--builder cross|zigbuild --target
<triple>`.

### Baking the motion host (the working recipe)

The host runs **on the robot itself**, same machine as the go2web bridge —
that is why it dials the bridge over loopback and listens on its own port
(see the deployment comment in `go2/zenoh/blueprints.py`).

No host-side toolchain setup. The nix dev shell carries the whole thing —
rust 1.97.1 pinned through the `rustOverlay` input (oxalica/rust-overlay) with
`aarch64-unknown-linux-gnu` in its target list, plus `cargo-zigbuild` and `zig`
from nixpkgs. Inside `nix develop` (or the direnv shell), `cargo`, `rustc`,
`zig` and `cargo-zigbuild` all resolve to `/nix/store/...` — which also shadows
the `~/.local/bin/cargo` niceness shim that used to break rustup proxying. No
rustup, no pipx, no PATH fronting.

Two environment traps hit on 2026-08-05, both dead as of the flake change: zig
hiding inside a pipx venv with no bin, and that `cargo` shim canonicalizing the
rustup proxy down to raw `rustup`. If you ever bake outside the nix shell, they
come back.

Then:

    dimos bake motion_planner trajectory_follower cmd_vel_mux go2_tf \
        -o motion-host --builder zigbuild --target aarch64-unknown-linux-gnu.2.31

The `.2.31` suffix pins the glibc floor the binary links against — check the
robot's (`ldd --version` on it) and pin at or below. Artifact lands at
`target/dimos-bake/motion-host`; copy to the robot, run under
`dimos-motion-host.service` with the zenoh env from the blueprint comment
(`DIMOS_ZENOH_CONNECT=tcp/127.0.0.1:7447` to reach the bridge, a fixed
`DIMOS_ZENOH_LISTEN` port of its own so the laptop can dial it).

`--builder zigbuild` is what makes the glibc pin possible at all; the aarch64
std the flake ships is what makes any aarch64 target work. Outside the nix
shell, a plain `--target aarch64-unknown-linux-gnu` against a host rust with no
cross std fails with `can't find crate for core` — that is the missing std, not
a bake bug.

### Config is the sharp edge

`--emit-config` builds the standalone stdin blob from **python class
defaults**, not from blueprint values. For this stack that is not cosmetic:

> The follower's eleven tuned `Embodiment.control` numbers and the planner's
> `replan_hz` / `goal_lookahead_m` all come from the blueprint, not from the
> class. A host baked today runs the class defaults. Silent, and it presents as
> a controller bug.

The mount used to be the worst case here — a config quaternion that defaulted
to identity, i.e. "the sensor is already level", true of no robot that tilts
its lidar. It is now read off tf instead, so it cannot be defaulted wrong: the
modules either have the leg or they hold. That removes the sharpest edge but
not the class of problem.

Either teach bake to read a blueprint, or hand-write the stdin JSON and treat
it as the deployment artifact — but decide deliberately. `mount_rotation` is
the test case.

What `motion-host.json` here actually is, since 2026-08-20: `--emit-config`'s
output (which carries the `graph` stamp, so a config baked for another graph is
refused) with the one value the deployment tunes — `max_speed` 0.7 — and a
`session` block. Its per-module `topics` maps are gone: the bake bakes the
wiring into the binary and stdin topics are overrides only, so restating them
was just another thing to drift.

It is ONE LINE, and it has to be: `read_launch_config` reads a single line, so a
pretty-printed blob crash-loops the host with `EOF while parsing an object at
line 1 column 1`. That is why the file is excluded from the repo's
`pretty-format-json` hook.

### Blueprint-as-arg

The eventual fix, and the reason it is worth doing: the blueprint is the only
place the tuned wiring and config actually live. The remappings
(`MLS.path→planner_path`, `global_map→global_map_unused`) are exactly the
`--remap` flags one would otherwise retype, where drift is a silent
misconnect.

Two design points:

- **Partition, do not require purity.** A blueprint is always mixed;
  `vis_module` will never be rust. Bake a named subset and leave the rest
  external.
- **Emit the replacement blueprint** — the `baked_host(...)` call with members,
  remaps and configs — so the binary and the python driving it cannot drift.

Known limit it will surface immediately: `select_modules` refuses duplicate
ids ("one instance per host, per-instance namespacing is not implemented"),
while blueprints can carry two instances of a class under different namespaces.
Not blocking for this stack.

### Zenoh links

The baked host needs links **both** ways: locally to the bridge (`odometry`,
`cmd_vel`) and across to the laptop (`local_map`, `planner_path`,
`tele_cmd_vel`). Given the known multicast problems on the 10.55.1.x LAN,
assume explicit connect endpoints rather than scouting — especially for
something starting at boot.

## Open questions

- ~~**Can the robot host the raycaster?**~~ Measured 2026-08-03: yes at 1.46
  cores and 97% of clouds, for a 5-minute map. See "How much slower is the
  robot?" above. What is still open is **bounding map growth** — per-frame cost
  doubled over 5 minutes and nothing caps it.
- **Plan-to-plan discontinuity.** Once the follower ticks steadily off a local
  path, what is left to twitch is the 5 Hz replan snapping the path between
  cycles. Does the battery score path-switch discontinuity across replans? If
  not, that is the metric to add — it is a planner property, and no amount of
  deployment work fixes it.
- **Live cloud size vs battery worlds.** `TargetEpisode.plan` builds its SDF
  grid over the pose+goal+cloud bounding box at 0.05 m, so cost tracks the live
  local map's extent and density, which the synthetic scenario worlds do not
  represent. Worth pushing one recorded `local_map` through `plan()` on the
  laptop if the follower ever stutters.
