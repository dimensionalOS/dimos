# Running the motion stack on your Go2

The local planner, trajectory follower, cmd_vel mux and the go2 tf tree run
on the robot as one baked binary; mapping stays on the laptop. Why the stack
is cut there, and what it measured: [motion-deployment.md](/docs/platforms/quadruped/go2/motion-deployment.md).
Each step ends with a check — every one has caught a real deployment mistake.
`<robot>` is your Go2's IP; `ssh root@<robot>` is written `ssh go2` below,
i.e. an ssh-config alias for root on it.

## 0. What you need

- this repo, inside its dev shell (direnv/nix — the shell carries the whole
  rust cross toolchain; nothing to install by hand)
- the [go2web](https://github.com/dimensionalOS/go2web) repo (the robot-side
  bridge: Point-LIO, recorder, zenoh)
- a Go2 with the Livox Mid-360 mount, and root ssh to it

## 1. go2web (the bridge) onto the robot

From your go2web checkout:

```sh
scripts/build-nix-musl.sh          # aarch64 static binary -> ./dimos-helper
scripts/deploy.sh go2              # copies binary + dimos-helper.service, restarts
```

**Check**: `ssh go2 journalctl -u dimos-helper -n 20` — the service is up and
logs one clock line per start:
`[pointlio] sidecar clock is already unix ... publishing stamps unchanged` (PTP
synced, step 2 done) or `... internal epoch ... mapping published stamps to
unix` (fallback estimator — fine, but do step 2).

## 2. PTP time sync for the Mid-360

The lidar free-runs on a power-on clock unless a PTP master exists on its
segment; it auto-slaves, no lidar-side config. Full story + gotchas:
`go2web/docs/mid360-ptp.md`.

```sh
ssh go2 apt install -y linuxptp    # one package, no deps
scp scripts/ptp4l-mid360.conf go2:/etc/linuxptp/        # from go2web
scp scripts/ptp4l-mid360.service go2:/etc/systemd/system/
ssh go2 'systemctl daemon-reload && systemctl enable --now ptp4l-mid360'
ssh go2 systemctl restart dimos-helper    # re-latch the clock mode
```

**Check**: the dimos-helper clock line now says `already unix`, and the offset
is tens of ms, not 1.7e9. (`time_type=1` from the lidar is NOT proof — it
latches forever once synced; only the stamps tracking wall time prove it.)

## 3. Bake the motion host and put it on the robot

From the dimos repo, inside the dev shell:

```sh
dimos bake --deployment dimos.robot.unitree.go2.zenoh.motion_host:GO2_MOTION_HOST \
    -o motion-host --builder zigbuild --target aarch64-unknown-linux-gnu.2.31

ssh go2 mkdir -p /root/motion-host
scp motion-host go2:/root/motion-host/
scp misc/motion-host/dimos-motion-host.service go2:/etc/systemd/system/
ssh go2 'systemctl daemon-reload && systemctl enable --now dimos-motion-host'
```

The `.2.31` pins the glibc floor to the Go2's Ubuntu 20.04; check the robot's
with `ldd --version` and pin at or below.

The binary carries its own config. The rust side has no defaults of its own —
python owns them — so `--deployment` names a `Deployment`
(`dimos/robot/unitree/go2/zenoh/motion_host.py`: the module list, class
defaults plus the deployment's few overrides, and the zenoh `session` block
that makes the host a client of go2web's router over loopback) and bake embeds
the resulting blob. There is no config file to ship or hand-edit; a value
changes by editing the deployment and rebaking. `test_motion_host.py` asserts
each module block re-validates, which is the drift that used to present as a
controller bug.

To try a value without rebaking, feed one JSON line on stdin: it deep-merges
over the embedded config (objects descend, leaves replace), so
`{"modules":{"trajectory_follower":{"config":{"embodiment":{"max_speed":0.5}}}}}`
changes that one number. A key the struct does not have, or a `graph` stamp
from another bake, is refused and named in the journal.

**Check**: `ssh go2 journalctl -u dimos-motion-host -n 20` — all four modules
log `module started`, go2_tf logs `publishing the go2 mount tree`, and within
seconds the planner/follower stall lines flip to `odometry: true` (that is the
loopback dial to the bridge working). They keep waiting on `local_map` /
`planner_path` until step 4 — that is correct.

## 4. The laptop half

```sh
dimos --robot-ip <robot> run go2-zenoh-motion-local
```

`-local` is the point: it is `go2-zenoh-motion` MINUS the modules the host now
runs. Plain `go2-zenoh-motion` against a live host gives you two planners, two
followers and two muxes on the same topics.

One dial: go2web runs as a zenoh ROUTER on 7447, and everything hangs off it —
the motion host on the robot connects to it as a client over loopback, and the
laptop's raycaster + MLS reach the robot-side planner through the same router.
Click a goal in the viewer.

**Check**: the robot walks. The follower journal shows plans arriving; a
standing robot logging `single-pose stub, i.e. no safe route` is the planner
honestly refusing — look at the map, not the code.

## 5. Recording and post-mortem

go2web records everything on the robot: `/tmp/go2-recordings/` — a
`*.zenoh.mcap` (all nav topics; this is what analysis wants) and a plain
`*.mcap` sidecar twin (500 Hz joints/IMU). Start/stop from the go2web UI.

```sh
scp go2:/tmp/go2-recordings/<stamp>.zenoh.mcap data/ml-trajectory-research/
```

The offline post-mortem that reads those recordings (map churn, plan flips,
planner and follower replay with input ablation) is not on this branch — it is
`adapter/diagnose.py` on `ivan/feat/trajectory_ctrl`. Oneliners for what IS
here: `dimos/navigation/motion/tools.md`.

---

Future: this page is the spec for an installer script — until it exists,
these steps are the installer.
