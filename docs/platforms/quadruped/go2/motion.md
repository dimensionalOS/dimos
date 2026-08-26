# Running the motion stack on your Go2

Planner, follower, cmd_vel mux and the go2 tf tree run on the robot as one
baked binary; mapping stays on the laptop
([the cut](/docs/platforms/quadruped/go2/motion-deployment.md)). `go2` below
is an ssh alias for `root@<robot>`. You need this repo's dev shell (carries
the rust cross toolchain), a [go2web](https://github.com/dimensionalOS/go2web)
checkout, and a Go2 with the Mid-360 mount.

## 1. go2web bridge (from the go2web checkout)

```sh
scripts/build-nix-musl.sh && scripts/deploy.sh go2
```
Check: `ssh go2 journalctl -u dimos-helper -n 20` logs a `[pointlio] sidecar clock` line.

## 2. PTP for the Mid-360 (`go2web/docs/mid360-ptp.md`)

```sh
ssh go2 apt install -y linuxptp
scp scripts/ptp4l-mid360.conf go2:/etc/linuxptp/
scp scripts/ptp4l-mid360.service go2:/etc/systemd/system/
ssh go2 'systemctl daemon-reload && systemctl enable --now ptp4l-mid360 && systemctl restart dimos-helper'
```
Check: the clock line now says `already unix`, offset in tens of ms.

## 3. Bake and install the motion host (from dimos)

```sh
dimos/navigation/motion/deploy/deploy.sh go2
```
It bakes, installs to `/root/motion-host/`, and enables the unit, in one step. Run
it from anywhere: paths resolve off the script, not your working directory. It
checks the robot over ssh before baking, so a robot that is off costs you seconds
rather than a build. The bake is unconditional -- cargo already does the
incremental staleness check, and a warm bake is about 25 seconds. `--no-build`
installs the binary you already have.
Config is embedded at bake time (`dimos/robot/unitree/go2/zenoh/motion_host.py`), so
changing a value means re-running the script. The binary would deep-merge one JSON line
on stdin over the baked config, but the installed unit does not feed it any, so a stray
`motion-host.json` next to the binary is a leftover from an older unit and does nothing.

The script reports `4/4 modules loaded config` and fails if it does not get four. To look
yourself: `ssh go2 journalctl -u dimos-motion-host -n 20`, where you want four
`config loaded` lines, then `pose: true` in the planner's warnings once tf is up.

## 4. Laptop half

```sh
dimos --robot-ip <robot> run go2-zenoh-motion-local
```
Click a goal in the viewer. Check: the robot walks; `single-pose stub` in the follower journal means no safe route, so look at the map.

## 5. Recordings

`ssh go2 ls /tmp/go2-recordings/`: `*.zenoh.mcap` is the one analysis wants; start/stop from the go2web UI.
Offline tools: `dimos/navigation/motion/tools.md`.
