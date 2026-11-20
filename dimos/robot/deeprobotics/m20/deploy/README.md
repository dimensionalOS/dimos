# M20 deployment

DimOS runs on the M20's three boards: GOS (`10.21.31.104`, where `dimos run m20` starts and the navigation runs),
AOS (`10.21.31.103`: the body IMU, the cameras and the motor bus) and NOS (`10.21.31.106`: the lidars' network and
dimSLAM). Each board runs a `dimos host` that takes the modules the blueprint places on it.

A laptop on the robot hotspot reaches GOS through AOS:

```bash
sudo ip route add 10.21.31.0/24 via 10.21.41.1
```

## Setup

1. Robot setup (clocks, the vendor services DimOS replaces, the ecan shim): `m20tool setup`.
2. The dimos Hosts, from the dimos checkout on GOS (re-run after changing the checkout, so the boards run the same
   revision):

   ```bash
   bash dimos/robot/deeprobotics/m20/onboard/install_dimos_host.sh gos aos nos
   ```

## Run

```bash
dimos run m20
```

`--choice M20Mode=sim` runs the same navigation on the simulated M20 instead (no robot needed). The real mode's own
choices (body filter, joint source, walker, body IMU, cameras, where dimSLAM runs) are in
`dimos/robot/deeprobotics/m20/blueprints/m20.py`.

## Walking

By default nothing in the real mode commands the legs: nothing reads `cmd_vel`. `--choice M20Walker=...` picks who
does, and needs an operator on site before anything is sent:

- `rl_deploy`: the vendor walking controller, on its own REAL_STEER topic (`m20/cmd_vel`, limited and gated as the
  vendor's bridge: only in RL control, zero otherwise). Needs rl_deploy running and basic_server stopped (it writes
  nothing while another REAL_STEER writer is up). Standing up and the gait stay with the vendor remote.
- `controller`: the M20 controller (`m20/controller`) owns the motor bus in place of rl_deploy (rl_deploy stopped)
  and runs its walking policy (rl_deploy's state machine and ONNX runners) on `cmd_vel`, `motion_cmd` (passive,
  stand, rl_control, lie_down, joint_damping) and `gait_cmd`. Its gains stay off (`allow_gains`) until an operator
  turns them on, so out of the box it runs the state machine without moving anything.

The controller also serves the joint state and batteries (`--choice M20Joints=controller`): beside rl_deploy it reads
the ecan shim's stream; owning the bus (rl_deploy stopped, the walker above) it sends rl_deploy's startup and then
the zero-gain sitting command, with the charge-enable request while docked.

Offline: `controller/rust` has the MuJoCo backend and its tests (`cargo test`), and the dry-run shadow beside
rl_deploy (`controller/deploy/shadow`, a dev tool, not in the blueprint).
