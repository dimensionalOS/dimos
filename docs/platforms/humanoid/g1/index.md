# Unitree G1

![output](assets/g1_rerun.webp)

## Requirements

- Unitree G1 EDU (need SDK/SSH access)
- Laptop/Desktop with Ubuntu 22.04/24.04 with CUDA GPU (recommended), or macOS (experimental)

## 1. Get SSH Access

### Get Ethernet Working
1. Plug an Ethernet cable from the robot into your Laptop
2. Open up your Laptop's graphical network manager, manually set the IP addr of your system to `192.168.123.100`
3. Run wired ssh command:

```bash
ssh -L 3030:localhost:3030 unitree@192.168.123.164
# Password: 123
```

### Use Ethernet to get WiFi Working

After ssh-ing in, find additional IPs:
```bash
hostname -I
```
The second address allows SSH after disconnecting Ethernet.

WiFi passwords (varies by unit): `888888888` or `00000000`

### Network Interface Names

Common interface names needed for SDK examples:
- `eth0` / `enp2s0`: Ethernet
- `wlan0`: WiFi

Check with: `ip addr show`

### Remote Network

Recommended to setup [tailscale](https://tailscale.com/tailscale-ssh) to avoid needing to setup rounter specific configuraions for wireless control.

## 2. Install dimOS

SSH into the robot, then:

```bash
# pick the "developer" setup
bash <(curl -fsSL https://pub-4767fdd15e6a41b6b2ce2558d71ec8d9.r2.dev/install.sh)
```

#### Notes

dimOS handles DDS setup automatically. If you're using the Unitree SDK directly, set:
```bash
export CYCLONEDDS_HOME="$HOME/cyclonedds/install"
```

## 3. Get the G1 in Sport Mode

**WARNING**: You *need* to have the G1 in a good physical position before running this.

Get the hand-held controller for the G1.

Note: this button combination may vary based on the model of the G1

1. If you have a gantry, hang the robot up where its feet are touching the floor, knees straight.
   - Press **L2 + B** (no movement, color change)
   - Press **L2 + Up** (should straighten out)
   - Press **R2 + A** (will attempt to self-balance)
2. If don't have a gantry, there is a make-shift way to get it working. You should get a second person to help.
   - Make the robot lie down flat on the ground
   - Press **L2 + B** (no movement, color change)
   - Press **L2 + Up** (should straighten out)
   - The robot will be super stiff now. Manually pick it up into a standing position and hold it there.
   - Press **R2 + A** (will attempt to self-balance)

## 4. Start G1 teleoperation

Use a clear, level work area. Keep the Unitree remote in an operator's hands,
keep the emergency stop reachable, and use a gantry or spotter for the first
hardware run. The upper-body planner does not include leg collision geometry;
do not walk while executing planned arm motion.

The robot must already be standing and balancing in sport mode. Stop any other
program that can publish Unitree low-level commands before starting DimOS.

On the G1 computer:

```bash
cd ~/cc/dimos
uv sync --extra all

LD_PRELOAD=/lib/aarch64-linux-gnu/libgomp.so.1 \
  uv run dimos \
    --viewer none \
    run unitree-g1-teleop \
    --network-interface eth0
```

`--network-interface` is blueprint configuration, so it belongs after the
blueprint name. On the Jetson, preloading `libgomp` avoids the static TLS error
that can occur while importing Pinocchio.

For an upper-body-only session, omit the mapping and route-planning workers but
keep `MovementManager`, which routes the Quest walking command:

```bash
LD_PRELOAD=/lib/aarch64-linux-gnu/libgomp.so.1 \
  uv run dimos \
    --viewer none \
    run unitree-g1-teleop \
    --network-interface eth0 \
    --disable point-lio ray-tracing-voxel-map cost-mapper \
      replanning-a-star-planner websocket-vis-module
```

Wait for the Quest server to report that it is listening on port 8443. The
process starts unarmed and in dry-run on real hardware.

### Arm and enable the controller

Use the hardware commands from a second SSH session. They are the supported
operator interface; `dimos shell` is intended for debugging, not routine robot
startup.

```bash
# Inspect the interlocks and planner connection.
uv run dimos hardware g1 status

# Start the 10-second handover ramp while motor output remains in dry-run.
uv run dimos hardware g1 arm

# Inspect the physical robot and remote, then allow motor commands.
uv run dimos hardware g1 enable

# Optional: collision-check and execute a conservative bimanual ready pose.
uv run dimos hardware g1 ready
```

`ready` moves both shoulders slightly forward, bends both elbows, and uses 25%
of the configured trajectory speed. It refuses to run until arming is complete
and dry-run has been disabled.

To stop arm motion and return to the safe startup state:

```bash
uv run dimos hardware g1 disable
```

This command cancels the active trajectory, enables dry-run, and disarms the
controller. Keep the Unitree remote available as the independent physical
safety control.

### Connect the Quest headset

Open the following address in the Quest browser and accept the self-signed TLS
certificate:

```text
https://<g1-computer-ip>:8443/teleop
```

The controls are:

| Input | Operation |
|---|---|
| Left stick | Move forward or backward; yaw in strafe mode |
| Right stick | Yaw |
| Press right stick | Publish a zero-velocity stop command |
| Hold X + A | Engage both arms from a shared reference pose |
| B | Start or save a recording episode |
| Y | Discard the current episode |

Quest arm targets preempt a running planned arm trajectory because the Quest IK
task has the higher coordinator priority. Releasing arm tracking does not
resume an old trajectory.

### Connect the manipulation panel

The G1 teleop blueprint starts the Viser manipulation panel on all network
interfaces at port `8095`. Open this address from a computer on the robot's
trusted network:

```text
http://<g1-computer-ip>:8095
```

Viser remains available when the global viewer is `none`; it is independent of
Rerun. The panel can plan and execute arm motion, so do not expose port `8095`
to an untrusted network.

### Optional Rerun viewer

The teleoperation UI is the Quest page. Rerun is a separate visualization of
the robot, maps, and trajectories. Start the blueprint with `--viewer rerun`
for a local native viewer, or leave it at `none` for the onboard computer.

## 5. Legacy navigation viewer example

In the ssh terminal `ssh -L 3030:localhost:3030 unitree@192.168.123.164`

```sh skip
source .venv/bin/activate
uv run dimos --rerun-host 0.0.0.0 run unitree-g1-nav-simple
# should print out something like:
# ============================================================
# Rerun gRPC server running (no viewer opened)
#
# Connect a viewer:
#   dimos-viewer --connect rerun+http://0.0.0.0:9877/proxy --ws-url ws://0.0.0.0:3030/ws
#   dimos-viewer --connect rerun+http://192.168.123.164:9877/proxy --ws-url ws://192.168.123.164:3030/ws  # eth0
#   dimos-viewer --connect rerun+http://100.88.236.73:9877/proxy --ws-url ws://100.88.236.73:3030/ws  # tailscale0
#   dimos-viewer --connect rerun+http://10.0.0.197:9877/proxy --ws-url ws://10.0.0.197:3030/ws  # wlan0
#   dimos-viewer --connect rerun+http://172.17.0.1:9877/proxy --ws-url ws://172.17.0.1:3030/ws  # docker0
#
#   hostname: ubuntu
# ============================================================
```

On your laptop:

```sh skip
# install uv
curl -LsSf https://astral.sh/uv/install.sh | sh
uv venv --python "3.12"
# use uv to get the dimos viewer
uvx dimos-viewer --version

# run the connect command. NOTE: the address will be different for you
uvx dimos-viewer --connect rerun+http://100.88.236.73:9877/proxy --ws-url ws://100.88.236.73:3030/ws
```

The viewer should open up. It'll run in faster-than-real speed until its caught up with reality, then should show what's happening in real time.

## Troubleshooting

### `libgomp.so.1: cannot allocate memory in static TLS block`

Launch with:

```bash
LD_PRELOAD=/lib/aarch64-linux-gnu/libgomp.so.1 uv run dimos ...
```

If the failing path is under `roboplan.libs`, the RoboPlan aarch64 wheel has
loaded a second private `libgomp`; preloading the platform library alone is not
enough. See the [RoboPlan static TLS handoff](roboplan-libgomp-handoff.md) for
the confirmed reproducer, temporary workaround, and wheel acceptance criteria.

### A mapping module tries to build with Nix

Use the upper-body-only launch command above. It disables Point-LIO, voxel
mapping, cost mapping, route planning, and the navigation web view without
removing the Quest camera, arm control, recording, or walking-command router.

### `dimos hardware g1 status` cannot connect

The teleop blueprint must still be running, and both terminals must use the
same DimOS transport configuration. Check the primary process with
`uv run dimos status` and `uv run dimos log -f`.

### Ready-pose planning fails

Do not bypass the planner. Confirm that the robot is stationary, both arm and
waist joint states are arriving, no object starts in collision with the upper
body, and `status` lists `g1_upper_body/left_arm` and
`g1_upper_body/right_arm`.

#### Keyboard Controls Not Working

This usually means port `3030` wasn't forwarded. The `3030:localhost:3030` in the ssh command is what forwards the port. If you use VS Code with the SSH plugin, ports will be forwarded automatically. However sometimes the auto-forward will map 3030 to 3031 - thus breaking the connect command. Clear whatever is on port 3030 (on the G1 sid and the Laptop) then try again.

#### Viewer Crashing

If the viewer keeps crashing for you, there are two options for now:
1. On the G1 (ssh connection) change `_MAX_HZ` (inside `dimos/robot/unitree/g1/blueprints/primitive/unitree_g1_vis.py`) to a lower number, like 20 or 15
2. Get more RAM



## External Resources

- [Unitree Developer Docs](https://support.unitree.com/home/en/developer)
- [Sport Mode Services](https://support.unitree.com/home/en/developer/sports_services)
- [Unitree SDK2 Python](https://github.com/unitreerobotics/unitree_sdk2_python)
