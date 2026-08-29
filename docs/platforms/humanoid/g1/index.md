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

The robot must already be standing and balancing in sport mode. Use a clear,
level work area, keep the Unitree remote and emergency stop reachable, and use
a gantry or spotter for the first hardware run. Do not walk while executing
planned arm motion because the upper-body planner excludes leg geometry.

On the G1 computer:

```bash
uv sync --extra all
uv run dimos run unitree-g1-teleop --network-interface eth0
```

The teleop blueprint excludes navigation and mapping, so no module-disable
arguments are needed. Wait for the Quest server to listen on port `8443`, then
activate the robot from a second SSH session:

```bash
uv run dimos hardware g1 status
uv run dimos hardware g1 activate
uv run dimos hardware g1 status
uv run dimos hardware g1 ready
```

`activate` runs the GR00T pose ramp and requires interactive confirmation before
enabling output. Check the status before `ready` moves both arms to the
conservative ready pose. Routine startup must use these hardware commands rather
than `dimos shell`. `activate --ready` remains available as a combined shortcut.

Open `https://<g1-computer-ip>:8443/teleop` in the Quest browser and accept the
self-signed certificate.

| Input | Operation |
|---|---|
| Left stick | Move forward or backward; yaw in strafe mode |
| Right stick | Yaw |
| Press right stick | Publish a zero-velocity stop command |
| Hold X + A | Engage both arms from a shared reference pose |
| B | Start or save a recording episode |
| Y | Discard the current episode |

The blueprint also serves the Viser manipulation panel at
`http://<g1-computer-ip>:8095`. It can execute arm motion; only expose this port
on a trusted robot network. Quest arm targets preempt planned arm trajectories.

When finished, cancel arm motion, enter dry-run, and disarm:

```bash
uv run dimos hardware g1 disable
uv run dimos stop
```

`disable` is a soft policy disarm into current-pose hold. It is not an
emergency stop and does not terminate low-level commands; use the Unitree
physical stop for emergencies and `dimos stop` for routine shutdown.

### SONIC full-body PICO teleoperation

SONIC uses the same `dimos hardware g1` lifecycle commands, discovered from
the running controller's task card.

#### Experimental JetPack 5 / CUDA 11.8 runtime

NVIDIA's supported onboard SONIC deployment uses JetPack 6 and TensorRT 10.7.
DimOS also provides an experimental ONNX Runtime path for the original G1 PC2
JetPack 5 image. It installs CUDA 11.8 alongside the existing CUDA 11.4 stack;
it does not flash the robot or replace the Jetson Linux BSP. See NVIDIA's
[Jetson CUDA upgrade
guide](https://developer.nvidia.com/blog/simplifying-cuda-upgrades-for-nvidia-jetson-users/)
and [official SONIC deployment
requirements](https://nvlabs.github.io/GR00T-WholeBodyControl/getting_started/installation_deploy.html)
before choosing this path.

From NVIDIA's [CUDA 11.8
archive](https://developer.nvidia.com/cuda-11-8-0-download-archive), select
`Linux / aarch64-jetson / Ubuntu / 20.04 / deb`, run the generated repository
setup commands, then install the versioned packages:

```bash
sudo apt-get update
sudo apt-get install cuda-runtime-11-8 cuda-compat-11-8
```

Install the pinned JetPack 5 ONNX Runtime 1.18.1 wheel and run the offline
safety gates:

```bash
cd ~/cc/dimos
bin/hardware/g1/setup-sonic-jp5 --check
bin/hardware/g1/setup-sonic-jp5

export PATH=/usr/local/cuda-11.8/bin:$PATH
export LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
export LD_LIBRARY_PATH=/usr/local/cuda-11.8/lib64:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/usr/local/cuda-11.8/compat:$LD_LIBRARY_PATH
source .venv-sonic-jp5/bin/activate

sudo nvpmodel -m 0
sudo jetson_clocks
dimos hardware g1 sonic-doctor
```

The JetPack 5 environment contains the real-hardware SONIC stack, not the
development-only simulation and test dependency groups. Those groups contain
prebuilt ARM64 packages that require a newer glibc than Ubuntu 20.04. Rehearse
the simulation on the development workstation, then use this environment for
the onboard diagnostic and real-hardware launch. The setup script creates this
environment with CPython 3.10 so the Open3D ARM wheel remains compatible with
JetPack 5's Ubuntu 20.04 userspace.

The setup script installs
[`onnxruntime-gpu-extended-auto==1.23.3`](https://github.com/jeff-hykin/onnxruntime-gpu-extended-auto)
with target-side package detection. CUDA 11 and cuDNN 8 resolve to the pinned
`onnxruntime-gpu-extended==1.18.1.11.8` JetPack 5 wheel. The script bypasses
the pip cache and verifies the dispatcher, distribution, runtime, and CUDA
provider versions after installation.

`sonic-doctor` never contacts the robot. It validates MAXN and locked CPU/GPU
clocks, the exact model hashes for both official policy bundles,
CUDA execution partition, numerical output, and onboard latency. The encoder
and decoder are forbidden from using CPU fallback. The planner may use CPU
only for its audited shape/index operators; a larger or different partition
fails the check. Do not continue if any check fails.

Run the activated environment's `dimos` executable directly. Do not use
`uv run`: dependency synchronization can reinstall the CPU-only
`onnxruntime` package over the JetPack 5 GPU wheel.

Rehearse the full stack in MuJoCo before connecting policy output to motors:

```bash
uv run python bin/hardware/g1/setup-sonic-models \
  --profile sonic-v1.1

dimos --simulation mujoco --viewer none \
  run unitree-g1-sonic-webxr-teleop \
  --sonic-pipeline sonic-v1.1
```

Require ten minutes of stable planner balancing and repeat the
`PLANNER -> POSE -> PLANNER` transition before proceeding. Then start the
real-hardware blueprint with:

```bash
dimos --viewer none run unitree-g1-sonic-webxr-teleop \
  --network-interface <robot-nic>
```

The first hardware test requires the official overhead gantry, both feet in
contact with the floor, and three operators: one at the Unitree remote and
physical stop, one wearing the PICO, and one at the DimOS terminal. Do not run
the native `g1_deploy_onnx_ref` process at the same time, and do not attempt
untethered walking during the first session.

Follow NVIDIA's [whole-body teleoperation safety
guide](https://nvlabs.github.io/GR00T-WholeBodyControl/user_guide/teleoperation.html)
and [PICO workflow](https://nvlabs.github.io/GR00T-WholeBodyControl/tutorials/vr_wholebody_teleop.html).
Full-body tracking includes the operator's feet, so an occluded or incorrectly
tracked leg can command an unsafe whole-body reference. Wear close-fitting
pants, keep at least 3 m of clear space around the robot, and do not proceed if
tracking latency is above 30 ms or any body joint is unstable.

dimOS uses the robot-policy lifecycle to gate the WebXR reference:

```text
UNARMED/current hold
        |
        | dimos hardware g1 arm
        v
CONTROL/dry-run, WebXR PLANNER --A+X--> POSE_TRANSITION --> POSE preview
        |                                              |
        | dimos hardware g1 enable                     | enable returns to PLANNER
        +-----------------------------+----------------+
                                      v
       CONTROL/live, WebXR PLANNER --A+X--> POSE_TRANSITION --> POSE
```

Run `status`, `arm`, `status`, dry-run POSE preview, `enable`, and `status` as
separate steps so the team can inspect the reference between transitions.
Dry-run still executes SONIC inference and publishes `world/sonic_reference`,
but the task returns no learned-policy joint command. `enable` always enters
WebXR `PLANNER`; if dry-run preview is in `POSE_TRANSITION` or `POSE`, enabling
clears the pose reference and preview-only policy history, then returns to
`PLANNER` before motor output resumes. Press A+X again to enter live `POSE`.

Before pressing A+X, stand upright with feet together, look forward, keep the
upper arms down, bend the forearms 90 degrees forward, and point the palms
inward.

Select the NVIDIA policy bundle when launching the blueprint. Encoder,
decoder, observation layout, and pose window always switch together:
[NVIDIA's model card](https://github.com/NVlabs/GR00T-WholeBodyControl/blob/main/docs/source/model_card.md)
documents both contracts, and the setup script downloads their pinned files
from [`nvidia/GEAR-SONIC`](https://huggingface.co/nvidia/GEAR-SONIC/tree/main).

| `--sonic-pipeline` | Pose window | Use when |
|---|---:|---|
| `sonic-v1.1` (default) | 10 frames / about 200 ms | Matching the official temporal input is more important than latency |
| `sonic-low-latency` | 4 frames / about 80 ms | NVIDIA's released low-latency model and body-frame observation layout |

```bash
# Official ten-frame path (the flag may be omitted)
uv run dimos --simulation mujoco run unitree-g1-sonic-webxr-teleop \
  --sonic-pipeline sonic-v1.1

# Official four-frame low-latency path
uv run python bin/hardware/g1/setup-sonic-models \
  --profile sonic-low-latency
uv run dimos --simulation mujoco run unitree-g1-sonic-webxr-teleop \
  --sonic-pipeline sonic-low-latency
```

The selection is fixed for the process lifetime; restart the blueprint to
change it. A+X blends the current planner encoder token into the live PICO
token over 0.5 seconds by default. Tune that handoff without changing the pose
window:

```bash
uv run dimos --simulation mujoco run unitree-g1-sonic-webxr-teleop \
  --sonic-pipeline sonic-v1.1 \
  --pose-transition-seconds 0.8
```

The duration must be positive and finite. The blend follows new PICO frames as
they arrive; it does not freeze the operator's pose at the A+X press.

MuJoCo keeps its existing fast iteration lifecycle: the simulated policy
auto-arms with no ramp or dry-run and enters WebXR `PLANNER` as soon as control
starts. Wait for the pose buffer, then press A+X. Real hardware instead starts
unarmed in dry-run and requires the CLI `arm` and `enable` sequence above. To
rehearse that sequence against a running simulation, first run `dimos hardware
g1 disable`, then use the same `arm`, `status`, and `enable` commands as on the
robot.

Use the CLI and controller in this order:

1. Run `dimos hardware g1 arm`, then `dimos hardware g1 status`. Confirm
   `armed: True`, `dry_run: True`, and `webxr: planner`.
2. Align the operator with the robot and wait until `pose_buffer` reports
   `ready`: about 200 ms for `sonic-v1.1` or 80 ms for
   `sonic-low-latency` after complete tracking begins.
3. Press **A+X** to enter dry-run `POSE_TRANSITION`, followed by `POSE` after
   the configured handoff. The robot must not follow the pose. Confirm
   `dry_run: True`, `webxr: pose`, and `reference: webxr_pose` with
   `dimos hardware g1 status`, and inspect `world/sonic_reference` in Rerun.
4. Run `dimos hardware g1 enable`. This clears the preview and returns to
   `PLANNER` before learned-policy motor output resumes. Confirm
   `dry_run: False`, `webxr: planner`, and `reference: planner` with `status`.
5. Realign the operator, wait for `pose_buffer` to become ready again, and
   press **A+X** to enter live `POSE`. Do not proceed if the preview was
   unstable, incorrectly oriented, or did not match the operator.
6. Press **A+X** again to return to the balancing planner.
7. Finish routine operation with `dimos hardware g1 disable`, followed by
   `dimos stop`.

SONIC inference runs at 50 Hz. On hardware, the G1 connection holds the newest
policy target and publishes it to `rt/lowcmd` at 500 Hz. Runtime timing is
reported under `policy_timing` by `dimos hardware g1 status`, but it does not
gate POSE or force a return to PLANNER. Run `sonic-doctor` before hardware use;
its policy and planner latency checks are the performance acceptance gate.

ABXY has no SONIC teleoperation action. The terminal owns live policy output,
while the PICO wearer owns only the `PLANNER`/`POSE` tracking toggle. Neither
software control is an emergency stop; use the Unitree physical stop for
emergencies.

### Inspecting the SONIC pose reference

Launch the Rerun viewer when testing teleoperation:

```bash
uv run dimos --simulation mujoco --viewer rerun --rerun-open web run unitree-g1-sonic-webxr-teleop
```

The browser opens the direct Rerun Web viewer at `http://localhost:9878`.
This blueprint sends only the `world/sonic_reference` layer to Rerun; it does
not stream the G1 model, sensors, or other DimOS topics. The bridge retains only
the newest pending reference, displays it at up to 30 Hz, and prioritizes live
data when a browser connects or falls behind. The newest accepted skeleton is
bright cyan, the preceding frame is a faint trail, and RGB axes show the
separately sent root and wrist orientations. The layer remains visible through
`POSE_TRANSITION` and `POSE`, then clears on the immediate return to `PLANNER`
or `OFF`. In `sonic-v1.1`, the newest cyan input contains approximately 200 ms
of reference history; the `sonic-low-latency` option reduces that window to
approximately 40 ms. The separate `--pose-transition-seconds` handoff applies
only when entering POSE.

For the first hardware run, rehearse the complete lifecycle in simulation and
inspect `dimos hardware g1 status` before every real transition. Tracking loss
or a WebXR reference-space change returns `POSE_TRANSITION` or `POSE` to
`PLANNER`, clears the old reference, and rebuilds the pose buffer before A+X
can enter `POSE` again.

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

### SONIC cannot activate `CUDAExecutionProvider`

SONIC requires GPU inference for responsive and safe teleoperation. Startup
fails instead of running the models on CPU if CUDA cannot be activated. If the
error mentions `libcublasLt.so.12` on a CUDA 13 host, install the project's CUDA
extra:

```bash
uv sync --extra all
```

DimOS uses ONNX Runtime's CUDA 12 build and preloads its CUDA 12/cuDNN 9
libraries from the virtual environment. A CUDA 13 NVIDIA driver can run this
CUDA 12 application; do not point `LD_LIBRARY_PATH` at CUDA 13 libraries to
satisfy a `.so.12` dependency.

On successful startup, the SONIC log lists `CUDAExecutionProvider` first for
the encoder, decoder, and planner. ONNX Runtime may also list its automatically
registered CPU provider; SONIC verifies that CUDA is active and never retries a
failed model with CPU-only inference. To inspect the preloaded libraries
independently:

```bash
uv run python -c 'import onnxruntime as ort; ort.preload_dlls(); ort.print_debug_info()'
```

See ONNX Runtime's [CUDA execution-provider requirements and preload
API](https://onnxruntime.ai/docs/execution-providers/CUDA-ExecutionProvider.html#preload-dlls).
GLFW, Wayland, and `libdecor-gtk.so` warnings come from the MuJoCo viewer and do
not cause ONNX Runtime to fall back to CPU.

On the G1 JetPack 5 PC2, a CPU-only `onnxruntime` installation is not usable.
Do not run the x86 CUDA-extra instructions above. Re-enter the isolated
environment and validate its pinned CUDA 11 wheel:

```bash
export PATH=/usr/local/cuda-11.8/bin:$PATH
export LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
export LD_LIBRARY_PATH=/usr/local/cuda-11.8/lib64:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/usr/local/cuda-11.8/compat:$LD_LIBRARY_PATH
source .venv-sonic-jp5/bin/activate

python -c 'import onnxruntime as ort; print(ort.__version__, ort.get_available_providers())'
dimos hardware g1 sonic-doctor
```

The expected version is `1.18.1` with `CUDAExecutionProvider` listed first by
the SONIC sessions. Rerun `bin/hardware/g1/setup-sonic-jp5` if the version is
different or only `CPUExecutionProvider` is available.

The auto dispatcher must run through the target environment's regular `pip`,
not `uv pip`: its CUDA/cuDNN dependency is computed while building on the G1.
The setup script handles this distinction and uses `--no-cache-dir` so a wheel
selected on another JetPack release cannot be reused.

### `libgomp.so.1: cannot allocate memory in static TLS block`

RoboPlan 0.6.0's aarch64 wheel bundles a renamed private `libgomp`, while
Pinocchio loads the system copy. On affected systems, start the blueprint with
both libraries preloaded:

```bash
ROBOPLAN_GOMP="$(find "$PWD/.venv/lib/python3.12/site-packages/roboplan.libs" \
  -maxdepth 1 -name 'libgomp-*.so*' -print -quit)"
test -n "$ROBOPLAN_GOMP" || {
  echo "RoboPlan's bundled libgomp was not found"
  exit 1
}

LD_PRELOAD="$ROBOPLAN_GOMP:/lib/aarch64-linux-gnu/libgomp.so.1" \
  uv run --no-sync dimos run unitree-g1-teleop --network-interface eth0
```

Preloading only the system library is insufficient. If startup still fails,
confirm that `ROBOPLAN_GOMP` resolves to a file and appears first in
`LD_PRELOAD`.

### Activation or ready-pose recovery

Use the individual stages to identify whether the arming ramp, output enable,
or planned ready motion failed:

```bash
uv run dimos hardware g1 arm
uv run dimos hardware g1 enable
uv run dimos hardware g1 ready
```

`ready` requires completed arming, enabled output, and disengaged Quest arm
tracking. Run `uv run dimos hardware g1 disable` before restarting the sequence.

### A mapping module tries to build with Nix

Update this branch. The G1 teleop blueprint no longer includes Point-LIO, voxel
mapping, cost mapping, route planning, or the navigation web view. Seeing one
of those modules means the checkout predates the upper-body-only composition.

### `dimos hardware g1 status` cannot connect

The teleop blueprint must still be running, and both terminals must use the
same dimOS transport configuration. Check the primary process with
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
