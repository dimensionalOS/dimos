# R1 Pro SysID collection

**No robot commissioning has been performed.** Do not run the live graph until the
robot operator approves the site and confirms the physical E-stop. Use the
[R1 Pro safety checklist](../../../../docs/platforms/safety/r1pro.md), no payload,
a clear/level area with stopping room in both directions, and a spotter at the
E-stop. Only this DimOS graph may publish motor commands: stop teleop, navigation
and other producers, including ROS publishers in the vendor domain. Run the graph
and RPC client on the robot host. Source the ROS/vendor environment (`rclpy`,
`hdas_msg`); this blueprint does not boot HDAS or change the vendor control mode.
While idle it sends no commands; connection shutdown still invokes its standard
zero/hold routine. The software checks do **not** perform collision, stability,
workspace or firmware-stop verification.

## 1. Preview, without ROS or a robot

```sh
CUDA_VISIBLE_DEVICES='' python -m dimos.robot.galaxea.r1pro.sysid --dry-run --case=torso-j2:chirp > preview.jsonl
```

The preview is on an exact 50 Hz grid with a synthetic zero posture. It does not
certify clearance, the live entry posture, or actual ROS publication cadence.
Use `--help` for case names.

## 2. Record and run one short trial at a time

Use a unique `DIMOS_RUN_ID` and an unused Zenoh discovery address/domain. Keep
both settings identical for the recorder and operator client. The recorder uses
all seven stream names in the whitelist; all are typed Zenoh streams.

```sh
CUDA_VISIBLE_DEVICES='' dimos --record mcap --record-engine rust \
  --zenoh-domain r1pro_sysid --zenoh-scout-addr 224.0.0.224:19546 \
  --record-topics r1pro_diagnostics,r1pro_position_command,r1pro_velocity_command,joint_state,odometry,imu_chassis,imu_torso \
  run r1pro-sysid
```

Wait for recorder readiness and fresh feedback on the three upper-body segment
topics plus `/motion_control/chassis_speed`. In a second terminal on the robot,
create the operator client; keep it open for abort/status:

```sh
CUDA_VISIBLE_DEVICES='' DIMOS_TRANSPORT=zenoh ZENOH_DOMAIN=r1pro_sysid \
  ZENOH_SCOUT_ADDR=224.0.0.224:19546 python -i
```

```python
from dimos.core.rpc_client import RPCClient
from dimos.robot.galaxea.r1pro.blueprints.basic.r1pro_sysid import R1ProSysIdCoordinator

rpc = RPCClient.remote(R1ProSysIdCoordinator)
rpc.task_invoke("sysid", "status")
rpc.task_invoke("sysid", "run_trial", {"case": "vx:0.2"})  # preview only
# After clearance and operator authorization, explicitly request one trial:
rpc.task_invoke("sysid", "run_trial", {"case": "vx:0.2", "live": True})
rpc.task_invoke("sysid", "status")
# If anything is unexpected, use the physical E-stop; software abort is secondary.
rpc.task_invoke("sysid", "abort")
```

- **A — chassis:** `vx:0.2`, `vx:0.5`, `vx:0.8`; `vy:0.2`, `vy:0.5`,
  `vy:0.8`; `wz:0.3`, `wz:0.8`, `wz:1.5` (insert `-` after `:` for the
  negative trial). Each step holds 1.5 s before zero. `deadband` ramps forward
  from 0.01 to 0.10 m/s over 2 s; `coupled` is 0.5 m/s and 0.8 rad/s for
  1.5 s. `vx:1.2` and `vx:-1.2` are **preview only**, because the deployed
  connection and Coordinator model limit is ±1.0 m/s. Do not bypass this cap.
- **B — upper body:** `<joint>:step:0.1`, `<joint>:step:0.25` (or negative),
  or `<joint>:chirp`. Choose `torso-j2`, `shoulder` (left J1), `elbow` (left
  J4), or `wrist` (left J6). The six-second chirp sweeps through 0.5/1/2/3 Hz,
  with a 0.15→0.10 rad amplitude schedule and tapered endpoints. The task
  always uses complete torso/left/right position frames.
- **C — gravity:** `zero`, `arms-forward`, `torso-bent`. Use a separately
  approved method to stage the robot within 0.35 rad per joint of the desired
  pose, then remove that command producer. The trial approaches over 1.5 s and
  holds for 3 s; it is not a posture planner. Arms-forward uses −0.95 rad on
  both shoulder J1s; torso-bent uses +0.20 rad on torso J2.

Each trial requires fresh stamped feedback, a stationary chassis at entry, and
model-limit margin. Software abort fences new moving publications at the
Coordinator, then requests zero body speed and a full measured (or last valid)
position hold on the next eligible tick. A command already in transport/ROS may
still reach the vendor. If the control path is lost, the configured connection
deadman is 0.1 s; do not treat that as physical stop latency. E-stop clear does
not resume a trial. Confirm zero motion and read `status` before selecting a new
case. Stop the operator client with `rpc.stop_rpc_client()` and end the recording
with Ctrl+C after post-roll, allowing the recorder to flush.

## 3. Verify the resulting `memory.mcap`

- `r1pro_diagnostics`: `std_msgs.String` containing v1 JSON. `rx` events carry
  raw control feedback, status and IMUs as seen by ROS callbacks. `tx` events
  contain the actual vendor ROS fields (including tracking-speed arrays) when
  `RawROS.publish` returns, not motor-application acknowledgement. `header_ns`
  is the original ROS stamp (null for headerless messages); `monotonic_ns`
  marks receive entry or pre-publish send, and `unix_ns` pairs it to wall time.
- `r1pro_position_command`, `r1pro_velocity_command`: Coordinator output;
  `joint_state`, `odometry`, `imu_chassis`, `imu_torso`: decoded feedback. Do
  not substitute recorder log time or fused joint-state time for the raw RX
  clock. Effort values are vendor feedback, not calibrated torque.
- Check the recorder-reported path, channel counts, raw header ages, `seq`
  gaps, actual TX cadence, start/stop zeros, and matched RX/TX before fitting.
  The tap drops on overflow and the bus/recorder may also lose samples; sort
  cross-thread events by `monotonic_ns`. Record run ID, git revision, case,
  posture/payload, abort reason and robot firmware alongside the artifact.
