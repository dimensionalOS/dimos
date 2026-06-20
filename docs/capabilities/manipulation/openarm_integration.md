# OpenArm Integration

Guide for running the **OpenArm** — an open-source bimanual 7-DOF research arm built from Damiao DM-J quasi-direct-drive motors — under the dimos manipulation + control stack.

**If you're standing in front of the hardware and just want to run it, skip to [Quick start](#quick-start).**

Related:
- Upstream hardware + C++ reference: [enactic/openarm_can](https://github.com/enactic/openarm_can)
- How to integrate any new arm: [adding_a_custom_arm.md](/docs/capabilities/manipulation/adding_a_custom_arm.md)

---

## Why this integration is different

Every other arm in dimos wraps a vendor Python SDK:

| Arm | Transport | Python SDK |
|---|---|---|
| xArm | TCP/IP | `xarm-python-sdk` |
| Piper | CAN (via SDK) | `piper_sdk` |
| R1 Pro | Galaxea | Galaxea SDK |
| Go2 / G1 | WebRTC | Unitree SDK |
| Panda | FCI | `panda-py` |

**OpenArm historically shipped no stable Python SDK.** The default `openarm` adapter still uses dimos' in-tree raw-CAN driver and remains the existing production path, so users still select `adapter_type="openarm"` for OpenArm hardware. DimOS also provides an opt-in `openarm_rs` adapter for environments that install the Rust-backed `can-motor-control` Python binding through the manipulation extra.

## Adapter paths

| Adapter | Hardware API | Dependency expectation | Typical use |
|---|---|---|---|
| `openarm` | In-tree SocketCAN Damiao driver | `python-can` plus Pinocchio for gravity feed-forward | Existing OpenArm coordinator, planner, and teleop blueprints. |
| `openarm_rs` | Rust-backed `can-motor-control` Python binding | Install `dimos[manipulation]` so `can_motor_control` is importable | OpenArm RS bring-up, binding-backed coordinator operation, and gravity-compensation-only validation. |

Selecting `openarm_rs` is explicit through blueprint or hardware config. Registry discovery remains available without `can_motor_control`; selecting the adapter fails with a clear missing-binding error if the package is absent. Future Damiao-based arms should subclass the shared Damiao adapter base with their own typed motor/gain/limit metadata instead of relying on OpenArm defaults.

## Architecture

```
ManipulationModule  →  ControlCoordinator  →  OpenArmAdapter  →  OpenArmBus  →  SocketCAN  →  arm
    (Drake plan)         (100Hz tick loop)    (dimos protocol)  (CAN driver)
```

Code layout:

```
dimos/hardware/manipulators/openarm/
├── driver.py          # OpenArmBus, DamiaoMotor — pure CAN driver, no dimos deps
├── adapter.py         # OpenArmAdapter — implements dimos ManipulatorAdapter protocol
├── test_driver.py     # 13 unit tests (virtual CAN loopback, no hardware)
└── test_adapter.py    # 11 unit tests (virtual CAN + mock state frames)

dimos/robot/catalog/openarm.py     # openarm_arm() and openarm_single() config factories
dimos/robot/manipulators/openarm/
├── blueprints.py      # coordinator-*, planner-*, keyboard-teleop-* blueprints
└── scripts/           # bring-up + diagnostic scripts (run manually by humans)
    ├── openarm_can_up.sh         # bring SocketCAN interfaces up (needs sudo)
    ├── openarm_can_probe.py      # enumerate & read state from all 8 motors
    ├── openarm_set_mit_mode.py   # one-time CTRL_MODE=MIT write per motor
    └── ... (diagnostics)

data/openarm_description/          # URDF + meshes (in-tree; may migrate to LFS)
└── urdf/robot/
    ├── openarm_v10_bimanual.urdf  # both arms (14 DOF, used by coordinator)
    ├── openarm_v10_left.urdf      # left arm + torso (7 DOF, per-side planning)
    ├── openarm_v10_right.urdf     # right arm + torso (7 DOF)
    └── openarm_v10_single.urdf    # standalone arm (Pinocchio FK for teleop)
```

Workspace analysis is generic and lives in [dimos/utils/workspace.py](/dimos/utils/workspace.py) — works for any URDF, not just OpenArm.

---

## Quick start

You need:

- 2× **OpenArm v10** arms, wired to USB-CAN adapters
- 2× **USB-CAN adapters** (we used gs_usb family, VID:PID `1d50:606f`, e.g. CANable 2.0). Classical CAN @ 1 Mbit is enough; CAN-FD not required
- **Python 3.12 venv with dimos installed** plus `python-can >= 4.3` and `pinocchio`
- **sudo** on first run (to bring up the CAN interfaces)

### 1. Bring up the CAN buses

```bash
sudo ./dimos/robot/manipulators/openarm/scripts/openarm_can_up.sh can0 can1
```

This sets both interfaces to classical CAN @ 1 Mbit with a 1000-frame TX queue (enough headroom for the 100 Hz tick loop). If only one bus is present, pass just that one: `sudo ... openarm_can_up.sh can0`.

**Troubleshooting:**
- `Operation not permitted` → you forgot `sudo`.
- `Operation not supported` on `fd on` → your adapter doesn't support CAN-FD. The script defaults to classical, so this shouldn't happen unless you set `MODE=fd`.
- Only one `can*` interface appears → the other adapter isn't enumerating. On gs_usb boards, the **blue LED** indicates USB enumeration. If one adapter only shows red/green, swap the USB cable (many USB-C cables are charge-only).

### 2. Verify all 16 motors are alive

```bash
python ./dimos/robot/manipulators/openarm/scripts/openarm_can_probe.py --channel can0
python ./dimos/robot/manipulators/openarm/scripts/openarm_can_probe.py --channel can1
```

Expected: `8/8 motors replied` on each bus, with plausible joint positions and rotor temps around 25–30 °C.

### 3. (First time only) Put motors in MIT mode

Damiao motors have a persistent `CTRL_MODE` register. They ship in POS_VEL mode by default, which means they will reply to enable/state queries but **silently ignore** any MIT control frames — the "motor doesn't move, error grows" failure. The adapter writes MIT on every `connect()` by default, so this step is usually automatic. If you want to set it explicitly once:

```bash
python ./dimos/robot/manipulators/openarm/scripts/openarm_set_mit_mode.py --channel can0
python ./dimos/robot/manipulators/openarm/scripts/openarm_set_mit_mode.py --channel can1
```

The register is persistent across power cycles, so you only need this once per motor (or after a firmware reset).

### 4. Run a blueprint

| Blueprint | What it does |
|---|---|
| `coordinator-openarm-mock` | Bimanual, mock adapters. No hardware. |
| `openarm-mock-planner-coordinator` | Drake planner + bimanual mock, Meshcat viz. Great smoke test. |
| `coordinator-openarm-left` / `coordinator-openarm-right` | Single arm, real hardware on can0 / can1. |
| `coordinator-openarm-bimanual` | Both arms, real hardware, no planner. |
| `openarm-planner-coordinator` | **Main usable blueprint** — Drake planner + both arms on real hardware. |
| `keyboard-teleop-openarm-mock` / `keyboard-teleop-openarm` | Single-arm Cartesian IK + pygame keyboard, mock / real. |
| `coordinator-openarm-rs` | Opt-in single-arm coordinator path using the new `adapter_type="openarm_rs"` driver, without the planner. |
| `openarm-rs-planner-coordinator` | **New driver usable blueprint** — `ManipulationModule` + `ControlCoordinator` + `openarm_rs` on one arm. |

**Safety before hot-plugging hardware:** hold the arms before starting. On connect, the adapter enables all motors and sends gravity-comp holds — the arms go slightly stiff but don't leap. Ctrl-C to cleanly disable and exit.

First-time recommendation for the existing `openarm` adapter: mock planner to verify everything wires up, then real single-arm, then bimanual.

```bash
# smoke test (no hardware)
dimos run openarm-mock-planner-coordinator

# single-arm bring-up (hold the arm physically first)
dimos run coordinator-openarm-left

# full bimanual with planner
dimos run openarm-planner-coordinator
```

For the `openarm_rs` binding path, stage validation before trajectory control: binding mock or vcan, one motor enable/read, full-arm state monitor, adapter gravity compensation, then trajectory-control validation.

```bash
# requires the can-motor-control binding from dimos[manipulation]
sudo MODE=fd ./dimos/robot/manipulators/openarm/scripts/openarm_can_up.sh can0

# low-level coordinator: enables the arm and sends one current-position hold frame
dimos run coordinator-openarm-rs

# manipulation workflow: startup hold, then plan, preview, and execute through the new driver
dimos run openarm-rs-planner-coordinator
```

`OpenArmRSAdapter` opens CAN-FD by default (`canfd=True`) and computes model gravity feed-forward in-place when `gravity_comp=True` (the OpenArm blueprint default). It intentionally supports position and effort semantics only: position commands are sent as MIT commands with preset `kp/kd` gains and optional gravity feed-forward, while effort/gravity-only commands use `kp=0` so the arm does not hold a target pose. Velocity commands are rejected because nonzero gains make MIT commands maintain the supplied `q`. Set `gravity_comp=False` in adapter kwargs to keep position MIT commands but omit model feed-forward torque.

Meshcat will appear at http://localhost:7000.

### 5. Drive the new driver from the manipulation client

With `openarm-rs-planner-coordinator` running in one terminal, open a second terminal and start the REPL client:

```bash
python -i -m dimos.manipulation.planning.examples.manipulation_client
```

This gives you an interactive Python prompt with these functions:

| Function | Purpose |
|---|---|
| `robots()` | List configured robots (for `openarm-rs-planner-coordinator`: `["arm"]`) |
| `joints(robot_name)` | Read current joint positions (7 floats) |
| `ee(robot_name)` | Read current end-effector pose |
| `state()` | Module state: `IDLE`, `PLANNING`, `EXECUTING`, `FAULT`, etc. |
| `plan([q1..q7], robot_name)` | Plan a collision-free trajectory to a joint configuration |
| `plan_pose(x, y, z, robot_name=...)` | Plan to a Cartesian EE pose (preserves current orientation) |
| `preview(robot_name)` | Animate the planned path in Meshcat without executing |
| `execute(robot_name)` | Send the planned trajectory to the coordinator |
| `home(robot_name)` | Plan + execute to home joints |
| `commands()` | Print all available functions |

#### Example session — simple joint moves

```python skip
>>> robots()
['arm']

>>> joints()
[0.02, -0.01, -0.13, 0.15, 0.17, -0.07, 0.10]

>>> # One-liner: plan → preview in Meshcat → execute on hardware
>>> plan([0.3, 0, 0, 0, 0, 0, 0]) and preview() and execute()
True

>>> joints()
[0.30, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00]   # arm is now at the commanded pose
```

`plan()` returns `True` on success, `False` if planning failed (check the coordinator terminal for `COLLISION_AT_GOAL`, `INVALID_START`, `NO_SOLUTION`, etc). The `and` chaining is an idiom — if any step fails, the next one is short-circuited.

If you ever get stuck in a `FAULT` state (e.g. an invalid plan was sent), reset the state machine:

```python skip
>>> _client.reset()
'Reset to IDLE — ready for new commands'
```

#### Example session — bimanual legacy `openarm` blueprint

Use this pattern with `openarm-planner-coordinator`, which configures `left_arm` and `right_arm`. The new `openarm-rs-planner-coordinator` blueprint is intentionally single-arm while the binding-backed driver path is being validated.

```python skip
>>> # Move both arms to mirrored poses
>>> plan([0.5, 0, 0, 0, 0, 0, 0], robot_name="left_arm") and execute(robot_name="left_arm")
True
>>> plan([-0.5, 0, 0, 0, 0, 0, 0], robot_name="right_arm") and execute(robot_name="right_arm")
True
```

Each arm plans and executes independently — the coordinator runs both trajectories simultaneously on separate tick-loop tasks.

#### Example session — Cartesian target

```python skip
>>> ee()           # see where the EE currently is
>>> plan_pose(0.1, 0.3, 0.5) and preview()
True
>>> execute()
True
```

If you don't know which Cartesian targets are reachable, check first with the workspace tool — see [Workspace analysis](#workspace-analysis) below. `plan_pose` will fail with `NO_SOLUTION` if the IK can't find a configuration reaching the target.

#### Adding obstacles

```python skip
>>> add_box("table", 0.4, 0.0, 0.1, w=0.6, h=0.4, d=0.05)  # rectangular obstacle
>>> add_sphere("ball", 0.3, 0.2, 0.4, radius=0.05)
>>> plan_pose(0.4, 0.0, 0.3, robot_name="left_arm")         # now plans around it
>>> remove("table")                                          # id returned by add_*
```

---

## Configuration

### Which CAN bus is which arm

Linux assigns `can0`/`can1` in USB-enumeration order, which isn't guaranteed stable across reboots or cable swaps. If the arms come up "swapped" (commanding `left_arm` moves the physical right arm), flip these two constants at the top of [blueprints.py](/dimos/robot/manipulators/openarm/blueprints.py):

```python
LEFT_CAN = "can1"
RIGHT_CAN = "can0"
```

No other code changes are needed. The `coordinator-openarm-rs` and `openarm-rs-planner-coordinator` blueprints currently target `RIGHT_CAN` (`can0`) and pass `canfd=True`; use `sudo MODE=fd ... openarm_can_up.sh can0` before running either one.

### Gain tuning (MIT kp/kd)

Defaults live in the adapter implementations — pointing here instead of restating the numbers so docs can't drift from code. The binding-backed `openarm_rs` path uses the upstream OpenArm ROS2 hardware presets from `openarm_hardware/openarm_simple_hardware.hpp`; see `OpenArmRSAdapter._DEFAULT_KP` / `_DEFAULT_KD` in [openarm_rs/adapter.py](/dimos/hardware/manipulators/openarm_rs/adapter.py). The in-tree `openarm` adapter keeps its own `_DEFAULT_KP` / `_DEFAULT_KD` in [openarm/adapter.py](/dimos/hardware/manipulators/openarm/adapter.py).

Guidelines:
- `kp ∈ [0, 500]` in MIT mode. Higher kp = stiffer position tracking; too high → oscillation.
- `kd ∈ [0, 5]`. Higher kd = more damping, but values above ~2 on these gearboxes cause high-frequency buzz/grinding.
- Gravity compensation is on by default (`gravity_comp=True`) for OpenArm-style adapters. The adapter uses Pinocchio to compute `G(q)` and adds it as feedforward torque in the same command path. This removes the need for very high kp to fight gravity, so prefer upstream-tuned kp/kd + gravity comp over increasing kp.
- For `openarm_rs`, use position commands for trajectory/coordinator control and direct effort/gravity-only commands for torque bring-up. Velocity commands are intentionally unsupported.

### Physical joint limits

The URDFs use the xacro-generated limits (which include per-side offsets for mirroring). The adapter's `get_limits()` reports the same per-side limits. If you measure tighter physical limits and want to enforce them, edit the URDFs directly — the planner will respect them.

### Disabling auto MIT-mode write

The adapter writes `CTRL_MODE=MIT` to every motor at `connect()`. It's idempotent (writing the same value is a no-op), so this is safe to leave on. To verify that a previous write persisted across a power cycle, flip `AUTO_SET_MIT_MODE = False` in [blueprints.py](/dimos/robot/manipulators/openarm/blueprints.py) and restart — the arms should still respond.

---

## Motor mapping (OpenArm v10)

Derived from the URDF's `joint_limits.yaml` (effort column) cross-checked against the Damiao torque tables. Both arms are identical.

| Send ID | Recv ID | Joint | Motor | vMax [rad/s] | tMax [Nm] |
|---|---|---|---|---|---|
| 0x01 | 0x11 | joint1 | DM8006 | 45 | 40 |
| 0x02 | 0x12 | joint2 | DM8006 | 45 | 40 |
| 0x03 | 0x13 | joint3 | DM4340 | 8 | 28 |
| 0x04 | 0x14 | joint4 | DM4340 | 8 | 28 |
| 0x05 | 0x15 | joint5 | DM4310 | 30 | 10 |
| 0x06 | 0x16 | joint6 | DM4310 | 30 | 10 |
| 0x07 | 0x17 | joint7 | DM4310 | 30 | 10 |
| 0x08 | 0x18 | gripper | DM4310 | 30 | 10 |

Convention: `recv_id = send_id | 0x10`.

---

## Damiao protocol essentials

Ported from `enactic/openarm_can/src/openarm/damiao_motor/dm_motor_control.cpp`. You shouldn't need these unless you're modifying the driver.

### Enable / disable / zero-position

Send to the motor's send_id. 8-byte payload:

```
[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, CMD]
    where CMD = 0xFC (enable) | 0xFD (disable) | 0xFE (zero current pose)
```

### MIT control frame (8 bytes)

Bit layout: `q[16] | dq[12] | kp[12] | kd[12] | tau[12]`. Each float quantized via:

```python
def float_to_uint(x, lo, hi, bits):
    x = clamp(x, lo, hi)
    return round((x - lo) / (hi - lo) * ((1 << bits) - 1))
```

Gain ranges: `kp ∈ [0, 500]`, `kd ∈ [0, 5]`. Position/velocity/torque ranges come from the motor-type table above.

Byte layout:
```
byte0 = q_u >> 8
byte1 = q_u & 0xFF
byte2 = dq_u >> 4
byte3 = ((dq_u & 0xF) << 4) | ((kp_u >> 8) & 0xF)
byte4 = kp_u & 0xFF
byte5 = kd_u >> 4
byte6 = ((kd_u & 0xF) << 4) | ((tau_u >> 8) & 0xF)
byte7 = tau_u & 0xFF
```

### State reply (8 bytes, on recv_id)

Same `q | dq | tau` layout + 2 temperature bytes:

```
byte0 = motor_id_echo
byte1..5 = q | dq | tau (same packing as above)
byte6 = t_mos (°C)
byte7 = t_rotor (°C)
```

### CTRL_MODE register write

Broadcast frame on CAN ID `0x7FF`:

```
data = [send_id_lo, send_id_hi, 0x55, RID=10, val[0..3]]
     where val = 1 (MIT) | 2 (POS_VEL) | 3 (VEL) | 4 (POS_FORCE), little-endian uint32
```

Persistent across power cycles.

---

## Known gotchas

- **`ip link ... fd on` → `Operation not supported`.** gs_usb firmware doesn't support CAN-FD. Use classical CAN @ 1 Mbit (our bringup script's default).
- **Motors reply to probes but commands do nothing.** CTRL_MODE is not MIT. The adapter now writes MIT on connect, but if you disabled that and motors got reset, run `openarm_set_mit_mode.py`.
- **`COLLISION_AT_START` during planning.** `link5` and `link7` collision meshes overlap by 3 mm at every configuration. Handled by `OPENARM_COLLISION_EXCLUSIONS` in the catalog. If you see it anyway, the exclusion pairs may not be getting applied — check that the collision filter log line appears during world build.
- **`INVALID_START` during planning.** Hardware encoder noise pushed a joint 1 mrad past a URDF limit. Joint4 used to be exactly `lower=0.0` which tripped this — it's now `-0.01` to give breathing room. If you see it on a different joint, widen that limit by ~10 mrad.
- **"Transmit buffer full" (ENOBUFS) at 100 Hz.** Kernel TX queue too small. The bringup script sets `txqueuelen 1000`; the driver also retries on ENOBUFS. If you still see the error, check `ip -details link show canX | grep qlen`.
- **Arms swap sides.** USB enumeration order flipped. Swap `LEFT_CAN` / `RIGHT_CAN` in [blueprints.py](/dimos/robot/manipulators/openarm/blueprints.py).

---

## Design decisions

- **Driver separate from adapter.** `driver.py` has zero dimos deps → unit-testable with a virtual CAN bus, reusable outside dimos.
- **MIT mode for OpenArm RS position/effort.** The binding-backed `openarm_rs` path intentionally rejects velocity commands; nonzero MIT gains hold `q`, so velocity semantics are not exposed. Position uses preset `kp/kd`; effort/gravity-only commands use `kp=0`.
- **Gravity compensation on by default.** Eliminates steady-state position error without needing high kp. Needs Pinocchio + the per-side URDFs.
- **One adapter per CAN bus, keyed by `address`.** Matches the Piper adapter pattern. Bimanual = two adapters with different `address` values.
- **Per-side URDFs for Drake planning.** Loading the full 14-DOF bimanual URDF twice (once per robot instance) creates phantom-arm collisions with the "other" arm frozen at zero. The per-side URDFs keep only one arm's links + the torso, avoiding the phantom collisions while matching the bimanual kinematics exactly.
- **URDF stays in-tree (`data/openarm_description/`) for now.** Can migrate to LFS later — only the path constant in the catalog changes.
- **CAN bringup stays manual (`sudo`).** Auto-bringup from `connect()` would need sudo-in-a-library or a systemd unit; the explicit script is clearer and testable. For production, add a oneshot systemd unit that runs the script at boot.

---

## Workspace analysis

For figuring out which targets are reachable before planning, use the generic workspace tool:

```bash
# Visualize the left arm's reachable workspace as a point cloud
python -m dimos.utils.workspace data/openarm_description/urdf/robot/openarm_v10_left.urdf

# Check if a specific target is reachable
python -m dimos.utils.workspace data/openarm_description/urdf/robot/openarm_v10_left.urdf query 0.1 0.3 0.5

# Get a list of reachable poses near a target, ranked by manipulability
python -m dimos.utils.workspace data/openarm_description/urdf/robot/openarm_v10_left.urdf suggest 0.1 0.3 0.5

# Interactive: visualize + type targets to query
python -m dimos.utils.workspace data/openarm_description/urdf/robot/openarm_v10_left.urdf interactive
```

Points are colored by Yoshikawa manipulability index: green = dexterous, red = near singularity. Avoid planning targets in the red regions.

---

## Testing

```bash
# Unit tests (no hardware, use virtual CAN)
.venv/bin/python -m pytest dimos/hardware/manipulators/openarm/ -v
```

Expected: 24 passed (13 driver + 11 adapter). All tests use `can.Bus(interface="virtual")` loopback — no real hardware needed.
