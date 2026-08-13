# OpenYAM (Anvil Robotics) integration

Status: **adapter implemented, awaiting hardware verification.** Anvil has
not published the OpenYAM's motor BOM, CAN ID map, or URDF (their
`OpenYAM_hardware` / `OpenYAM_description` repos are placeholders), so this
integration is built on the I2RT YAM conventions the OpenYAM is patterned
after. Everything uncertain is an adapter kwarg; the probe script below
verifies each assumption against a real unit before any motor is enabled.

## Architecture

Same two-layer split as OpenArm:

- `dimos/hardware/manipulators/openyam/` — `OpenYamAdapter` plus YAM
  specifics (motor layout, transport resolution). The Damiao MIT-mode frame
  codec and bus class are reused from
  `dimos.hardware.manipulators.openarm.driver` — the wire protocol is
  identical.
- `dimos/robot/manipulators/openyam/` — config, blueprints, scripts. The
  planning model (`yam_description`, LFS-backed) was already wired before
  the hardware adapter existed; `make_openyam_hardware(adapter_type="openyam",
  address=YAM_CAN)` now connects it to a real arm.

Blueprints: `coordinator-openyam-can` (trajectory control),
`openyam-can-planner-coordinator` (planner + coordinator). The mock
variants (`coordinator-openyam`, …) are unchanged.

## Assumed hardware layout (verify before trusting)

| Motor | Send ID | Reply ID | Type (assumed) |
|-------|---------|----------|----------------|
| joint1–3 | 0x01–0x03 | send \| 0x10 | DM4340 |
| joint4–6 | 0x04–0x06 | send \| 0x10 | DM4310 |
| gripper | 0x08 | 0x18 | DM4310 |

Classical CAN @ 1 Mbit/s, MIT control mode. This matches the topology in
the canonical Linux-side integration (PR #3129, `OpenYamDamiaoAdapter` on
`can-motor-control`); opening the gripper decreases motor position.
Overrides (all
`adapter_kwargs`): `arm_motor_types`, `motor_ids`, `recv_id_offset`,
`gripper_motor_id`, `gripper_motor_type`, `kp`, `kd`, `position_lower`,
`position_upper`, `bitrate`, `interface`.

Wrong motor *type* mis-scales velocity/torque quantization (position is
unaffected — all DM types share ±12.5 rad); wrong *ID* means no replies.
Damiao motors only transmit state in reply to command frames, so a silent
bus with power on usually means wrong IDs, not a dead arm.

## Bring-up

### 1. Probe (both platforms)

```bash
# Passive first — prints every frame, moves nothing:
python dimos/robot/manipulators/openyam/scripts/openyam_can_probe.py --channel <CH> --listen 5
# Then active — enables each motor, reads one state frame, disables:
python dimos/robot/manipulators/openyam/scripts/openyam_can_probe.py --channel <CH>
```

If motors reply on unexpected IDs, the probe prints those frames — use them
to set `motor_ids` / `recv_id_offset` instead of editing code.

### 2a. Linux (SocketCAN)

```bash
sudo ./dimos/robot/manipulators/openyam/scripts/openyam_can_up.sh can0
# then --channel can0
```

### 2b. macOS (no SocketCAN)

The arm ships with a candlelight-firmware CANable 2.0 (VID:PID
`1d50:606f`). The adapter drives it through the shared
`dimos.hardware.can.gs_usb_bus.GsUsbMacBus` — the libusb userspace bus
hardware-validated on the Galaxea A1Z at a 250 Hz control loop — NOT
python-can's stock `gs_usb` backend, which mis-handles candlelight
devices from userspace (kernel-driver detach on a driverless OS, hardcoded
TX endpoint, unfiltered TX echoes; observed as a wedged dongle that drops
off USB). Dependencies:

```bash
brew install libusb
uv pip install 'python-can>=4.3' gs_usb
# probe / adapter address: gs_usb:0   (index of the dongle)
```

`resolve_transport` also maps `address="can0"` → `gs_usb` automatically on
macOS, so Linux-flavored configs run unmodified. If the dongle runs
CDC/serial firmware instead of candlelight, pass its serial path
(`/dev/tty.usbmodem*`) and the adapter uses SLCAN.

Known macOS caveats: gs_usb TX queues are small (the driver already retries
on ENOBUFS); a second dongle on the same machine sometimes needs the SLCAN
path; USB scheduling jitter makes the 100 Hz coordinator loop less regular
than SocketCAN on Linux — fine for bring-up and teleop, use a Linux box for
anything latency-critical.

## Calibration TODOs (first session with real hardware)

1. Probe → correct `arm_motor_types` / `motor_ids` in
   `dimos/hardware/manipulators/openyam/driver.py` defaults.
2. Gripper endpoints: `OPENYAM_GRIPPER_OPEN_RAD` / `_CLOSED_RAD` in
   `dimos/robot/manipulators/openyam/config.py` are placeholders (0.0 open,
   1.5 closed — opening decreases position). Zero the gripper motor open,
   close it by hand, read the probe's `q`, set the constant.
3. Joint limits default to ±π — replace with measured values (or Anvil's
   URDF when published) via `position_lower`/`position_upper`.
4. No gravity feedforward yet: unlike OpenArm there is no published mass
   model, so POSITION mode is pure PD + the motor's own behavior. Default
   gains are deliberately conservative; expect some sag under load until a
   URDF with inertials exists (then port OpenArm's Pinocchio path).

## Safety notes

- Always run the passive `--listen` probe before the active one on an
  unverified unit.
- MIT mode has no firmware watchdog by default on some DM firmware; if the
  control process dies mid-motion the last command keeps acting. Keep the
  e-stop reachable and the workspace clear on first enable.
- `write_enable(False)` (coordinator deactivate) drops all torque — the arm
  will fall. Hold it or rest it on the table before deactivating.
