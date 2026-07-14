# Galaxea A1Z — hardware bring-up runbook

The dimos side is **done and sim-verified** (branch `krishna/task/a1z-arm`). This is the
step-by-step to validate on real hardware the day the arm arrives. Unlike the A1X, the
A1Z has a **public open-source SDK** and a **classic-CAN direct-to-motor** protocol, so
there is no reverse-engineering — it's a clean bring-up.

**Safety conventions (from the A1X bring-up):** docs-first; read-only before commanding;
climb the ladder (passive → no-op → tiny delta → teleop); keep the e-stop / power switch
in reach; the A1Z SDK also has its own temperature / stale-feedback / command-flood soft
e-stops, and `zero_gravity_mode` lets the arm float for hand-guiding.

## Hardware facts (from the SDK + URDF)
- 6-DOF, motors on **classic CAN** (`can0`), **1 Mbps**, MIT force-position protocol.
- Motor IDs: joints 1-3 = MotorA `0x01/0x02/0x03` (±50 Nm), joints 4-6 = MotorB
  `0x04/0x05/0x06` (±7-28 Nm).
- Joint limits (rad): j1 ±2.094, j2 [0, 3.142], j3 [-3.142, 0], j4 ±1.309, j5 ±1.484,
  j6 ±2.007.
- Adapter: HHS USB-CANFD (VID:PID a8fa:8598) per Galaxea docs — but **any socketcan
  adapter works**, incl. the PEAK PCAN-USB Pro FD we already have (run it in classic-CAN
  mode, not FD).

## Step 0 — install the SDK (one-time, on the laptop)
```bash
pip install "a1z @ git+https://github.com/userguide-galaxea/GALAXEA-A1Z.git"
# or clone + pip install -e . ; deps (numpy, python-can, pin) already in dimos
python -c "from a1z.robots.get_robot import get_a1z_robot; print('a1z SDK ok')"
```

## Step 1 — wire + bring up CAN (classic, NOT FD)
Wire per the arm's CAN connector: CANH ↔ CANH, CANL ↔ CANL (+ GND if the connector has
it), 120 Ω termination on. Then:
```bash
sudo ip link set can0 up type can bitrate 1000000     # classic CAN, 1 Mbps — no 'fd on'
candump can0                                            # passive listen
```
Power the arm on. Expect per-motor frames on IDs 0x01-0x06 (request/response — the arm
may be quiet until the SDK polls it; that's fine, unlike the A1X the A1Z is host-driven).

## Step 2 — SDK-level read-only sanity (no motion)
The SDK's own examples are the safest first contact. Read state without commanding:
```bash
python - <<'EOF'
from a1z.robots.get_robot import get_a1z_robot
r = get_a1z_robot(can_channel="can0", zero_gravity_mode=True)
r.start()                       # enables motors; zero-gravity so it floats, no snap
print("pos (rad):", r.get_joint_state()["pos"])
r.stop()                        # disables motors
EOF
```
Cross-check the printed angles against the arm's physical pose. In `zero_gravity_mode`
the arm is limp/hand-guidable — move a joint by hand and re-read to confirm the sign and
which index maps to which joint (matches `arm_joint1..6`).

## Step 3 — dimos coordinator on real hardware (feedback first)
```bash
dimos --can-port can0 run coordinator-a1z
```
This builds the `a1z` adapter (real), enables motors (auto_enable), and ticks at 100 Hz.
Watch the log for `Added hardware arm` + joint state flowing. No trajectory is commanded
yet — the coordinator just holds/reads. Ctrl-C to stop (adapter disables motors).

## Step 4 — first motion: tiny, planned, supervised
Two terminals. Keep the power switch in reach; be ready to Ctrl-C.
```bash
# Terminal A — planner + coordinator + Meshcat
dimos --can-port can0 run a1z-planner-coordinator \
  -o manipulationmodule.visualization.backend=meshcat
# Terminal B — interactive planning client
python -m dimos.manipulation.planning.examples.manipulation_client
```
In the client:
```python
joints()                                  # read current (should match physical)
plan([j + 0.05 for j in joints()])        # TINY 0.05 rad nudge from current
preview()                                  # inspect in Meshcat BEFORE executing
execute()                                  # supervised — hand on the switch
joints()                                   # confirm it moved to ~target
```
Respect the limits (§ hardware facts) — j2 is [0, 3.142], j3 is [-3.142, 0], so a naive
all-positive target is out of range for j3 and the planner will refuse (good).

## Step 5 — teleop
```bash
dimos --can-port can0 run keyboard-teleop-a1z     # + open http://localhost:7000
```
Keys drive the end-effector via IK (W/S/A/D/Q/E linear, R/F T/G Y/H angular, Esc quit).
Start with small motions; the SDK's soft e-stops backstop a runaway.

## Notes / gotchas
- **zero_gravity vs position-hold:** the dimos adapter defaults to position-hold
  (`zero_gravity=False`) so commanded positions are actually tracked. Pass
  `zero_gravity=True` via adapter_kwargs only for hand-guided teach mode.
- **PEAK adapter:** bring it up in **classic** mode for the A1Z (`bitrate 1000000`, no
  `fd on`) — the A1Z is classic CAN, unlike the A1X which needs FD.
- If `candump` is silent after the SDK starts, check motor power + IDs (the SDK logs
  "receive timeout. Check motor power and ID.").
- Log findings here as a dated trail (see the A1X `BRINGUP_LOG.md` for the format).
