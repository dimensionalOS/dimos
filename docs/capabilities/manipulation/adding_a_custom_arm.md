---
title: "How to Integrate a New Manipulator Arm"
---
An **adapter** wraps your vendor's SDK. The `ControlCoordinator` calls it at 100Hz. Write it, and motion planning, teleop, and pick-and-place all work with your arm without knowing it exists.

## What the coordinator calls

```
connect()  →  write_enable(True)

every tick:
    read_joint_positions()    # radians
    read_joint_velocities()   # rad/s
    read_joint_efforts()      # Nm
        ... coordinator decides what to command ...
    set_control_mode(mode)    # only when it changes
    write_joint_positions()   # radians  (or write_joint_velocities)

disconnect()
```

Your adapter owns no threads. It is passive, the coordinator calls it.

## 1. Copy the mock

```bash
cp -r dimos/hardware/manipulators/mock dimos/hardware/manipulators/yourarm
```

`mock/adapter.py` implements all 26 methods against fake state — a working adapter with the vendor calls missing. Replace the fakes. No base class, match the method names and DimOS accepts it.

## 2. Convert units, every time

Your SDK almost certainly speaks degrees. DimOS speaks radians, and SI units, and they go straight into the IK solver.

```python skip
def read_joint_positions(self) -> list[float]:
    raw = self._sdk.get_joint_positions()               # degrees
    return [math.radians(p) for p in raw[:self._dof]]   # radians

def write_joint_positions(self, positions, velocity=1.0) -> bool:
    return self._sdk.set_joint_positions([math.degrees(p) for p in positions])
```

Grippers are separate from joints, and they work in **meters** of opening:

```python skip
def write_gripper_position(self, position: float) -> bool:
    return self._sdk.set_gripper_mm(position * 1000.0)
```

## 3. Missing features are fine

No velocity feedback? Return zeros. **Not `[]`** — the coordinator indexes these lists positionally, so an empty one is an `IndexError` on the next tick.

```python skip
def read_joint_velocities(self) -> list[float]:
    return [0.0] * self._dof
```

No force-torque sensor, no Cartesian mode? Reads return `None`, writes return `False`. Never raise: an exception on the tick thread is not caught.

```python skip
def read_force_torque(self) -> list[float] | None:
    return None
```

## 4. Two things that will bite you

```python skip
def __init__(self, address="192.168.1.100", dof=6, **_: object) -> None:
    #                                              ^^^^^^^^^^^^
    # The coordinator passes hardware_id and more. Without this: TypeError.

def connect(self) -> bool:
    from yourarm_sdk import YourArmSDK   # import HERE, not at module scope.
    ...                                  # else `import dimos` breaks for
                                         # everyone without your arm.
```

## 5. Register and run

```python skip
# dimos/hardware/manipulators/yourarm/_registry.py    (stdlib imports only)
ADAPTER_FACTORIES = {"yourarm": "dimos.hardware.manipulators.yourarm.adapter:YourArmAdapter"}
```

Add `"yourarm"` to `EXPECTED_NAMES` in `dimos/hardware/test_adapter_registries.py`. Then, under `dimos/robot/manipulators/yourarm/`:

```python skip
# config.py
def yourarm_hardware(hw_id="arm") -> HardwareComponent:
    return HardwareComponent(
        hardware_id=hw_id,
        hardware_type=HardwareType.MANIPULATOR,
        joints=make_joints(hw_id, 6),        # ['arm/joint1', ...]
        adapter_type="yourarm",              # matches ADAPTER_FACTORIES
        address="192.168.1.100",             # handed to your __init__
    )

# blueprints/basic.py
_hw = yourarm_hardware("arm")
coordinator_yourarm = autoconnect(coordinator(hardware=[_hw], tasks=[trajectory_task(_hw)]))
```

`autoconnect` is also what registers the blueprint's CLI name. Build one another way and `dimos run` will not find it.

```bash
pytest dimos/robot/test_all_blueprints_generation.py   # regenerates the registry; commit it
dimos run coordinator-yourarm
```

Set `adapter_type="mock"` and all of this runs with no arm attached. Do that first — then the only thing left to debug on hardware is your SDK calls.

## Going further

Real examples: `a750/adapter.py` (vendor SDK), `openarm/` (raw SocketCAN, no SDK).

Motion planning needs a URDF and a `RobotModelConfig`; Cartesian and VR teleop each add a coordinator task. All of it, plus templates and a verification loop, is in [Adding a New Arm](/docs/coding-agents/adding_a_new_arm.md) — the page to hand an AI agent.
