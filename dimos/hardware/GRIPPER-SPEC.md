# Spec — Gripper API Refactor

## 1. Principle

> **`write_joint_positions()` is the API for all joints. A gripper joint is just a robot
> joint of a special type, unified from the consumer's perspective.**

Read as two clauses, one per layer:

- **Above the task** — `GripperControlTask` owns the gripper. Anything that interfaces
  with a gripper goes through it. There is no other way in.
- **Below the task** — one ordered array, one `write_joint_positions()`. A gripper entry
  differs from an arm entry only in the unit it carries.

**This spec is PR 1: the existing gripper path**, so every claim in it is verifiable on
hardware we already own — xArm, Piper, a1z, a750. PR 1 ships in **four separately
verifiable parts** (§7.1); the split changes only the order things arrive in, nothing
below changes because of it.

Supporting a **third-party** gripper is the goal that motivated the whole effort, and it is
PR 2 (§7). It builds directly on this: the joint model, the control task, and the units all
carry over unchanged. What PR 2 adds is the plumbing to give a gripper its own adapter.

> **Baseline.** Every `file:line` reference in this document resolves at commit
> `e994783d9`. Where a symbol name is given alongside a line, the symbol is
> authoritative — lines shift as the parts land.

---

## 2. At a glance

One path, end to end. A gripper joint travels the same route as an arm joint and differs
only in the unit it carries.

```
  PROPOSED ARCHITECTURE

  consumer        agent skill (0-1) . teleop . grasp policy (native)
                  no new ports                                  [R14 R26 R27]
                            |
                            |  task_invoke("arm_gripper", ...)      [R23 R26]
                            |  gripper_command . teleop_buttons     [R16]
                            v
  control task    GripperControlTask - sole owner of gripper joints
                  converts once . two modes . no other way in   [R14 R16-R22]
                            |
                            v
  wire            coordinator_joint_state
                  vectors, in each joint's declared unit            [R11 R12]
                            |
                            v
  arbitration     tick loop - priority, preemption, e-stop
                  unchanged; grippers now take part for the first time
                            |
                            v
  wrapper         ConnectedHardware - one ordered array, no conversion   [R25]
                            |
                            v
  adapter         the arm's own adapter                          [R4 R6 R7 R8]
                  all_joints  = [...arm..., ...gripper...]
                  gripper_dof = how many trail
                            |
                            v
  hardware        one connection, one vendor SDK                       [R3]
```

> The standalone gripper shape — its own component, its own adapter — is PR 2 (§7).

### 2.1 What changes

| | Added | Deleted |
|---|---|---|
| **Model** | `gripper_dof` on the component; `arm_joints` / `gripper_joints` derived | the `joints` field; `read/write_gripper_position` |
| **Control** | `GripperControlTask` — sole owner; inherits `gripper_command` and `teleop_buttons` | both gripper RPCs; `claim_with_gripper`, `append_gripper_position`; the arm tasks' gripper config |
| **Units** | adapter declares its range via `get_limits()`; task converts | `gripper_open/closed_position`; `_normalized_to_physical` |

---

## 3. Why this refactor (Current Status)

### 3.1 A shortcut route to hardware

Arm joints travel: task → arbitration → tick loop → adapter. Gripper joints skip all of it:

```python
# CURRENT — coordinator.py:901-916
@rpc
def set_gripper_position(self, hardware_id, position) -> bool:
    return hw.adapter.write_gripper_position(position)      # straight to hardware
```

No task, no arbitration, no priority, no preemption.

### 3.2 A single-joint ceiling

`write_gripper_position(position: float)` takes one scalar (`manipulators/spec.py:229`) and
`make_gripper_joints()` returns exactly one name (`components.py:106-115`). A multi-joint
gripper cannot be expressed.

### 3.3 A third-party gripper cannot be supported at all — *the motivation*

Gripper control lives inside each arm's adapter, so a gripper is reachable only through the
arm that ships with it. A gripper from another vendor has no path.

> This is why the refactor exists, and it is **not fixed by this PR**. It is fixed in PR 2
> (§7), which needs a real device to verify against. What this PR does is make that possible:
> a gripper joint stops being a special case everywhere above the adapter, so adding a second
> kind of adapter is the only remaining step.

### 3.4 A gripper joint has no single owner

`teleop_task` and `eef_twist_task` both claim gripper joints via `claim_with_gripper`
(`cartesian_ik_task.py:61`) at differing priorities, and the RPC in §3.1 bypasses
arbitration entirely. Which input reaches the hardware depends on configuration and on
blueprint ordering.

### 3.5 An undefined unit contract, with a live bug

`a1z` and `piper` emit normalized `0.0–1.0`; `xarm` emits adapter-native `0.85`. All three
feed the same `ConnectedHardware._normalized_to_physical`, so the xArm value is mapped twice:

```
CURRENT BEHAVIOUR — measured against the real xarm7 teleop blueprint

task emits for FULLY OPEN      = 0.85
value reaching adapter         = 0.7225        (0.85 × 0.85)
xArm SDK set_gripper_position  = 722.5         (expected 850.0)
shortfall                      = 15.0% of travel lost
```

Reproduced against the real repository configuration. Root cause: conversion happens in
three places — blueprints, the hardware wrapper, and adapters — with no rule saying which is
authoritative.

---

## 4. Requirements

### 4.1 Two device shapes, one control path

**R1.** A gripper is modelled as **joints on the arm's `HardwareComponent`**, reached
through the arm's adapter — every gripper we have today is a capability of its arm's SDK
(xArm, Piper, a1z, a750).

> The **standalone** shape — a gripper as its own component with its own adapter — is PR 2
> (§7). There is no device to verify it against yet.

**R2.** Above the hardware layer a gripper joint is treated identically to an arm joint:
same task, same units, same joint representation, same `write_joint_positions()`.

**R3.** No connection-sharing mechanism is introduced. An integrated gripper is reached
through the arm's existing adapter because it *is* part of that device.

### 4.2 Adapter protocols

**R4.** `write_joint_positions()` and `read_joint_positions()` MUST cover **all** joints
the adapter owns, in `all_joints` order, with gripper entries trailing.

```python
# PROPOSED
write_joint_positions([0.1, -0.4, 0.0, 0.9, 0.0, 1.2,  0.042])
#                      └──── 6 arm joints, radians ─┘  └ gripper, native (R12)
```

> `all_joints` is the stored list and *is* the adapter's array; `arm_joints` and
> `gripper_joints` are derived from it (R28).

**R4a.** Velocity writes carry the same all-joints array as positions and reads.
Whether a gripper joint acts on a velocity command is the **adapter's capability
decision**: ignore the trailing entries (xArm), refuse (Piper, a1z), report `0.0` on
reads where no sensor exists. The wrapper special-cases nothing on either path.
*(Revised on review from an arm-only write rule.)*

**R5.** `read_gripper_position()` and `write_gripper_position()` are **removed** from
`ManipulatorAdapter` (`manipulators/spec.py:225-231`). They are a single-scalar API, which
R11 replaces with vectors.

**R6.** How an adapter fulfils `write_joint_positions()` internally is **not specified**.
An integrated gripper is, by definition, a capability of the arm's own SDK, so that
adapter necessarily reaches it through the vendor's own call — but that is an
implementation detail of one adapter, invisible above the hardware layer, and it places no
constraint on any other gripper.

> **This does not limit third-party support.** A gripper from another vendor gets its own
> adapter in PR 2 (§7); nothing about how the xArm adapter talks to its own gripper
> constrains it.

<details><summary>Illustrative — one way an integrated adapter might do it</summary>

```python
# PROPOSED — illustrative only, not required by this spec
def write_joint_positions(self, positions, velocity=1.0):
    arm, grip = positions[: self._arm_dof], positions[self._arm_dof :]
    ok = self._arm.set_servo_angle_j([math.degrees(p) for p in arm], ...) == 0
    if grip:
        # grip[0] is ALREADY in this adapter's declared unit (R12/R13) — the
        # 0-850 the xArm SDK takes. No scaling here. Scaling it would be §3.5.
        ok = self._arm.set_gripper_position(grip[0], wait=False) == 0 and ok
    return ok
```

Non-normative. An adapter may satisfy R4 however its SDK requires — but it may **not**
convert the gripper entry. The value handed to it is already in the unit it declared
through `get_limits()`; a second conversion here is precisely the bug §3.5 measures.

</details>

**R7.** An adapter receives its gripper joint count from the component's `gripper_dof`
(R28), as a `gripper_dof` constructor kwarg. It MUST NOT infer it from array length, and
it is no longer declared separately.

**R8.** `get_dof()` reports **arm joints only** — its meaning today is unchanged.
`get_gripper_dof()` reports gripper joints, returning `0` when there are none. A caller
wanting the total adds them. On a `GripperAdapter`, `get_dof()` reports its own joints.

> **Confirmed 2026-08-07: the carve-out stands.** It only looked odd while a standalone
> gripper rode `ManipulatorAdapter` (§7.2's demonstration used `get_dof() == 0` there,
> and worked). Inside a dedicated `GripperAdapter` protocol there is no arm to be
> ambiguous with, so `get_dof()` naturally means "my joints".

> `get_dof()` is therefore **not** the length of `get_limits()`, `read_joint_positions()`
> or the `write_joint_positions()` array — those cover all joints (R4, R13). This is the
> one place in the protocol where two lengths are legitimately different; R13 states it.

**R9.** *Moved to PR 2 — branch `jhengyi/gripper_adapter_pr2` (§8).* **Shape confirmed**
(2026-08-07, per the 2026-08-04 design meeting): a **separate, minimal protocol** — the
fourth adapter kind alongside manipulators, drive trains and whole-body — deliberately
kept small ("basic lifecycle stuff"), merging with `ManipulatorAdapter` later only if
the two converge. Grippers get per-device folders under `hardware/grippers/`, each
carrying its own `adapter.py` and, for SDK-less devices like the H100, its
`driver.py` / `transport.py`.

> **The drift guard.** The shared joint-array surface — `read/write_joint_positions`,
> the R4/R4a rules, `get_limits`, `get_gripper_dof` — MUST stay signature-identical
> with `ManipulatorAdapter`, enforced by a **conformance test** that fails CI if the two
> protocols disagree. One contract, stated twice, checked mechanically: this is how a
> separate protocol coexists with "``write_joint_positions()`` is the API for all
> joints" without re-creating §3.5's two-authorities shape.

PR 2 opens with the protocol **proposal**; implementation starts after team approval —
the sequence agreed in the design meeting.

**R10.** A **joint** means an *actuated degree of freedom*, not necessarily a mechanical
hinge. A pneumatic or suction gripper is one joint with limits `[0, 1]`; a
pressure-controlled gripper is one joint whose native unit is kPa. A soft continuum
gripper with no discrete actuation state is **out of scope** — it would need its own
message type, not a looser version of this one.

### 4.3 Units — joint-native on the wire, task converts

**R11.** Every **per-joint** gripper API is a **vector**, never a scalar — the adapter
protocol (R4), and the task's `set_position` / `set_normalized` / `set_reference_pose` /
`get_position`. This is what removes the single-joint ceiling in §3.2.

> The **intent** APIs are deliberately one number: `set_sweep(v)` (R18) and the agent
> skills (R26). They are not exceptions to R11 — they carry a scalar *intent*, which the
> task expands into a joint vector before anything below it sees a value. A scalar that
> reaches the protocol is a ceiling; a scalar that stops at the task is an interface.

**R12.** A gripper joint carries **the unit its adapter actually speaks** wherever it
appears — `coordinator_joint_state`, and every array below the task — metres for a
sliding jaw, radians for a rotating knuckle, or the vendor's own scale where the SDK is
dimensionless. Arm joints are unchanged (radians).

> Gripper *commands* do not ride `joint_command`. They reach `GripperControlTask` (R16),
> which is what makes R17's single owner true: a raw publish that bypassed the owner
> would be §3.1's shortcut wearing a different hat. `joint_command` stays purely
> arm-valued — which is also what stops a sender inventing a number it cannot know (see
> R16's note on the keyboard).

**R13.** The **adapter declares its range** and exposes it via `get_limits()`. A private
module constant remains the implementation; `get_limits()` makes it readable. Two adapters
already have the constant — `piper/adapter.py` `GRIPPER_MAX_OPENING_M = 0.08`,
`galaxea_a1z/config.py` `max_opening_m = 0.1`.

`get_limits()` follows the **same shape as the adapter's array** (R4): one entry per
joint, in `all_joints` order, gripper entries trailing. Its length is therefore
`get_dof() + get_gripper_dof()` — deliberately *not* `get_dof()`, which stays arm-only
under R8. One ordering convention governs the whole adapter surface. No caller relies on
today's arm-only length: `get_limits()` currently has zero call sites outside the
adapters themselves.

What each adapter declares for its gripper:

| Adapter | Gripper range | Verified on |
|---|---|---|
| `xarm` | `(0.0, 850.0)` — dimensionless SDK scale, mm conversion dropped | xArm6 / xArm7 |
| `piper` | `(0.0, 0.08)` m — the existing `GRIPPER_MAX_OPENING_M` | Piper |
| `galaxea_a1z` | `(0.0, 0.1)` m — the existing `max_opening_m` | a1z |
| `sim` | the MJCF joint range, over SHM (R13a) | any sim scene |
| `a750` | `(0.0, 0.06)` m — the existing `GRIPPER_MAX_OPENING_M` | **not verified** |
| `mock` | `(0.0, 1.0)`, overridable per test | unit tests |
| `openarm` | none — `get_gripper_dof()` returns `0` | — |

> **The declared unit is what the adapter actually speaks.** Piper (`0.08`) and a1z (`0.1`)
> are genuinely metres. xArm is not: `MM_TO_M`/`M_TO_MM` belong to cartesian pose and were
> reused on the gripper path (`xarm/adapter.py:386, :397`), where the SDK takes a
> dimensionless `0–850`. Today's `0.85` is just `850/1000`. xArm therefore declares
> `(0.0, 850.0)` and drops the mm conversion.

> **a750 is converted blind** — §7 lists step-1 hardware as xArm, Piper and a1z, and there
> is no a750 on the bench. It is converted anyway because the protocol change is global;
> leaving it behind would make it the one adapter off the protocol, which is the dead code
> §7.1's rule forbids. Its joint keeps the name `arm/finger`: the task claims whatever
> `hw.gripper_joints` returns, so the name never routes anything, and renaming is
> user-visible churn for no gain.
>
> Pre-existing and **not fixed here**: `keyboard_teleop_a750` has no gripper task and
> `KeyboardTeleopModule` hardcodes `"arm/gripper"`, so a750's gripper is unreachable today
> — same class of defect as `coordinator_teleop_dual` (R30). File both together.

**R13a.** The **sim** adapter learns its range from the MJCF, over SHM. Its native gripper
unit is genuinely a joint position in the model's own range — the value it writes is
clamped into `MujocoSimModule._gripper_joint_range` before actuation — and that range
lives in another process, so the adapter cannot otherwise know it.

The `grp` SHM segment widens from `[position, target]` to
`[position, target, range_lo, range_hi]`. `MujocoSimModule` writes the range once, where
it already detects the gripper; `sim/adapter.py` reads it on connect and returns it from
`get_limits()`.

> The MJCF stays the single declaration — a blueprint-config range would be a second place
> that can disagree with the model, the exact shape of §3.5. This also discharges R7 for
> sim, which today infers its gripper by counting (`_has_gripper = num_joints > self._dof`).
> Scope: adds `simulation/engines/mujoco_shm.py` and `mujoco_sim_module.py` to §6's table,
> about twelve lines.

**R14.** `GripperControlTask` converts between scales in exactly one place: itself. It
exposes **both** interfaces:

```python
# PROPOSED — GripperControlTask
set_position(values: list[float])     # native units, per joint
set_normalized(values: list[float])   # 0.0–1.0 per joint → lo + (hi-lo)*v
```

Nothing above the task ever needs a vendor range; anything needing real units can have them.

**R14a.** How the task obtains its range, precisely — this chain is normative because the
obvious shortcut violates R7:

1. The task factory receives the coordinator's hardware dict —
   `create_task(cfg, hardware)` with `hardware: {hardware_id: ConnectedHardware}`.
   `_setup_from_config()` connects hardware before creating tasks, so the adapter is
   live at construction.
2. Every name in `cfg.joint_names` MUST resolve (via `split_joint_name`) to **one**
   hardware id — a gripper task spans exactly one device — and MUST equal that
   component's `gripper_joints`, in order. Any mismatch raises at construction.
3. `limits = connected.adapter.get_limits()`; the gripper entries are the trailing
   slice at `len(component.all_joints) - component.gripper_dof` — the R28 split, taken
   from the **component's declared count**, never from comparing array lengths (R7).
4. The per-joint `(lo, hi)` pairs are cached at construction; ranges are static.

**R15.** `HardwareComponent.gripper_open_position` / `gripper_closed_position` are removed
(`components.py:97-98`), together with `_normalized_to_physical` /
`_physical_to_normalized` (`hardware_interface.py:232-244`). Conversion happens in exactly
one place: the task.

> Normalization is `0.0` = fully closed, `1.0` = fully open. This is not a new convention —
> it is what `_normalized_to_physical` already implements, what
> `galaxea_a1z/adapter.py:479` states in its docstring, and what
> `keyboard_teleop_module.py:64-65` already uses.

### 4.4 `GripperControlTask`

**R16.** A new task type `gripper`. It is invoked by command, and it consumes exactly the
two streams that carry **gripper intent**:

```python
# PROPOSED — new task type, does not exist today
TASK_CONSUMES = {"gripper": {
    "gripper_command": ("on_gripper_command", "broadcast"),   # Bool: open / closed
    "teleop_buttons":  ("on_teleop_buttons",  "broadcast"),   # analog trigger, hand-scoped
}}
TASK_EXPOSES  = {"gripper": ["set_position", "set_normalized", "set_sweep",
                             "set_reference_pose", "get_position",
                             "get_normalized", "get_state"]}
```

Numeric targets arrive through `TASK_EXPOSES` via the existing `task_invoke` RPC
(`ControlCoordinator.task_invoke`, `coordinator.py:803`) — an RPC **into a task**, which
then flows through arbitration and the tick loop.

> **The task consumes these streams itself; nothing forwards.** There is no inter-task
> mechanism to forward with — a task is constructed with the hardware dict and holds no
> reference to any other task — and none is added. The arm tasks simply delete their
> gripper code. Both ports already exist on the coordinator; they move from `eef_twist`
> and `teleop_ik` to their new owner. **No new ports.**
>
> **Both streams carry intent, never a joint value.** A browser toggle says *"closed"*;
> a trigger says *"squeezed 42%"*. Neither sender knows the gripper's travel, and neither
> should (R14). `KeyboardTeleopModule`'s `[` / `]` therefore move off `joint_command`
> and onto `gripper_command`: today they publish `1.0` as a joint value, which on a1z's
> `0–0.1` m range is ten times over-range and survives only because the adapter clamps.
> A Bool is what the keyboard actually has.
>
> **Routing note (deferral confirmed 2026-08-07):** `broadcast` is correct while each
> blueprint has one gripper. The moment two gripper tasks coexist (arm gripper +
> standalone H100), `gripper_command` needs `by_task_name` routing, as
> `coordinator_ee_twist_command` already does. This lands in **step 3** — the PR that
> first makes two grippers coexist.

> **The task owns the gripper completely** — it commands it, and it reports it.
> `get_position` returns the **measured** position from the `CoordinatorState`
> snapshot; `get_normalized` returns the same reading mapped to `0.0`–`1.0` of travel —
> the task converts, since it owns the range (R14). `get_state` reports what the task
> is doing and carries its limits for consumers with physical targets (R18) —
> illustratively:
> `{"state": "holding"|"idle", "target": [...] | None, "joints": [...], "limits": [(lo, hi), ...]}`.
>
> This cannot disagree with `coordinator_joint_state`: `tick_loop.py:180-195` passes the
> *same* `joint_states` object to `compute()` and to the publisher. The stream and this RPC
> are one snapshot, offered as a subscription and as a one-shot read.

**R17.** It is the **sole claimant of gripper joints**. `teleop_task` and `eef_twist_task`
drop `claim_with_gripper` and `append_gripper_position`; the helpers
(`cartesian_ik_task.py:61-72`) are deleted. They forward nothing — their inputs already
reach the gripper task directly (R16). One joint, one owner.

> **The trigger's polarity inverts, and this must be deliberate.** Today
> `TeleopIKTask.on_gripper_trigger` maps trigger `0 → open`, `1 → closed`: squeezing
> closes. On the `0.0` = closed / `1.0` = open scale every API in R19 shares, that is
> `set_normalized([1.0 - trigger])`. Get it wrong and there is no error — just a gripper
> that opens when told to close. Pin it with a test.
>
> **The VR trigger keeps its engagement gate.** Today the trigger only reaches hardware
> while the operator is engaged (`TeleopIKTask.is_active()` requires a live pose stream);
> moving it to its own task would otherwise drop that gate, and a disengaged operator
> resting a finger would close the gripper. The gripper task reproduces it: accept the
> trigger only while the configured hand's primary button is held, behind
> `require_engagement: bool = True`.

**R17a.** Their gripper **configuration** goes with it. `gripper_joint`,
`gripper_open_pos`, and `gripper_closed_pos` are removed from both task configs
(`teleop_task.py:72-74, 268-270`; `eef_twist_task.py:49-51, 193-195`), and the blueprint
constants that feed them are deleted — `XARM_GRIPPER_PARAMS` (`xarm/config.py:61-65`) plus
the inline `params={...}` gripper dicts in the `a1z` and `piper` teleop blueprints.

> Endpoint values disappear from the task layer entirely. Under R13/R14 the range comes
> from `get_limits()` and normalization happens inside `GripperControlTask`, so no
> blueprint declares an open/closed endpoint any more. This is what removes the duplicated
> endpoints that caused §3.5 — there is no longer a second place to disagree.

**R18.** Two control modes:

| Mode | API | Use |
|---|---|---|
| **Fine-grained** | `set_position([...])` / `set_normalized([...])` | per-joint control; grasp policies |
| **Scalar** | `set_sweep(value: float)` | one number: `0.0` = fully at the **reference pose**, `1.0` = fully open |

**R19.** `set_sweep(v)` interpolates between an **open posture** and a **reference
posture**, both joint vectors. **`v` has the same meaning as in `set_normalized`: `0.0` is
fully closed, `1.0` is fully open.**

```
PROPOSED

sweep(v) = reference_pose + (open_posture - reference_pose) * v
           v = 0.0 → fully at the reference pose (the grasp, applied)
           v = 1.0 → fully open
```

**R19a.** The **reference pose is the vendor's grasp pose**, not a derivation from joint
limits. Joint limits describe travel, not grasping — driving every joint of a multi-finger
hand to its extreme is a fist or a self-collision, not any grasp the vendor ships.

| Gripper | `open_posture` | `reference_pose` |
|---|---|---|
| Single joint | its open limit | its closed limit — for a jaw, closed *is* the grasp |
| Multi-joint | its joints' open limits | **MUST be declared in config.** No default exists |

**R19b.** `set_sweep` MUST refuse, with a clear error, on a multi-joint gripper that has no
reference pose configured. It MUST NOT fall back to interpolating against joint limits.

> A vendor's documented grasp poses become named vectors in blueprint config — the H100's
> four gestures (two-finger, three-finger, and two pinches) are four entries, any of which
> may be selected as the reference. `set_reference_pose()` replaces it at runtime, so a
> grasp planner can supply a computed pose and `set_sweep` then scales *that* grasp.

> **Polarity is identical across every scalar API.** `set_normalized([0.0, ...])`,
> `set_sweep(0.0)`, and `close_gripper()` all close; `1.0` opens in each case. A mismatch
> here produces no error — just a gripper that opens when told to close.

The protocol itself knows only vectors — postures live in config, never in `GripperAdapter`.

**R20.** **Every** command in `TASK_EXPOSES` MUST return immediately — `set_position`,
`set_normalized`, `set_sweep`, `set_reference_pose`, `get_position`, `get_state`. The
setters record the target and refresh the hold; the readers return the latest snapshot.
None may block, wait, or sleep. (Nothing "reactivates" — under R21a the task is always
active; a setter only changes what `compute()` emits on the next tick.)

> **Hard constraint.** `task_invoke` holds `_task_lock` (`coordinator.py:813`) and the tick
> loop needs that same lock every tick (`tick_loop.py:268`). Blocking inside a task command
> would stall the **entire coordinator** — no arbitration, no hardware writes for any
> joint — not merely the gripper.

**R21.** The task emits its target for a bounded hold duration, then stops emitting. It MUST
NOT stop on "measured position reached target": a gripper stalled on a grasped object
never reaches its commanded position, so that condition would misreport every successful
grasp. Once it stops emitting, `ConnectedHardware` holds the last commanded value in its
array and keeps re-sending it every tick, so closing force is maintained.

`hold_duration` is configurable and defaults to `0.0`, meaning **hold indefinitely** —
matching the `servo_gripper` tasks it replaces, which are configured `timeout: 0.0`
(`a1z/blueprints/teleop.py:50`, `piper/blueprints/teleop.py:65`).

**R21a.** `is_active()` returns **True always**; `compute()` decides what to emit.

> **Why.** The tick loop passes state only to tasks whose `is_active()` is True
> (`tick_loop.py:270`). A task that went inactive at the end of its hold would stop
> receiving snapshots, so `get_position` would go stale exactly when the gripper sits
> idle — which is most of the time. So `compute()` runs every tick: it records the
> snapshot, returns the command during the hold, and `None` after — which is precisely
> how a task declines arbitration for a tick (`tick_loop.py:299-301`).
>
> **One consequence, accepted deliberately:** `remove_hardware()` refuses to remove
> hardware whose joints an *active* task claims (`coordinator.py:419-427`), so a
> registered gripper task blocks it. In practice tasks are removed before hardware —
> `_setup_from_config`'s rollback already does exactly that.

**R22.** It uses `ControlMode.SERVO_POSITION`, matching `servo_task` and
`trajectory_task` so that an integrated gripper never disagrees on mode with the arm tasks
sharing its component (`tick_loop.py:386-397`). Priority is uncontested by R17.

### 4.5 Coordinator and `ConnectedHardware`

**R23.** `@rpc set_gripper_position` and `get_gripper_position` are **removed**
(`ControlCoordinator`, `coordinator.py:901-931`). The capability moves to
`task_invoke("arm_gripper", ...)` — `set_normalized` for the agent skills (R26), which is
the scale they already think in. Only the shortcut route dies.

**R24.** *Moved to PR 2 — branch `jhengyi/gripper_adapter_pr2` (§8).* **Confirmed as
part of R9's package**: the fourth kind ships with its type and its
registry, mirroring how `drive_trains` and `whole_body` are wired.

**R25.** `ConnectedHardware` builds one ordered array from `all_joints` and makes one call.
Its gripper branches are deleted — in `read_state`, `write_command`, and
`_initialize_last_commanded` (`hardware_interface.py:125-137, 192-203, 213-221`). It
performs **no** unit conversion, and wraps either adapter kind unchanged.

### 4.6 `ManipulationModule`

**R26.** **No new ports.** The skill surface speaks **one scale, `0.0`–`1.0`**, closed to
open — the polarity R19 fixes for every scalar API.

| Skill | Becomes | Why |
|---|---|---|
| `set_gripper(position)` | `task_invoke("arm_gripper", "set_normalized", {"values": [position]})` | `0.0`–`1.0`, not metres |
| `get_gripper()` | `task_invoke("arm_gripper", "get_normalized")` | so `set_gripper(get_gripper())` is a no-op |
| `open_gripper()` | `set_sweep(1.0)` | R19a — see below |
| `close_gripper()` | `set_sweep(0.0)` | R19a — see below |

**The module retains no gripper state** — no cached range or position, no gripper
handling in the joint-state callback. `get_gripper()` is a thin call into the task,
which owns the range and normalizes (`get_normalized`, R16). Long-horizon skills call
the module's own gripper skills, never `task_invoke` directly. `get_gripper()`'s
docstring MUST be corrected: it claimed metres while returning `0.85`, and both halves
of that were wrong. *(Revised on review from a stream-side cache.)*

All four hard-coded `0.85` (`manipulation_module.py:1686, 1914`;
`pick_and_place_module.py:518, 627`) and the `0.0` closes become sweep calls, so **no
endpoint value survives above the task** — which is the point of R15.

> **`open`/`close` use `set_sweep`, not `set_normalized`.** On a single jaw the two are
> identical. On a multi-joint hand they are not: `set_normalized([0,0,0,…])` drives every
> joint to its limit — R19a's fist — while `set_sweep(0.0)` goes to the vendor's grasp
> pose. Nothing changes today; it is correct when the H100 arrives.
>
> **Normalized, not native, and not in tension with grasp work.** The skills sit above
> the task, and nothing above the task needs a vendor range (R14). The consumer with a
> physical target reads the task's `get_state` limits and commands `set_position` in
> native units (R18). Two consumers, two doors. *(A module-level `get_gripper_limits()`
> was briefly added, then removed on review — the module holds no range.)*

**R27.** The three `@skill` methods keep their synchronous `bool`, reporting **command
acceptance** — identical to today, where the `bool` means "the SDK accepted it", not "the
gripper arrived". `pick_and_place_module`'s three `time.sleep()` calls are **retained**;
replacing them requires grasp detection, which is out of scope here.

### 4.7 Configuration

Two changes to how a blueprint declares a gripper — and one thing to deliberately leave
alone.

**R28.** A `HardwareComponent` stores **one ordered joint list** plus a count of how many
trailing entries are the gripper. The two views are **derived, never stored**:

```python
# PROPOSED
all_joints:  list[JointName]      # THE stored list — this IS the adapter's array
gripper_dof: int = 0              # how many trailing entries are the gripper

arm_joints     = all_joints[: len(all_joints) - gripper_dof]     # derived
gripper_joints = all_joints[len(all_joints) - gripper_dof :]     # derived
```

The split MUST be computed as `len(all_joints) - gripper_dof`, **never** by negative
slicing: `-0 == 0` in Python, so `all_joints[:-gripper_dof]` returns an empty list when
`gripper_dof == 0`, silently reporting that a gripper-less arm has no arm joints and that
every joint is a gripper joint.

*Why a count, not two lists.* The adapter must know where the gripper starts, so the number
is required either way. Two lists make it exist twice — implicitly as `len(gripper_joints)`,
explicitly as the adapter's argument — and two copies can drift, silently mis-splitting the
array. One list also makes the ordering a fact rather than a convention.

> **The model already contains PR 2's shape.** A standalone gripper is
> `gripper_dof == len(all_joints)`: the split lands at `0`, `arm_joints` is empty, and
> `gripper_joints` is everything. Integrated and standalone are the same arithmetic with
> different numbers — no special case anywhere above the adapter.

**R28a.** The `joints` field is **deleted**, not redefined. Keeping the name while changing
its meaning would let `hw.joints` still resolve and quietly return more joints than before.
Deleting it makes every call site raise `AttributeError` until updated deliberately.

The sites group into **three different correct answers** (the mechanical remainder —
`openyam`, `openarm` and `a750` configs, six `unitree` blueprints,
`control/blueprints/mobile.py` — takes the same answer as its group; §6 lists them):

| Site | Becomes | Why |
|---|---|---|
| 13 blueprint claims (`hw.joints`) | `arm_joints` | arm tasks must not claim the gripper (R17) |
| `common/blueprints.py:79` | `arm_joints` | the shared `trajectory_task()` helper — governs every arm blueprint |
| `hardware_interface.py:61` | `arm_joints` | `ConnectedHardware`, may have a gripper |
| `hardware_interface.py:273, 361` | `all_joints` | `ConnectedTwistBase` / `ConnectedWholeBody` — never have grippers |
| `coordinator.py:289, 301, 313` | **per adapter kind** | manipulator wants arm DOF; base and whole-body want all |

> `common/blueprints.py:79` carries a decision, not a rename. If it became `all_joints`,
> every trajectory task would claim the gripper alongside `GripperControlTask` — two owners
> on one joint, contradicting R17.

**R29.** `make_gripper_joints()` takes a joint count. It is hardcoded to exactly one name
today (`components.py:106-115`), which is the single-joint ceiling in §3.2:

```python
CURRENT   make_gripper_joints("arm")      → ["arm/gripper"]   # hardcoded to one
PROPOSED  make_gripper_joints("hand", 6)  → ["hand/gripper1" … "hand/gripper6"]
```

**R30.** A blueprint exposing gripper control adds a `gripper` task claiming
`hw.gripper_joints`:

```python
# PROPOSED — single-joint jaw: reference pose derives from the closed limit (R19a)
TaskConfig(name="arm_gripper", type="gripper", joint_names=_arm_hw.gripper_joints)

# PROPOSED — multi-joint hand: reference pose is MANDATORY (R19a/R19b), from the
# vendor's documented grasps; `hand` scopes the VR trigger (R16/R17)
TaskConfig(
    name="hand_gripper",
    type="gripper",
    joint_names=_hand_hw.gripper_joints,
    params={
        "reference_pose": H100_GRASPS["two_finger_pinch"],   # joint vector, native units
        "hand": "right",                                     # which trigger, if any
    },
)
```

The name is **`{hardware_id}_gripper`**, defaulting to `arm_gripper`. It is not cosmetic:
it is what `task_invoke("arm_gripper", …)` addresses (R23, R26), and it must be unique per
arm on a multi-arm rig.

> **Why hardware-id first, unlike `traj_arm` / `eef_twist_arm`.** `gripper_arm` parses as
> a noun phrase — "a gripper arm" — which `traj_arm` cannot, and it degrades on real
> hardware ids (`gripper_xarm_arm`). `arm_gripper` mirrors the joint it claims:
> `xarm_arm_gripper` claims `xarm_arm/gripper`, immediately. Nothing hardcodes the
> string — `ManipulationModule` already resolves the hardware id
> (`_get_gripper_hardware_id()`), so it derives `f"{hw_id}_gripper"`, and a multi-arm rig
> addresses the right task with no extra config.

Two things worth knowing about the blueprints as they stand:

- **`keyboard_teleop_a1z` and `keyboard_teleop_piper` already ship this pattern**,
  hand-rolled as servo tasks claiming only `["arm/gripper"]`
  (`a1z/blueprints/teleop.py:45-51`, `piper/blueprints/teleop.py:60-66`). R30 replaces
  both with the `gripper` task type; they migrate together, because the keyboard's move
  to `gripper_command` removes the publisher both currently read.
- **`coordinator_teleop_dual` has no gripper path at all** (`common/mixed.py:75-88`):
  `gripper=True` on both arms, but neither `teleop_ik_task` is given gripper config and no
  gripper task exists. Its grippers are unreachable today — a pre-existing defect this
  refactor surfaces, worth filing separately (with `keyboard_teleop_a750`, R13).

## 5. Untouched

`servo_task`, `trajectory_task`, and `ManipulationModule`'s ports.

`pick_and_place`'s **skill signatures** are untouched, so every caller of it keeps
working. Two lines *inside* it are not: its hard-coded `0.85` open and `0.0` close become
sweep calls (R26), because no endpoint value may survive above the task.

Every site reading `component.joints` **is** modified (R28a) — each raises
`AttributeError` until updated. That is deliberate: it is what makes the rename
impossible to half-finish.

`teleop_task` and `eef_twist_task` **are** modified (R17) — they lose their gripper claims
and their gripper config. They forward nothing; their inputs reach the gripper task
directly (R16).

## 6. Blast radius

| Area | Files | Nature |
|---|---|---|
| **New task** | `control/tasks/gripper_task/` | `GripperControlTask` + `_registry.py` |
| Adapter protocol | `manipulators/spec.py` | Delete 2 methods; add `get_gripper_dof()` |
| Adapters | `xarm`, `piper`, `mock`, `sim`, `openarm`, `galaxea_a1z`, `a750` | Unified array, `gripper_dof`, range via `get_limits()` |
| Hardware wrapper | `hardware_interface.py` | One array; delete gripper branch + normalization; 3 `component.joints` reads |
| Components | `components.py` | store `all_joints` + `gripper_dof`, delete `joints`, add 2 derived properties; delete 2 endpoint fields; widen `make_gripper_joints` |
| Tasks | `teleop_task`, `eef_twist_task`, `cartesian_ik_task` | Drop gripper claims + gripper config fields; delete 2 helpers (R17, R17a); `eef_twist/_registry.py` drops its `gripper_command` binding |
| Coordinator | `coordinator.py` | Delete 2 RPCs; 3 `component.joints` reads |
| Manipulation | `manipulation_module.py`, `pick_and_place_module.py` | 4 method bodies, 1 docstring, and the status lines that claim metres (R26) |
| Teleop | `teleop/keyboard/keyboard_teleop_module.py` | `[` / `]` publish `gripper_command` (Bool) instead of `joint_command` (R16) |
| Blueprints | `xarm`, `a1z`, `piper` teleop + 13 claim sites | Add the gripper task; `hw.joints` → `hw.arm_joints`; delete `XARM_GRIPPER_PARAMS` (R17a) |
| Mechanical rename only | `openyam`, `openarm`, `a750` configs; `unitree` g1 ×2, go2 ×4; `control/blueprints/mobile.py` | `joints=` → `all_joints=` (R28a), no behaviour change — all in part 1.1 |
| Simulation | `simulation/engines/mujoco_shm.py`, `mujoco_sim_module.py` | Publish the MJCF gripper range over SHM (R13a) |
| Message docs | `msgs/sensor_msgs/JointState.py` | Note gripper joint units |

## 7. Delivery plan

**This PR is the spec — documentation only, no code.** Split so each PR is **separately
verifiable**: a PR that cannot be proven on hardware we own does not ship.

| Step | Delivers | Requirements | Roughly |
|---|---|---|---|
| **0 — this PR** | The spec. Agreement on the API before anything is written | — | docs only |
| **1 — this spec** ✅ | The joint model, units, `GripperControlTask`, RPC removal. **Four parts, §7.1** | everything except R9, R24 | **delivered**; verified on xArm6 |
| **2 — third-party support** | `GripperAdapter` (minimal, fourth kind) + `HardwareType.GRIPPER` + registry + the parity test — **shape confirmed; on branch `jhengyi/gripper_adapter_pr2`** | R9, R24 | mock + parity test now; the H100 validates |
| **3 — H100 integration** | Device folder (adapter + driver + transport); `by_task_name` gripper routing (R16); multi-joint `get_gripper()` skill surface (R26) | — | the H100 itself; may merge with step 2 |
| **4 — transport layering** | Routing gripper bytes through an arm's bus when mounted | — | mounted H100 |

### 7.1 Step 1 ships in four parts

A single PR here would change five independent layers at once — joint model, adapter
protocol, wire units, control path, consumer API. Command a gripper on the bench and
watch it land wrong: five layers could have mangled the value, and there is no way to
bisect. That is §3.5's failure mode reproduced in the delivery. So step 1 ships as four
parts, each proving **one claim**, each with a definition of done that needs no bench —
hardware proves the part; tests gate it.

> **The rule that keeps this from becoming legacy preservation.** A part MAY leave old
> code standing for one more part. It MUST NOT add new code that the final design
> deletes. No shims, no `joints_v2`, no compatibility layer, no feature flag. The entire
> transitional cost is **about nine lines**, named in 1.2, all deleted by 1.4.

| Part | Claim it proves | Requirements |
|---|---|---|
| **1.1** — the joint model | *Nothing changed.* | R28, R28a, R29 |
| **1.2** — one array, one unit | The value the task emits reaches the SDK unchanged | R4, R4a, R5–R8, R11, R12, R13, R13a, R15, R25 |
| **1.3** — the task exists | One task drives one real gripper, converting once | R14, R14a, R16, R18–R22 |
| **1.4** — one owner everywhere | No joint has two claimants; the shortcut route is gone | R2, R17, R17a, R23, R26, R27, R30 |

---

#### 1.1 — The joint model

**Claim: nothing changed.**

`HardwareComponent` stores one ordered array plus a count (R28); `arm_joints` and
`gripper_joints` become derived views; `make_gripper_joints()` takes a count (R29). Every
construction and read site moves with it — R28a's table plus §6's mechanical-rename row.

**Nothing else is touched.** Adapters, units, tasks, RPCs, blueprint behaviour:
identical. `ConnectedHardware` still normalizes exactly as it does today; the xArm still
loses its 15%.

*Done when (no hardware):*
- the full fast suite passes with **zero behavioural test edits** — only tests that
  literally assert the `joints` / `gripper_joints` fields change;
- `read_gripper_position` / `write_gripper_position` / `_normalized_to_physical` are
  provably untouched (grep);
- `mypy` is clean.

*Delivered.* 411 passed / 1 skipped, identical to the pre-change baseline; no diff line
touches the gripper or conversion path; mypy error count unchanged. The §3.5 chain was
re-measured and still produced **722.5** — which is the point: a part claiming to change
nothing must leave the bug exactly where it was. 28 files, of which 24 averaged three
lines each: wide and shallow, as intended.

---

#### 1.2 — One array, one unit

**Claim: the value the task emits is the value that reaches the vendor SDK.**

The adapter protocol collapses to one array (R4, R4a): gripper methods deleted (R5),
`get_gripper_dof()` added (R8), `get_limits()` covering all joints as the one
authoritative range declaration (R13, R13a). Units settle in the same part because they
cannot settle separately: the moment the gripper joins the array, whatever converts it
must be decided. `ConnectedHardware` loses its gripper branch and both conversion
helpers (R15, R25); xArm drops the mm conversion and declares `(0.0, 850.0)`; blueprint
endpoints become native.

*Done when (no hardware):*
- `ConnectedHardware.write_command` on a 6+1-joint mock issues **exactly one**
  `write_joint_positions` call carrying 6 arm radians + 1 trailing native gripper value,
  and `read_state` round-trips the trailing entry unconverted;
- the **§3.5 regression test**: the xArm adapter against a stubbed SDK receives
  **850.0** — not 722.5 — for fully open (`piper/test_teleop_gripper_mapping.py` is
  rewritten to pin the same for Piper's `0.08`);
- a conformance test over every registered adapter checks the R4/R4a/R13 lengths;
- sim: the `grp[2:4]` range round-trips over SHM (R13a).

*Delivered.* 435 passed. **Measured on a real xArm6:** commanded `850`, gripper reached
`842` — 0.9% short, the mechanical hard stop — against the 15% the double conversion
caused. The read is the cleaner evidence: the gripper surfaced as `837.0` before any
command was issued, where the old path would have reported `0.837`.

*Not measured:* Piper and a1z. Both encode metres into vendor units inside the adapter
(Piper ×1e6 into stroke units, a1z into a 0–1 fraction), which the xArm's dimensionless
passthrough never exercises. **Piper's 12.5% recovery (§7.2) is unverified.**

*Transitional cost (~9 lines, all deleted in 1.4):* the two gripper RPCs are rewired
through the array rather than deleted, because nothing replaces them until 1.4; the
hard-coded skill endpoints become native for the same reason; the teleop/eef-twist
blueprint endpoint params change value before they disappear.

*Known imprecision, gone in 1.3:* keyboard `[`/`]` still publishes `1.0` on
`joint_command` and relies on adapter clamping until it moves to `gripper_command`.

---

#### 1.3 — The task exists

**Claim: one task drives one real gripper, and converts exactly once.**

`GripperControlTask` arrives complete — streams and commands (R16), two modes (R18, R19,
R19a, R19b), non-blocking (R20), hold semantics (R21, R21a), `SERVO_POSITION` (R22),
limits acquired per R14a.

It proves itself on the **two keyboard blueprints that already ship this pattern**
hand-rolled as a servo task claiming only `["arm/gripper"]` —
`keyboard_teleop_a1z` (`a1z/blueprints/teleop.py:45-51`) and `keyboard_teleop_piper`
(`piper/blueprints/teleop.py:60-66`). Swapping each for the `gripper` type is a 1:1
replacement on a blueprint with no competing gripper claimant. R30 applies to these two
only.

> **Both migrate together, and this is forced.** `KeyboardTeleopModule` moves from
> `joint_command` to `gripper_command` in this part (R16), and *both* servo-gripper tasks
> read `joint_command`. Migrating one would leave the other's gripper with no publisher
> for a whole part. Two devices also give the claim two independent hardware witnesses
> in different units — a1z metres and Piper metres, against xArm's dimensionless scale
> proven in 1.2.

Everything else keeps working as in 1.2 — the other blueprints' arm tasks still own their
grippers, and the RPCs still exist.

*Done when (no hardware):*
- task unit tests pin: every command returns immediately (R20); `is_active()` always
  True with `get_position` fresh while idle (R21a); the hold emits, then `None`, and
  never stops on target-reached (R21); `set_normalized([0.0])` lands on the closed
  limit (R19 polarity); `set_sweep` refuses on multi-joint without a reference pose
  (R19b); the trigger inverts and respects the engagement gate (R17);
- R14a validation raises on joint names not matching one component's `gripper_joints`;
- a blueprint test: `keyboard_teleop_a1z` **and** `keyboard_teleop_piper` each carry a
  `type="gripper"` task named `arm_gripper` claiming exactly `["arm/gripper"]`, no
  `type="servo"` task claims a gripper joint anywhere, and `[` / `]` drive the mock
  adapter end-to-end through the coordinator.

*Delivered.* 484 passed; 41 new tests. **Proven on a real xArm6** rather than on the two
blueprints named above, whose devices were unavailable: the bench script gained a
`--via-task` mode that builds the real task against a live adapter. It resolved
`(0.0, 850.0)` from the adapter unprompted, and `task.get_position()` matched the measured
joint state exactly at both ends of the stroke — `842` and `-1` — which is R16's
one-snapshot guarantee holding on hardware.

---

#### 1.4 — One owner everywhere

**Claim: no gripper joint has two claimants, and the shortcut route is gone.**

`teleop_task` and `eef_twist_task` drop their gripper claims, config and helpers (R17,
R17a). The two RPCs are deleted (R23). The remaining blueprints get the gripper task
(R30). `ManipulationModule` is rewired (R26, R27). Every transitional line from 1.2 is
removed.

*Done when (no hardware):*
- a single-owner audit test: across every built-in blueprint, gripper joints appear
  only in the claims of `type="gripper"` tasks;
- `claim_with_gripper`, `append_gripper_position`, `XARM_GRIPPER_PARAMS`, both RPCs,
  and every 1.2 transitional line are gone (grep);
- skill round-trip on mock: `set_gripper(x)` then `get_gripper()` ≈ `x`;
  `open_gripper()` / `close_gripper()` land on the sweep endpoints with R19 polarity.

*Delivered.* 1253 passed; runtime ownership confirmed across all six gripper-bearing
blueprints — one owner each, always the gripper task.

**Proven on a real xArm6** via `keyboard-teleop-xarm6`: the arm jogs and `[` / `]` operate
the gripper. Since the RPC shortcut no longer exists, the gripper cannot have moved any
other way. The audit itself was tested by reintroducing a runtime-widened claim: it caught
it, and the static half did not.

*Not measured:* the VR trigger and its engagement gate (needs a Quest), and preemption
during a grasp.

### 7.2 What step 1 found that this plan did not anticipate

**1. The single-owner audit has to instantiate tasks.** `claim_with_gripper` widened the
claim inside `claim()`, at runtime. A scan of `TaskConfig.joint_names` — the obvious
check, and the one first written — saw an arm task claiming only arm joints while the tick
loop saw it claiming the gripper too. **The double ownership §3.4 describes was invisible
to static inspection**, and the audit only bites because it builds each task and reads its
claim. Where a task cannot be built (its model assets are absent), the audit instead
proves that task structurally incapable of owning a gripper: it declares none of those
joints and does not override `claim()` at all.

**2. §3.5 had a second instance, in Piper.** The blueprints declared
`gripper_open_position=0.07` while the adapter's real stroke is `0.08` — the same disease
as the xArm's, an endpoint written down somewhere that could disagree with the device,
costing **12.5% of travel**. It was not visible from the xArm measurement because the two
adapters fail differently: xArm's was a multiplication applied twice, Piper's was a
smaller number declared in the wrong place. R13 removes both by making the adapter the
only thing that declares a range. **Unverified on hardware.**

**3. The standalone gripper shape already works — R9 and R24 may be unnecessary.**
R28's split is `len(all_joints) - gripper_dof`; a gripper that is its own device sets
`gripper_dof == len(all_joints)`, so the split lands at `0`, `arm_joints` is empty, and
every joint is a gripper joint. The same arithmetic, different numbers.

Demonstrated end to end through the real coordinator with a 6-DOF standalone hand and
**no new code**: the manipulator registry built its adapter with `dof=0` and
`gripper_dof=6`, `add_hardware` routed all six joints, the gripper task resolved its own
limits, and both control modes worked — `set_position` per joint, and `set_sweep` onto a
configured vendor grasp pose.

> **What this changes: it de-risks PR 2; it does not decide it.** The plumbing —
> model, coordinator, task, wrapper, units — accepts the standalone shape with no new
> code, so PR 2's design starts from a working path rather than a guess. The decisions
> PR 2 still owns, informed by this finding:
>
> All four were **resolved on 2026-08-07**, after checking the design-meeting
> transcript and the PR review threads: R9 ships as a separate minimal protocol with a
> signature-parity test (the meeting's "fourth protocol, keep separate initially, merge
> later if similar"); R8's carve-out stands; R24 ships as part of R9's package; R16's
> routing change lands in step 3. The finding below remains what it was — evidence that
> the plumbing is ready, not a decision.
>
> **This was a mock.** It proves the plumbing accepts the shape. It does not prove any
> real device fits, and the H100 — the first third-party multi-joint gripper — is what
> PR 2 is designed for.

### 7.3 Carried forward

| | |
|---|---|
| Piper and a1z bench runs | 1.2's unit claim on metric adapters; Piper's 12.5% recovery |
| VR trigger on hardware | 1.4's engagement gate and trigger polarity |
| Preemption during a grasp | 1.4's arbitration claim |
| a750 | converted blind; no device to verify against (R13) |
| multi-joint `get_gripper()` | skill surface for a hand is undefined (greptile's review point); defined in step 3 |
| R10 soft-body exclusion | stands as signed; answer mustafab0's review thread rather than reopen |
| `keyboard_teleop_a750`, `coordinator_teleop_dual` | pre-existing: grippers unreachable, surfaced not caused by this work (R13, R30) |

Nothing in step 1 is waiting on a decision: every question raised against this spec was
resolved into the requirements above (R4a, R13, R13a, R14a, R16, R17, R21a, R26, R28,
R30).
## 8. PR 2 — split out, per review

This PR is **step 1 only**. The `GripperAdapter` protocol, `HardwareType.GRIPPER`,
the gripper registry, the standalone blueprints, the xArm-gripper hardware witness,
the bench scripts, and the full §8 proposal live on branch
**`jhengyi/gripper_adapter_pr2`**. The decisions recorded in R8/R9/R24 stand; they are
implemented and reviewed there.
