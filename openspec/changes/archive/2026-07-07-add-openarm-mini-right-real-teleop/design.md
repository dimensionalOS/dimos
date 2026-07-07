## Context

OpenArm Mini teleop currently has a generic `TeleopModule` path that emits OpenArm follower joint commands, a bimanual production-style OpenArm follower blueprint, and a left-side visualization-only Viser bring-up path. The next bring-up target is the physical right OpenArm follower. The user wants the blueprint to behave like existing xArm/Piper blueprints: no hardware connection setting means mock follower hardware, and an explicit connection setting opts into real follower hardware.

The right-arm path should use the standard manipulation stack for visualization rather than the custom OpenArm Mini Viser renderer. That means `ControlCoordinator` publishes `coordinator_joint_state`, `ManipulationModule` consumes it, and ManipulationModule's Viser backend renders the follower-observed state.

Current constraints:
- The real leader is always a physical OpenArm Mini right leader using Feetech serial and right-side calibration.
- The real follower is the right OpenArm arm on CAN, opt-in through global `--can-port`.
- Without `--can-port`, the follower side should be mock hardware but still exercise `ControlCoordinator` and ManipulationModule Viser.
- `ManipulationModule` expects coordinator joint state names in global form: `{robot_name}/{local_joint_name}`.
- OpenArm model configs expose local names such as `openarm_right_joint1` through `openarm_right_joint7`.

## Goals / Non-Goals

**Goals:**
- Add a right-arm blueprint, likely `openarm-mini-right-teleop-viser`, that connects real OpenArm Mini right leader teleop to `ControlCoordinator` and `ManipulationModule` Viser.
- Default the right leader port in this blueprint to `/dev/ttyACM0` while keeping CLI overrides available.
- Use global `--can-port` as real follower opt-in; use mock follower hardware when it is absent.
- Use global joint names through the coordinator for this blueprint so ManipulationModule can consume follower state directly.
- Remove the custom OpenArm Mini Viser visualizer from this right-arm integration path and rely on standard ManipulationModule Viser.
- Preserve existing left visualization and bimanual OpenArm Mini teleop behavior unless explicitly changed by this blueprint.

**Non-Goals:**
- No bimanual real-follower bring-up in this change.
- No gripper control or visualization.
- No automatic startup alignment gate beyond the existing calibration requirement and explicit real-hardware opt-in.
- No new global config field for OpenArm-specific CAN.
- No custom Viser renderer for the right-arm real integration path.

## Decisions

### Use one right-arm blueprint with mock/real follower selected by `--can-port`

The blueprint should always include `ControlCoordinator` and `ManipulationModule`. Its follower `HardwareComponent` should use `adapter_type="openarm"` and `address=global_config.can_port` when `global_config.can_port` is present, otherwise `adapter_type="mock"`.

Rationale:
- Matches existing xArm/Piper style.
- Keeps topology stable between preflight and real run.
- Exercises coordinator routing before physical hardware is connected.

Alternative considered: two explicit blueprints for mock and real follower. Rejected because the repository already uses connection-presence fallback patterns and the user preferred that approach.

### Use ManipulationModule's Viser, not the custom OpenArm Mini visualizer

The right-arm real integration path should render through `ManipulationModule.blueprint(robots=[openarm_model_config("right", name="right_arm")], visualization={"backend": "viser"})`.

Rationale:
- Reuses the standard planning/manipulation visualization stack.
- Shows follower-observed coordinator state instead of command-only preview.
- Avoids maintaining a parallel custom Viser renderer for real follower bring-up.

Alternative considered: fan out `joint_command` to a custom Viser module. Rejected because it bypasses coordinator/hardware feedback and duplicates Viser scene logic.

### Use global joint names for the right real-teleop blueprint

For this blueprint, the teleop command and coordinator hardware/task joint names should use `right_arm/openarm_right_joint1` through `right_arm/openarm_right_joint7`. The ManipulationModule robot remains named `right_arm` and uses local model joint names `openarm_right_joint1` through `openarm_right_joint7`.

Rationale:
- `ManipulationModule._on_joint_state()` consumes global joint names and strips the `right_arm/` prefix.
- Coordinator hardware interfaces can use arbitrary configured joint names and still send ordered positions to the OpenArm adapter.
- This avoids a separate name-translation module between coordinator and ManipulationModule.

Alternative considered: keep local names in coordinator and add a bridge to global names for ManipulationModule. Rejected as unnecessary extra topology for a blueprint-specific naming choice.

### Keep right-side leader defaults blueprint-local

The right real-teleop blueprint should construct `OpenArmMiniTeleopConfig(enabled_sides=("right",), port_right="/dev/ttyACM0")`. The global OpenArm Mini default can remain `/dev/ttyUSB0` for other contexts.

Rationale:
- Matches the user's actual right-leader device path without changing defaults for all OpenArm Mini uses.
- Keeps CLI override behavior available for other machines.

### Treat missing right calibration as a startup failure

The right real-teleop blueprint should require valid right-side calibration before connecting or publishing commands.

Rationale:
- The leader is real hardware and may drive a real follower.
- Default or inferred calibration is unsafe.

## Risks / Trade-offs

- **Global joint naming may affect adapter assumptions** → Test that coordinator hardware with namespaced joint names still orders positions correctly for the OpenArm adapter.
- **Mock fallback can look like a successful run** → Document that only `--can-port` opts into real follower hardware and that no `--can-port` is mock preflight.
- **ManipulationModule Viser shows follower state, not immediate leader command** → This is intentional; document the distinction as follower-observed visualization.
- **Real follower can move as soon as the operator supplies `--can-port`** → Keep `--can-port` as the explicit real-hardware opt-in and document startup alignment/preflight steps.
- **Current Viser-only custom module becomes redundant for this path** → Do not remove the left visualization-only blueprint unless tests/docs show it is unused by this change; this change only avoids using the custom renderer for the right real integration path.

## Migration Plan

1. Add or reuse helpers for right OpenArm hardware/model config with `right_arm` as the robot/hardware identity.
2. Add blueprint-local right OpenArm Mini teleop config defaults.
3. Build a right-arm hardware component that chooses mock or real OpenArm adapter based on `global_config.can_port`.
4. Wire `OpenArmMiniTeleopModule`, `ControlCoordinator`, and `ManipulationModule` through autoconnect.
5. Regenerate the blueprint registry and update docs/tests.

Rollback: remove the new right-arm blueprint, docs, registry entry, and tests. Existing OpenArm Mini teleop and left visualization paths should remain unaffected.

## Open Questions

None. Resolved decisions: right side only, `/dev/ttyACM0` leader default, `--can-port` real-follower opt-in, mock follower fallback, missing calibration fails startup, ManipulationModule Viser replaces the custom renderer for this path, and global joint names are used for coordinator/manipulation compatibility.
