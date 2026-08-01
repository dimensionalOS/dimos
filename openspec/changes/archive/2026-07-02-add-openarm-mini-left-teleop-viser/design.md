## Context

`add-openarm-mini-teleop` introduced the generic `TeleopModule` and an `OpenArmMiniTeleopAdapter` that emits OpenArm follower arm-joint `JointState` commands. The production OpenArm Mini blueprint routes those commands through `ControlCoordinator` and real OpenArm follower hardware.

For bring-up, operators need a lower-risk validation path: connect a real OpenArm Mini left leader, read its calibrated Feetech joint positions, and render the implied OpenArm left follower pose in Viser without connecting any OpenArm follower hardware or coordinator execution path.

Existing Viser manipulation support already handles URDF preparation and Viser joint-vector shape requirements through `ViserManipulationScene`. Existing OpenArm config helpers provide a left-arm model and joint names through `openarm_model_config("left")` and `openarm_joints("left")`.

## Goals / Non-Goals

**Goals:**

- Add a left-only OpenArm Mini teleop visualization workflow using a real OpenArm Mini left leader.
- Keep the follower side visualization-only: no `ControlCoordinator`, no `HardwareComponent`, no OpenArm follower adapter, and no mock follower hardware.
- Reuse the OpenArm Mini calibration and runtime transform path so Viser reflects the same leader-derived left OpenArm arm-joint command that production teleop would send.
- Render the left OpenArm follower model in Viser from incoming `JointState` commands.
- Register a runnable blueprint for the workflow.

**Non-Goals:**

- Do not add fake, replay, or sweep leader input in this change.
- Do not validate right-side OpenArm Mini teleop in this change.
- Do not connect OpenArm follower hardware, mock follower hardware, or `ControlCoordinator`.
- Do not add gripper visualization or gripper command semantics.
- Do not change production bimanual OpenArm Mini teleop behavior except where side selection is needed for reuse.

## Decisions

### Make OpenArm Mini adapter side selection explicit

The current adapter shape is bimanual by default. This change should add an explicit side-selection configuration for OpenArm Mini runtime use, with the visualization blueprint selecting only `left`.

The selected side controls:

- which calibration artifact is required
- which Feetech serial port is opened
- which bus is connected
- which side commands are emitted in the `JointState`

The default production behavior should remain bimanual so existing `openarm_mini_teleop_openarm` behavior is unchanged.

Alternative considered: create a separate left-only adapter class. This duplicates calibration, bus, and transform logic and increases the chance that visual validation diverges from production teleop.

### Use a visualization-only JointState-to-Viser module

The Viser validation path should add a small module that subscribes to the teleop module's `joint_command` output and renders a configured robot model in Viser. It should not instantiate `ControlCoordinator` or any OpenArm follower hardware component.

The visualizer module should normalize incoming `JointState` values by joint name into the OpenArm left model's joint order before rendering. This avoids relying on positional vector coincidence and keeps the module robust if command ordering changes. Missing required arm joints should be reported and skipped rather than silently rendering misleading poses.

Alternative considered: reuse `ManipulationModule` plus mock coordinator hardware. That still introduces coordinator and hardware-adapter concepts into a path whose purpose is to validate leader-to-visual mapping only. It also risks debugging coordinator state instead of OpenArm Mini calibration and Viser rendering.

### Reuse existing Viser scene helpers

The visualizer should reuse existing Viser runtime/scene helpers where practical, especially URDF preparation and joint configuration handling. The implementation must respect the existing Viser gotchas: pass prepared URDF paths, update configurations in the loaded URDF's actuated-joint order, and avoid unsupported visualization config keys because Viser config is strict.

Alternative considered: use raw `ViserUrdf` directly from the blueprint module. That is possible but would reimplement the URDF preparation and joint-order handling already centralized in the manipulation Viser scene code.

### Add one explicit left-only blueprint

The blueprint should be statically registered as a left-only validation path, likely named `openarm-mini-left-teleop-viser`.

It should wire:

```text
TeleopModule(OpenArmMiniTeleopAdapter(left only))
  joint_command -> OpenArm left JointState Viser visualizer
```

No side runtime flag is needed for v1. A right-side blueprint can be added later after left-side validation proves useful.

Alternative considered: one configurable blueprint with side selection. DimOS blueprint discovery is static, and existing OpenArm conventions already use explicit left/right blueprint names.

## Risks / Trade-offs

- Real leader hardware remains required → calibration/serial failures can prevent startup; mitigate with clear missing calibration and Feetech dependency errors from the existing adapter path.
- Direct Viser rendering bypasses `ControlCoordinator` → this validates leader transform and visualization, not coordinator routing or physical execution; production blueprint/hardware validation remains separate.
- Name/order mismatches can produce misleading visuals → normalize incoming `JointState` by name into model joint order and test missing-joint handling.
- Viser optional dependencies may be absent → fail with the existing Viser install hints and document the required extra.
- Left-only support touches the production adapter → keep bimanual as default and add tests that existing bimanual behavior remains available.

## Migration Plan

1. Add side-selection support to the OpenArm Mini adapter/config with bimanual default behavior preserved.
2. Add a visualization-only JointState-to-Viser module for OpenArm arm models.
3. Add the `openarm-mini-left-teleop-viser` blueprint and registry coverage.
4. Add tests for adapter side selection, visualizer joint-name normalization, and blueprint wiring.
5. Document the command and the fact that it requires real OpenArm Mini left leader hardware but no OpenArm follower hardware.

Rollback is straightforward: remove the new blueprint and visualizer module. The production OpenArm Mini teleop blueprint remains unchanged.

## Open Questions

- None for v1. Right-side and fake/replay visual validation are explicitly deferred.
