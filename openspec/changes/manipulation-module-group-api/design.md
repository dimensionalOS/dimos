## Context

The manipulation module is where planning groups become user-facing. It must expose explicit group APIs while retaining safe behavior for existing robot-scoped wrappers.

## Goals / Non-Goals

**Goals:**
- Support explicit group IDs for joint targets, pose targets, IK, previews, and robot info.
- Keep robot-scoped wrappers deterministic: no unique pose group means no successful robot-scoped pose action.
- Preserve safe return contracts for RPC-like public methods.
- Cover compatibility behavior in unit tests.

**Non-Goals:**
- Do not implement Viser panel behavior in this PR.
- Do not add new planner algorithms.
- Do not change control task behavior unless required by module API compilation.

## Decisions

- `get_ee_pose` returns `None` when no unique pose-targetable group exists.
- `plan_to_pose` returns `False` when no unique pose-targetable group exists.
- `inverse_kinematics_single` returns `IKResult(NO_SOLUTION, message=...)` when no unique pose-targetable group exists.
- Explicit group APIs are preferred for multi-group robots.

## Risks / Trade-offs

- This file has a large diff. Keep the PR focused on API behavior and tests; do not include UI state-machine changes.
- Some compatibility wrappers are intentionally conservative to avoid silently selecting the wrong group.
