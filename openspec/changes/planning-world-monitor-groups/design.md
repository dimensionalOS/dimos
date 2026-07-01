## Context

Planning groups become useful when world backends can answer queries for a specific group. This layer should prove that configured groups map correctly onto backend model instances, joint order, FK target frames, and Jacobian columns.

## Goals / Non-Goals

**Goals:**
- Implement group FK/Jacobian support in Drake and RoboPlan worlds.
- Make WorldMonitor expose group-scoped state and query helpers.
- Validate group-local joint ordering and global/local joint-state mapping.
- Keep base pose/base link semantics safe by rejecting unsupported backend cases rather than silently producing wrong transforms.

**Non-Goals:**
- Do not migrate planner or IK algorithms yet.
- Do not expose new public module APIs yet.
- Do not include Viser UI changes.

## Decisions

- Robot-scoped FK/Jacobian compatibility wrappers may remain temporarily but must resolve through exactly one pose-targetable group or fail clearly.
- RoboPlan Jacobians must be projected into group-local joint order; returning a full backend Jacobian is not sufficient.
- Backends should reject unsupported base-pose/base-link combinations explicitly.

## Risks / Trade-offs

- Backend capabilities differ. Tests should pin the supported subset and error messages.
- Some existing tests may need compatibility shims until PR 3 migrates algorithms and module APIs.
