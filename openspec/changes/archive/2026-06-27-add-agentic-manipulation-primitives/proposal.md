## Why

DimOS has manipulation planning and control primitives, but the current skill-facing surface is either too low-level for direct agent use or tied to existing pick-and-place semantics. We need a universal agentic manipulation module that exposes a small, testable primitive skill surface before building higher-level manipulation benchmark episodes or MCP tool profiles.

## What Changes

- Add a universal `AgenticManipulationModule` that wraps existing manipulation/control RPCs through the DimOS Spec injection pattern.
- Expose an initial joint-space and gripper primitive skill surface suitable for direct agent use: robot state, joint motion, open gripper, and close gripper.
- Add simulator-free unit tests for the facade behavior and skill surface.
- Add a script-hosted Robosuite layer-2 API demo that constructs the full stack dynamically and calls the new module API directly.
- Keep Robosuite, benchmark runtime clients, MCP filtering, LLM agent execution, and Cartesian planning outside the universal module scope.

## Capabilities

### New Capabilities
- `agentic-manipulation-primitives`: Universal agent-facing manipulation primitive skills and a script-hosted Robosuite API validation path.

### Modified Capabilities

## Impact

- Affected code areas:
  - `dimos/manipulation/` for the new module, RPC Spec, and simulator-free tests.
  - `scripts/benchmarks/` for the Robosuite layer-2 demo.
  - runtime sidecar/manipulation docs for the manual validation command and scope boundaries.
- The new module must not import Robosuite, runtime sidecar clients, or benchmark-specific APIs.
- The Robosuite validation remains script-only and manual/heavy; default unit tests remain lightweight.
- Future MCP/agent work can expose this module without changing its primitive behavior.
