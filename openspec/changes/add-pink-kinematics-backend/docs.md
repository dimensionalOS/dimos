## User-Facing Docs

- Update manipulation/planning user docs to list supported kinematics backends: `jacobian`, `drake_optimization`, and `pink`.
- Document that `pink` is a local differential IK backend, not a global IK solver.
- Document optional dependency installation for Pink and its QP solver backend, including the final supported command chosen during implementation.
- Add a manual QA section for selecting Pink on the simulation-safe xArm blueprint with CLI overrides:

```bash
uv sync --extra all --extra pink
uv run dimos --simulation run xarm-perception-sim \
  -o pickandplacemodule.kinematics_name=pink
uv run python -i -m dimos.manipulation.planning.examples.manipulation_client
```

Then in the client:

```python
robots()
joints()
ik_pose(0.45, 0.0, 0.25)
ik_pose(0.45, 0.0, 0.25, seed_joints=[0.0] * 7)
plan_pose(0.45, 0.0, 0.25)
preview()
```

Document that `ik_pose(...)` is a client convenience wrapper around the
`solve_ik(Pose, ...)` RPC and that `seed_joints` can initialize local IK from a
known joint configuration.

- Document comparison against the baseline Jacobian simulation blueprint:

```bash
uv run dimos --simulation run xarm-perception-sim
```

## Contributor Docs

- If backend selection or dependency setup is documented for contributors, update the relevant manipulation planning or testing documentation with:
  - how to run unit tests for kinematics factory/backend behavior;
  - how to use CLI `-o` module config overrides to select Pink on existing blueprints;
  - how to run manual simulation QA without executing hardware.

## Coding-Agent Docs

- No AGENTS.md update is required unless implementation uncovers a recurring coding-agent gotcha.
- If added, coding-agent guidance should emphasize that Pink is selected through `KinematicsSpec`/factory wiring and does not replace the customized Pinocchio control solver.

## Doc Validation

- Run the relevant documentation checks used by this repository for any changed docs.
- At minimum, run targeted tests that validate commands and generated registry state:

```bash
uv run pytest dimos/robot/test_all_blueprints_generation.py
uv run pytest dimos/manipulation/test_manipulation_unit.py dimos/manipulation/test_manipulation_module.py
```

- If docs include executable Python snippets, validate them with the repository's doc snippet tooling if available.

## No Docs Needed

Documentation is needed because this change introduces a new user-selectable backend, optional dependency requirements, limitations, and a manual QA command path.
