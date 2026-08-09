## User-Facing Docs

- Update `docs/usage/modules.md` with the shared typed-bus pattern: several connection outputs may feed one coordinator input when stream name and exact type match. Explain why resolved descriptions travel on a static port instead of generating module subclasses.
- Update `docs/usage/blueprints.md` with a two-connection example using distinct `instance_name` values, shared control streams, one coordinator, and one manipulation model.
- Update `docs/usage/configuration.md` with per-instance connection configuration examples such as `--left.address`, `--right.address`, `LEFT__ADDRESS`, and `RIGHT__ADDRESS`. Remove hardware-specific `GlobalConfig` examples that the migrated connections now own.
- Add or revise a control capability guide under `docs/capabilities/` that explains canonical interface names, connection descriptions, command profiles, coordinator introspection, lifecycle states, omission policies, watchdog behavior, and recovery after a fault or emergency stop.
- Update manipulation documentation to describe one statically prepared robot model and planning groups. Remove multi-robot examples, robot-ID selectors, local/global joint mappings, and the singular end-effector assumption.
- Update platform guides and README files for xArm/gripper, G1, Piper/OpenArm, mobile bases, simulation, and replay with their connection instance configuration, supported command profile, canonical units, and hardware bring-up checklist.
- Update skill/MCP examples whose public signatures lose `robot_name`, `robot_id`, `hardware_id`, or gripper hardware/range parameters.
- Clearly state that mixed protocol releases are unsupported: a coordinator and all participating connections must come from the same compatible release.

## Contributor Docs

- Add a contributor architecture section under `docs/development/` describing the ownership rule: connections own device I/O and safety; the coordinator owns interface arbitration and robot lifecycle; manipulation owns semantic planning for one prepared model.
- Document how to add a connection profile: declare fixed typed ports, define instance config, publish one immutable resolved description, publish complete state snapshots, validate command batches atomically, implement the watchdog/safe stop, and provide fake/simulation coverage.
- Document the scalar interface naming and SI-unit rules, description validators, sequence/freshness semantics, and the boundary between scalar control state and rich typed sensor streams.
- Document the transport restriction: current SHM/pSHM is acceptable for the single-writer command direction but not for the multiwriter connection-state direction.
- Update hardware testing guidance with the contract, simulation, and platform-owner validation matrix from `design.md`.
- Record the intentional removal policy so contributors delete old adapters, registry paths, robot selectors, and global config fields at each cutover instead of adding migrations.

## Coding-Agent Docs

- Update `docs/coding-agents/` architecture/navigation guidance so agents find control interfaces through connection descriptions rather than coordinator hardware registries.
- Add a concise control-refactor rule to the relevant scoped `AGENTS.md`: do not introduce robot IDs, coordinator-owned device adapters, local/global joint-name mappings, dynamic coordinator subclasses, or compatibility shims.
- Point coding agents to the connection-profile checklist and require them to distinguish OpenSpec capability specs from Python `Spec` Protocols.
- Update repository maps if paths, public coordinator APIs, connection modules, or generated blueprint files move during the stack.

## Doc Validation

Run the validation appropriate to each changed document:

```bash
uv run doclinks
uv run md-babel-py run <changed-executable-doc>
bin/gen-diagrams
uv run sphinx-build -W docs docs/_build/html
pytest dimos/robot/test_all_blueprints_generation.py
```

- Run `doclinks` for all Markdown link changes.
- Run `md-babel-py` only for documents containing executable examples and verify both one-connection and two-connection snippets.
- Run `bin/gen-diagrams` when a generated architecture diagram changes; do not hand-edit generated outputs.
- Build docs with warnings as errors using the repository-supported Sphinx invocation.
- Regenerate and verify the built-in blueprint registry whenever blueprint module variables, imports, or names change.
- Exercise documented CLI examples against a fake or simulation blueprint before merging PR7.

## No Docs Needed

Not applicable. This refactor changes public configuration, blueprint composition, lifecycle behavior, manipulation terminology, hardware bring-up, and contributor extension points; documentation is a required deliverable.
