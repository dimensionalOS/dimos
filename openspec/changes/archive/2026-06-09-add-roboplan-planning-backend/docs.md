## User-Facing Docs

- Update `docs/capabilities/manipulation/readme.md` to describe RoboPlan as an optional manipulation planning backend alongside Drake. Include when a user would choose RoboPlan, how backend selection fits the existing manipulation module/blueprint configuration surface, and that execution still routes through existing coordinator trajectory tasks.
- Update `dimos/manipulation/planning/README.md` to add:
  - `planning_backend="roboplan"` configuration examples.
  - Required RoboPlan robot planning assets: URDF, SRDF if required by the selected RoboPlan APIs, package paths, planning group or active joint names, base frame, end-effector frame, joint order, and joint limits.
  - RoboPlan-specific `planning_backend_options` examples for RRT, IK, retiming, and scene projection.
  - Supported and unsupported feature table for joint planning, pose planning/IK, collision validation, primitive obstacles, mesh/pointcloud layers, attached objects, FK/Jacobian/distance queries, and Viser-owned path rendering.
  - Troubleshooting for missing RoboPlan modules or native bindings, including a verification command such as `python -c "import roboplan.core, roboplan.rrt"` and optional `roboplan.toppra` when TOPPRA retiming is selected.
- Update any manipulation blueprint docs that introduce a RoboPlan demo or configuration. If no new runnable blueprint is added, document RoboPlan selection as a module/blueprint configuration option rather than a new top-level CLI command.
- Add or update a short hardware-safety note in the manipulation docs: RoboPlan changes planning only; real robot execution remains mediated by ControlCoordinator trajectory tasks and should be validated in mock/sim before supervised hardware use.

## Contributor Docs

- Update `docs/development/dimos_run.md` only if a new RoboPlan demo blueprint is exported for `dimos run`; include the blueprint name, required extra/dependencies, and mock/sim-first validation guidance.
- Update `docs/development/testing.md` if the implementation adds RoboPlan-specific test markers, optional dependency test skips, or a new command for RoboPlan integration tests.
- If packaging changes add RoboPlan to an optional dependency extra, document the dependency channel and platform caveats where contributor dependency guidance already lives. Mention that default Drake-backed manipulation tests should not require RoboPlan unless a test explicitly selects `planning_backend="roboplan"`.

## Coding-Agent Docs

- Update `docs/coding-agents/index.md` or a manipulation-specific coding-agent guide if one exists to note:
  - Use the active backend Protocols (`PlanningBackend`, `SceneFacade`, `PlannerFacade`) rather than direct Drake `WorldSpec` access for new backend-neutral manipulation code.
  - Keep RoboPlan imports lazy inside the RoboPlan backend path.
  - Set RoboPlan planner/IK/TOPPRA options explicitly instead of relying on upstream defaults.
  - Do not edit `dimos/robot/all_blueprints.py` manually; regenerate it with the blueprint generation test if new RoboPlan blueprints are exported.
- No repository-level `AGENTS.md` update is required unless implementation changes add new persistent coding-agent rules beyond the existing generated-registry and manipulation guidance.

## Doc Validation

- Run doc link validation after documentation changes:

```bash
uv run doclinks
```

- Run markdown command validation for changed docs that contain executable examples, especially manipulation docs with code blocks:

```bash
uv run md-babel-py run docs/capabilities/manipulation/readme.md
uv run md-babel-py run dimos/manipulation/planning/README.md
```

- If new blueprint exports are documented, regenerate and validate the generated blueprint registry:

```bash
pytest dimos/robot/test_all_blueprints_generation.py
```

- If docs include generated diagrams or architecture figures, run the repository's diagram generation command for the touched diagram source. No diagram update is required by this docs plan unless implementation adds one.

## No Docs Needed

Documentation changes are needed. RoboPlan introduces a user-selectable planning backend with optional native dependencies, backend-specific robot asset requirements, safety/QA expectations, and contributor-facing testing/packaging implications. Without docs, users would not know how to select or configure RoboPlan safely, and contributors could accidentally make RoboPlan a hard dependency for default manipulation stacks.
