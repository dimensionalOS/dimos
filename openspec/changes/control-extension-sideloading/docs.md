## User-Facing Docs

Add a new external robot package guide under `docs/capabilities/` or another user-facing location chosen during implementation. The guide should explain the package anatomy and focus on how external packages register ControlCoordinator extension points.

Recommended guide outline:

```text
dimos_mydog/
├── adapters.py
├── blueprints.py        # calls register_extensions()
├── tasks/
│   └── gait.py          # lazy control task factory
├── skills.py            # optional
└── prompts.py           # optional
```

The guide should show all supported embodiments briefly:

- `HardwareType.MANIPULATOR` for an external arm.
- `HardwareType.BASE` for an external robot dog or mobile base.
- `HardwareType.WHOLE_BODY` for an external humanoid or whole-body robot.

Use a `BASE` / robot-dog style walkthrough as the primary example:

```python
from dimos.control.components import HardwareType
from dimos.control.extensions import register_hardware_adapter, register_control_task

from dimos_mydog.adapters import MyDogAdapter


def register_extensions() -> None:
    register_hardware_adapter(HardwareType.BASE, "mydog", MyDogAdapter)
    register_control_task("mydog_gait", "dimos_mydog.tasks.gait:create_task")


register_extensions()
```

Then show unchanged blueprint usage:

```python
HardwareComponent(hardware_type=HardwareType.BASE, adapter_type="mydog")
TaskConfig(type="mydog_gait")
```

The guide should explicitly state boundaries:

- This change handles external hardware adapter and control task registration.
- External blueprint discovery is separate.
- Package auto-discovery and entry points are future work.
- Direct imports from low-level registry singletons are internal/advanced and should not be the documented external package path.

Consider updating existing custom arm documentation to point external-package authors to the new guide instead of requiring files under `dimos/hardware/manipulators/`.

## Contributor Docs

No contributor-process documentation is required unless implementation discovers new testing or release steps. If duplicate registry behavior exposes built-in collisions, note the expected duplicate policy near relevant registry tests or developer docs.

## Coding-Agent Docs

No `AGENTS.md` or coding-agent documentation update is required for phase 1. The external robot package guide should be sufficient for future coding agents working on robot package examples.

## Doc Validation

Run documentation checks appropriate to the files changed during implementation. Expected commands include:

```bash
doclinks
md-babel-py run <changed-doc-path>
```

If the chosen docs do not contain executable snippets, record that no snippet execution is required.

## No Docs Needed

Documentation is required because this is a new public extension surface for external robot authors.
