# OpenSpec Workflow

DimOS uses OpenSpec as the checked-in planning layer for behavior changes. OpenSpec artifacts live under `openspec/` and should describe what the system is supposed to do, why it is changing, and how contributors or agents should validate the work.

## Terminology

Keep these two meanings separate:

- **OpenSpec capability spec**: Markdown requirements under `openspec/specs/<capability>/spec.md`. These describe observable behavior and acceptance scenarios.
- **DimOS Spec**: Python Protocol/RPC contracts in files like `dimos/navigation/navigation_spec.py` or `dimos/manipulation/control/arm_driver_spec.py`. These describe module interfaces for code wiring.

Use "OpenSpec capability spec" in prose when there is any chance of confusion.

## Schema

The project uses the `dimos-capability` schema configured in `openspec/config.yaml`.

The artifact flow is:

```text
proposal
  ├── specs
  ├── design
  └── docs
        └── tasks
```

| Artifact | Purpose |
|---|---|
| `proposal.md` | Intent, scope, affected DimOS surfaces, and capability impact. |
| `specs/<capability>/spec.md` | Behavior-first requirements and scenarios. |
| `design.md` | Module, stream, blueprint, skill/MCP, safety, and rollout decisions. |
| `docs.md` | Documentation impact and doc validation plan. |
| `tasks.md` | Implementation, docs, verification, and manual QA checklist. |

## When to create a change

Create an OpenSpec change when work changes observable behavior, public CLI/API/MCP behavior, robot behavior, hardware/simulation/replay workflows, docs that users rely on, or cross-module architecture.

Do not create a change for a purely mechanical refactor, typo fix, or internal cleanup unless it changes behavior or needs cross-session planning context.

## Writing specs

OpenSpec capability specs are behavior contracts, not implementation plans.

Good spec content:

- User- or developer-visible behavior.
- Public CLI/API/MCP tool behavior.
- Stream or message behavior that downstream modules rely on.
- Robot safety constraints and hardware/simulation/replay expectations.
- Scenarios that can be tested or manually verified.

Avoid in specs:

- Private class/function names.
- Generated-file mechanics.
- Library choices and wiring details.
- Step-by-step implementation tasks.

Put those details in `design.md` or `tasks.md`.

## Capability names

Prefer behavior-domain names over code names. Useful starting points:

- `module-system`
- `blueprint-composition`
- `cli-lifecycle`
- `agent-skills-mcp`
- `configuration`
- `navigation-stack`
- `manipulation-stack`
- `hardware-adapters`
- `simulation-replay`
- `documentation-system`

Add specs progressively as changes need them. Do not try to backfill the whole project at once.

## Validation

Use OpenSpec validation before implementation and before archiving:

```bash skip
openspec schema validate dimos-capability
openspec validate <change-name>
openspec templates --json
```

For documentation changes, also run the relevant doc checks from [Writing Docs](/docs/development/writing_docs.md):

```bash skip
md-babel-py run <doc>
```

When a change touches blueprint names, module-level blueprint variables, or module registry inputs, run:

```bash skip
pytest dimos/robot/test_all_blueprints_generation.py
```

Then run focused tests for the changed code and manually QA through the actual surface: CLI command, MCP tool, HTTP API, simulation/replay blueprint, hardware procedure, or library driver.
