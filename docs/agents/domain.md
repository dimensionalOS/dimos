# DimOS agent domain context

## Context loading

Before working on a change, load the repository context in this order:

1. Read `AGENTS.md` and follow its applicable instructions.
2. Read `openspec/config.yaml` for the OpenSpec schema, terminology, and rules.
3. Read the relevant files under `openspec/specs/`.
4. Read the root `CONTEXT.md` if it exists.
5. Read relevant records under `docs/adr/` if that directory exists.

`CONTEXT.md` and `docs/adr/` are optional. If either is absent, continue
silently; do not report the absence as an error. Select specs and ADRs based on
the affected behavior and implementation surface rather than reading
unrelated material.

## Two meanings of “spec”

Keep these terms separate:

- An **OpenSpec spec** is a behavior specification under `openspec/specs/`.
  It describes observable behavior, user or developer outcomes, public
  interfaces, safety constraints, and testable scenarios.
- A **DimOS Python Spec Protocol** is a code-level interface contract, usually
  a `Protocol` inheriting from `dimos.spec.utils.Spec`, often found in a
  `*_spec.py` file. It describes module RPCs and injected interfaces.

An OpenSpec spec is not a Python Protocol, and a Python Protocol does not
replace an OpenSpec behavioral requirement. Keep implementation details such as
class names, module wiring, stream types, generated registries, and rollout
steps in the OpenSpec change design or tasks unless they are externally
observable.

## Work layout

Organize work through this chain:

```text
Linear issue -> OpenSpec change -> implementation tasks -> pull request
```

Linear provides intake and tracking. The OpenSpec change is the source of truth
for the behavioral change, design, and tasks. The pull request implements and
reviews those tasks. Keep the identifiers and links aligned across all three
artifacts; any Linear link edit requires user confirmation before it is made.

When a task affects behavior, update the relevant OpenSpec change and, where
appropriate, the corresponding spec under `openspec/specs/`. Include concrete
scenarios for behavioral requirements. Call out DimOS Python Spec Protocols,
blueprint composition, streams, skills/MCP exposure, generated files, and
hardware, simulation, or replay assumptions in design and task material when
they are relevant.

## Conflicting guidance

Surface conflicts between an ADR and an OpenSpec spec explicitly. Do not
silently reconcile, overwrite, or guess which decision applies. Report the
conflict, identify the affected behavior or implementation, and ask for the
decision or update the authoritative document only when instructed.
