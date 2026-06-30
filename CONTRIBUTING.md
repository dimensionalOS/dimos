# Contributing to DimOS

DimOS welcomes community contributions. This guide explains how to start work, communicate scope, and submit a pull request that maintainers can review efficiently.

## Before you code

Choose the path that keeps discussion focused:

- Small, safe changes can go straight to a pull request.
- Non-trivial changes should start with a GitHub issue before implementation.
- If you already have a draft pull request for a non-trivial change, open or link an issue for the design discussion and keep the PR focused on the implementation.
- Core architecture changes should start with a GitHub issue or discussion before implementation.

New contributors can start with issues labeled `good first issue`.

Core architecture includes modules, streams, transports, blueprints, agents, public APIs, robot/platform support, and major dependency changes.

## Pull requests

Before opening a pull request, make sure it is:

- scoped to one issue or clearly stated problem
- linked to the relevant issue or discussion, unless it is a small, safe change
- focused, without unrelated cleanup or formatting changes
- clear about what changed and why
- validated with the checks relevant to the files you changed

The pull request template asks for the details maintainers need. Keep deeper design discussion in the issue or discussion that the PR links to, not in the PR thread.

## Validation

Run the relevant local checks before submitting. Pre-commit and CI will also guide you toward the required checks for the files you changed.

For test guidance, see [docs/development/testing.md](docs/development/testing.md).

## AI-assisted contributions

AI-assisted contributions are welcome when the human contributor owns the result.

If AI materially helped prepare a pull request, disclose that in the PR. You are responsible for understanding the change, checking it against the agreed scope, and running relevant validation.

If you use an AI coding agent, review its work before submitting. Do not submit an agent-generated PR that you cannot explain yourself. If the PR is not ready for human review, keep it as a draft.

Before submitting, make sure you can explain in your own words:

- what changed and why
- which issue or discussion scope the PR follows
- what checks you ran and the results
- any risks, assumptions, or follow-up work

## Contributor License Agreement

Before submitting a pull request, read and approve the [Contributor License Agreement](CLA.md).
