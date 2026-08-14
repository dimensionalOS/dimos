# Contributing to Dimensional

Thank you for your interest in contributing to Dimensional. This document describes how to contribute to dimOS, whether you are opening an issue, starting a discussion, or sending a pull request.

Dimensional is a fast-moving, pre-release robotics OS maintained by a small team. We genuinely want your contributions, and a few minutes spent reading this will make the difference between a PR that gets merged and one that goes stale. Thank you ❤️

> [!NOTE]
> This document covers the process: how to report, propose, and get changes accepted.
>
> For the technical mechanics, see [AGENTS.md](AGENTS.md). That covers building, testing, and running dimOS, the repo layout, the module and blueprint system, code style, the git workflow, and how to add a skill. The two documents are meant to be read together.

## Table of contents

- [The critical rule](#the-critical-rule)
- [AI usage](#ai-usage)
- [Safety first](#safety-first)
- [First-time contributors](#first-time-contributors)
- [Issues vs discussions](#issues-vs-discussions)
- [How to](#how-to)
- [Contributor License Agreement](#contributor-license-agreement)
- [Code of conduct](#code-of-conduct)

## The critical rule
**You must understand what you submit.** This is the one rule we will not bend on.

dimOS moves real hardware. A single misunderstanding in a control loop or a driver can crash a quadruped, damage an arm, or send a drone off course. The cost of a change you do not fully grasp is not a failed test. It is broken equipment or someone getting hurt.

So before you open a pull request, make sure you can explain what your change does and how it fits into the rest of the system in your own words. If you cannot do that without leaning on an AI tool, the change is not ready. Writing code with AI is fine and often encouraged. Shipping code you have not reasoned through yourself is not.

## AI usage
dimOS is an agent-native project. Our entire premise is that you can "vibecode" robots in natural language, and we build with AI tools every day. AI is welcome here.

That said, contributing to dimOS has a quality bar that using dimOS does not. We have a short, clear policy covering disclosure and the human-in-the-loop requirement. Please read it before contributing: [AI Usage Policy](AI_POLICY.md). This is important.

## Safety first
> [!WARNING]
> Read this section if your change touches hardware.

dimOS controls physical robots. Contributions that affect control loops, motor drivers, planners, or anything that can produce motion carry real-world risk.

- **Test in simulation or replay before hardware.** Most subsystems can be exercised with `dimos --simulation run ...` or `dimos --replay run ...` with no robot attached. Use these first.
- **Never submit untested motion, control, or driver changes.** If you tested on real hardware, say which robot, which firmware, and what you observed in the PR description. If you could not test on hardware, say so explicitly.
- **Default to caution.** When in doubt about the safety implications of a change, open a discussion before writing code, or ask in [Discord](https://discord.gg/dimos).



## First-time contributors
We want people to use Dimensional, learn it, and help us build it. You do not need robotics hardware or prior experience with every part of the stack to make a useful contribution.

**Start here.**
Browse [open issues labeled](https://github.com/dimensionalOS/dimos/issues?q=is%3Aopen+is%3Aissue+label%3A%22good+first+issue%22) `good first issue`. These are scoped tasks that maintainers have marked as approachable for newcomers. Pick one, read the description, and comment if you want to work on it or need clarification. Maintainers are happy to point you in the right direction.

**Get set up.**
Follow [AGENTS.md](AGENTS.md) for install (`uv sync --extra all`), running blueprints in simulation or replay, and the test workflow (`uv run pytest`). Most changes can be developed and validated without a physical robot.

**This still applies to you.**
The [critical rule](#the-critical-rule) and [AI policy](AI_POLICY.md) apply to every outside contribution, including yours. Use AI tools if they help you learn the codebase, but understand what you submit. Link your PR to the issue you are working on.

## Issues vs Discussions
We keep a clean separation so maintainers can find work that is actually ready.

- **[Issue](https://github.com/dimensionalOS/dimos/issues) tracker is for actionable, accepted work.** Every open issue should be something a contributor can pick up and start on.
- **[Discussions](https://github.com/dimensionalOS/dimos/discussions) are for everything else.** This includes bug reports that need triage, feature ideas, design questions, and Q&A. Once a discussion produces a well-scoped, agreed-upon task, a maintainer promotes it to an issue.

This means most contributions start as a discussion, not an issue or a PR.

## How to:
Pick the path that matches what you want to do.
### Report a bug
1. Search [existing issues](https://github.com/dimensionalOS/dimos/issues) and [discussions](https://github.com/dimensionalOS/dimos/discussions) first, including closed ones. Your bug may already be known or fixed.
2. If it is new, open a [GitHub discussion](https://github.com/dimensionalOS/dimos/discussions) describing the problem. Include your OS, install method (`uv`, Nix, Docker), the robot or blueprint involved, the exact command you ran, and the full error or unexpected behavior.
3. If an open issue or discussion already matches, do not add a "+1" comment, use an emoji reaction or the upvote button instead. Comments notify everyone subscribed. Reactions do not.

### Propose a feature
1. Search issues and discussions to make sure it has not been proposed already.
2. Open a [discussion](https://github.com/dimensionalOS/dimos/discussions) in the feature or ideas category describing what you want and why. Keep it focused.
3. If maintainers accept it, it becomes an issue. That is your green light to implement.

### Submit a pull request
1. **Match the PR to its weight:** Small, safe changes like typo and doc fixes can go straight to a PR. Anything non-trivial should map to an accepted issue first. Core architecture changes — modules, streams, transports, blueprints, agents, public APIs, robot or platform support, and major dependency changes — should start as a [discussion](https://github.com/dimensionalOS/dimos/discussions) before any code. Unscoped PRs are expensive to review, so they may sit unreviewed or be closed. When in doubt, open a discussion and link your branch.
2. **Sign the CLA:** All contributions require a signed [Contributor License Agreement](CLA.md). See [Contributor License Agreement](#contributor-license-agreement) below.
3. **Make every push count:** Use branch prefixes (`feat/`, `fix/`, `docs/`, and so on), target `main`, and run `uv run pytest` and pre-commit locally before pushing. Do not force-push or spam pushes; every push triggers roughly an hour of CI.
4. **Fill out the PR description template:** GitHub pre-fills it when you open the pull request. Do not clear it. Complete every section:
  - Contribution path: Link the issue or discussion, or tick the small-safe-change box.
  - Problem: What is broken or missing.
  - Solution: What your change does.
  - How to Test: The sim, replay, or hardware steps a reviewer can follow.
  - AI assistance: The tool and model used and how much, as required by the [AI policy](AI_POLICY.md).
  - Checklist: Including the CLA confirmation.
   If you strip out the template, maintainers may close the PR without reviewing it.
5. **Your PR description is the first thing we judge.** A clear, specific description is our first heuristic for whether a change was understood or slopped together. Write a good one, and ideally ping us on [Discord](https://discord.gg/dimos) before you start and after you push. If the PR is not yet ready for human review, keep it as a draft.
6. **PRs are not the place to debate design.** If a feature needs discussion, use a discussion and link your branch.

### Ask a question
Open a [Q&A discussion](https://github.com/dimensionalOS/dimos/discussions), or join the [Discord](https://discord.gg/dimos) and ask in the help channel. Questions do not need the detail a bug report does.

## Contributor License Agreement

dimOS is developed by Dimensional Inc. Before your first contribution can be merged, you will need to sign our [Contributor License Agreement](CLA.md). It clarifies the IP license you are granting and protects both you and the project. It does not take away your right to use your own work elsewhere. The signing prompt will appear on your first pull request.

## Code of conduct

Be respectful. There are humans on the other side of every issue, discussion, and review. Low-effort or hostile contributions put the burden of cleanup on a small team, so please do not create them. We are happy to help newcomers learn and grow. Meet us halfway with effort and good faith.

---

Thanks for helping build the operating system for physical space. We are glad you are here.
