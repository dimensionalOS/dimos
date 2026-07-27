# AI Usage Policy

Dimensional is an agent-native project. The whole point of dimOS is that you can "vibecode" robots in natural language, and we build the system itself with heavy AI assistance. **We are not an anti-AI project. Quite the opposite.**

This policy exists for one reason. Contributing to dimOS carries a quality and safety bar that simply using dimOS does not. The code here moves real hardware and is operating in production in safety-critical real-world environments. The problem we are guarding against is not AI tools. It is contributions where no human actually understands the code. So please read and follow these rules.

## The Rules

- **You must fully understand your PR**
This is what actually matters. If you cannot explain what your code changes do and how they interact with the rest of the system without the aid of an AI tool, do not submit them. Feel free to *gain* that understanding by interrogating an agent with access to the codebase before contributing. Before you submit, make sure you can explain in your own words:
  - what changed and why
  - which issue or discussion scope the PR follows
  - what checks you ran and their results
  - any risks, assumptions, or follow-up work

- **Disclosing AI usage (optional)**
We are not tracking who used which tool. If you want to note the tool and model (for example Claude Code with Opus 4.8, or Cursor with GPT-5), feel free to include them. What we care about is that you understand and stand behind the code, not whether AI helped write it.

- **Hardware changes demand extra scrutiny**
Control loops, motor drivers, planners, and anything that produces motion can damage equipment or hurt people. Understanding your change line by line is the bar for every PR, AI-assisted or not; for these areas you must also test in simulation or replay before making any hardware claim. See the safety section of [`CONTRIBUTING.md`](CONTRIBUTING.md).

- **Issues and discussions may use AI, but a human must review and edit the output**
AI tends to be verbose and to pad text with noise. Do your own research, trim it down, and write in your own voice, especially for feature proposals and bug reports.

- **No AI-generated media**
Text and code are the only acceptable AI-generated contributions. Please do not submit AI-generated art, images, video, audio, or diagrams.

## Using dimOS vs Contributing to dimOS

It is worth being explicit, because dimOS lives on both sides of this line.

- **Go wild with AI while using dimOS.** Vibecode your robot, point your favorite agent at [`AGENTS.md`](AGENTS.md), generate skills and blueprints in natural language, and let an LLM drive the CLI and MCP tools. This is the product working as intended.

- **Contributions may be AI-assisted, but must be human-understood.** The same tools are welcome, but the bar is that a human reviews, understands, and stands behind every line going into the shared codebase.

## Maintainers

These rules apply to outside contributions; maintainers may use AI tools at their discretion. They have earned the trust to apply good judgment about when and how.

## Reviews Cost Maintainer Time

Issues, discussions, and pull requests are where your work meets a maintainer's calendar. Low-effort or unreviewed submissions, especially confident-looking code or prose that nobody can explain, move the whole job of verification onto a small team and slow everyone else down.

This policy is about quality and safety on real systems, not about banning useful tools.
