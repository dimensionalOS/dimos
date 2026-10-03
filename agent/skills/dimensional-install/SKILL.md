---
name: dimensional-install
description: Set up DimOS from an existing checkout/environment or a new installation, following its own version-specific instructions with the normal coding tools.
---

Start with one question: drop a path to an existing DimOS checkout/environment, or describe the robot/app you want to build. Wait for that answer. Do not assume the current directory needs a new `.venv`.

Use the existing Read and Bash tools. There is no harness-owned DimOS install recipe.

1. Inspect the supplied path, OS, available Python/package tools and existing DimOS installations. Distinguish a checkout, Python environment, app workspace and remote MCP endpoint. Reuse a working installation; when several match, ask which one to use.
2. Read that checkout/version's `README.md`, `pyproject.toml` and linked installation instructions. For a new package install, retrieve the official DimensionalOS/dimos README and linked OS-specific instructions from GitHub, selecting a release/ref with the user when needed. Report the source/ref you used. Do not invent commands if the instructions cannot be retrieved. Python requirements, extras and system dependencies come from those instructions, not this skill.
3. Explain the selected destination and required changes briefly, then execute the documented commands for the user's requested setup. Show command progress in normal tool cards, inspect failures and resolve them. Never replace an existing environment, install every robot extra by default, or start physical hardware as an install check. System changes needing elevated privileges require the user's involvement.
4. Verify the selected executable, `importlib.metadata.version("dimos")`, imported module path, CLI help and blueprint discovery. Run package checks from outside the checkout so the current directory cannot shadow the installed package. Distinguish installed distribution metadata from a checkout version string. Inspect `dimos status` when supported; distinguish installed software from a running blueprint. Only start an explicitly requested replay/simulation or hardware target. Do not claim installation succeeded from package-manager output alone.
5. Save verified selections through the existing CLI: `"$DIMCODE_NODE" "$DIMCODE_CLI" dimos /absolute/bin/dimos`, `python /absolute/bin/python`, and `workspace /absolute/app`. The launcher variables work for source builds without a global `dimcode` command. Use absolute paths in the current session; the saved workspace applies to subsequent launches. Configure `connect NAME URL` / `relay URL [ROBOT]` only for endpoints the user selected; use `/reload` after the turn to discover changed MCP skills.

Provider credentials and the gateway-at-login choice belong to the bootstrap UI. Never request API keys in chat, read auth files, or dump environment variables. Use `/login` for model authentication. Do not stop/restart the gateway hosting this conversation from a Bash tool. If the user wants to change daemon startup later, give the exact `service install/status/uninstall` launcher command to run after detaching.

Finish with the selected paths, verified version, available blueprint result, what is actually running, and any remaining dependency or connection issue. Setup is a normal persistent Pi conversation: the user can ask questions, correct the plan, detach or resume.
