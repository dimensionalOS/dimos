# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Passthrough commands that launch the DimOS terminal UIs."""

from __future__ import annotations

import sys

import typer


def spy(ctx: typer.Context) -> None:
    """Universal transport spy: topics, rates, sizes across all pubsub transports."""
    # A root-level `--transport` (before the subcommand) sets the stack's pubsub
    # backend — which single transport dimos processes participate on. The spy is an
    # observer: it watches every transport and takes its own repeatable `--transport`
    # filter *after* the subcommand. The two look alike but mean different things, so
    # reject the root placement rather than silently ignoring the requested filter.
    if (ctx.obj or {}).get("transport") is not None:
        typer.echo(
            "Error: `--transport` before `spy` sets the stack backend, which the spy "
            "ignores. Put the filter after the subcommand: `dimos spy --transport <name>`.",
            err=True,
        )
        raise typer.Exit(2)
    from dimos.cli.spy.run_spy import main as spy_main

    sys.argv = ["spy", *ctx.args]
    spy_main()


def lcmspy(ctx: typer.Context) -> None:
    """Alias for `dimos spy --transport lcm`."""
    from dimos.cli.spy.run_spy import lcm_only_argv, main as spy_main

    sys.argv = lcm_only_argv(list(ctx.args))
    spy_main()


def agentspy(ctx: typer.Context) -> None:
    """Agent spy tool for monitoring agents."""
    from dimos.cli.agentspy.agentspy import main as agentspy_main

    sys.argv = ["agentspy", *ctx.args]
    agentspy_main()


def humancli(ctx: typer.Context) -> None:
    """Interface interacting with agents."""
    from dimos.cli.human.humanclianim import main as humancli_main

    sys.argv = ["humancli", *ctx.args]
    humancli_main()


def top(ctx: typer.Context) -> None:
    """Live resource monitor TUI."""
    from dimos.cli.dtop import main as dtop_main

    sys.argv = ["dtop", *ctx.args]
    dtop_main()
