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

"""The rerun-bridge command; the implementation lives in dimos.visualization.rerun."""

from __future__ import annotations

from typing import cast

import typer

from dimos.visualization.rerun.constants import RerunOpenOption


def rerun_bridge_cmd(
    memory_limit: str = typer.Option(
        "25%", help="Memory limit for Rerun viewer (e.g., '4GB', '16GB', '25%')"
    ),
    rerun_open: str = typer.Option("native", help="How to open Rerun: native, web, both, none"),
    rerun_web: bool = typer.Option(
        True, "--rerun-web/--no-rerun-web", help="Enable/Disable Rerun web server"
    ),
) -> None:
    """Launch the Rerun visualization bridge."""
    from dimos.visualization.rerun.bridge import run_bridge

    run_bridge(
        memory_limit=memory_limit,
        rerun_open=cast("RerunOpenOption", rerun_open),
        rerun_web=rerun_web,
    )
