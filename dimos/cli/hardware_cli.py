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

"""Hardware diagnostics and configuration commands."""

import typer

from dimos.cli.hardware.a1z import app as a1z_app
from dimos.cli.installer_cli import register as register_forwarders

app = typer.Typer(help="Diagnose and configure robot hardware", no_args_is_help=True)
app.add_typer(a1z_app, name="a1z")

# `hardware g1|jetson setup` is the Rust installer's; from an activated venv it must still reach it.
for robot in ("g1", "jetson"):
    robot_app = typer.Typer(
        help=f"Unitree {robot} bring-up (DimOS installer)", no_args_is_help=True
    )
    register_forwarders(robot_app, "setup")
    app.add_typer(robot_app, name=robot)
