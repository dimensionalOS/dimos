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

"""OpenArm Mini hardware commands.

This module is imported by the global hardware CLI. Keep command modules safe
to import without Rich, NumPy, control, manipulation, or hardware SDK imports.
"""

import typer

from dimos.teleop.openarm_mini.cli import calibrate, joint_tui, setup_motor_id

app = typer.Typer(help="Configure and inspect OpenArm Mini leader hardware", no_args_is_help=True)
app.command("calibrate")(calibrate.main)
app.command("joint-tui")(joint_tui.main)
app.command("setup-motor-id")(setup_motor_id.main)
