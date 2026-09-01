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

"""`dimos map` voxel-map commands; the implementations live in dimos.mapping.cli."""

import typer

from dimos.mapping.cli.map import main as _map_main
from dimos.mapping.cli.pose_fill import main as _map_pose_fill_main
from dimos.mapping.cli.rename import main as _map_rename_main
from dimos.mapping.cli.replay import main as _map_replay_main
from dimos.mapping.cli.replay_marker import main as _map_replay_marker_main

map_app = typer.Typer(help="Voxel-map tools over recorded sqlite datasets")
map_app.command("global")(_map_main)
map_app.command("rename")(_map_rename_main)
map_app.command("pose-fill")(_map_pose_fill_main)
map_app.command("replay")(_map_replay_main)
map_app.command("replay-marker")(_map_replay_marker_main)
