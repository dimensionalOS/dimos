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

"""Shared local assets and native camera renderer for the SF555 office."""

from functools import partial
import os
from pathlib import Path

from dimos.robot.assets.model import RobotModel
from dimos.sim2.sensors.camera.renderers.unreal.renderer import UnrealRenderer

ASSETS = Path(os.environ.get("DIMOS_SF555_ASSETS", str(Path.home() / ".cache/dimos/sf555")))
PROJECT = Path(
    os.environ.get("DIMOS_UNREAL_PROJECT", str(ASSETS / "DimosRenderer/DimosRenderer.uproject"))
)
EXECUTABLE = Path(
    os.environ.get(
        "DIMOS_UNREAL_EXECUTABLE",
        "/Users/Shared/Epic Games/UE_5.7/Engine/Binaries/Mac/UnrealEditor.app/Contents/MacOS/UnrealEditor",
    )
)

OFFICE_MODEL = RobotModel.from_file(ASSETS / "office.urdf")
OFFICE_RENDERER = partial(
    UnrealRenderer,
    executable=EXECUTABLE,
    project=PROJECT,
    level="/Game/SF555/Maps/Office",
    authored_instances=("office",),
    log_path=ASSETS / "unreal-camera.log",
    startup_timeout=600,
    capture_timeout=30,
)
