# Copyright 2025-2026 Dimensional Inc.
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

from __future__ import annotations

from pathlib import Path

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.module import Module
from dimos.core.stream import In, Out

SNAPSHOT_PATH = Path(__file__).with_name("test_graph_snapshot.html")


class ImageData:
    pass


class DepthData:
    pass


class OdometryData:
    pass


class PlanData:
    pass


class CmdVelData:
    pass


class PointCloudData:
    pass


class CameraModule(Module):
    # intentionally doesn't match "color_image"
    color_img: Out[ImageData]
    depth_image: Out[DepthData]
    point_cloud: Out[PointCloudData]


class OdometryModule(Module):
    odometry: Out[OdometryData]


class PerceptionModule(Module):
    color_image: In[ImageData]
    depth_image: In[DepthData]
    odometry: In[OdometryData]


class PlannerModule(Module):
    odometry: In[OdometryData]
    plan: Out[PlanData]


class PlannerModule2(Module):
    odometry: In[OdometryData]
    plan: Out[PlanData]


class ControllerModule(Module):
    plan: In[PlanData]
    odometry: In[OdometryData]
    cmd_vel: Out[CmdVelData]


class ControllerModule2(Module):
    plan: In[PlanData]
    odometry: In[OdometryData]
    cmd_vel: Out[CmdVelData]


class VisualizerModule(Module):
    color_image: In[ImageData]
    point_cloud: In[PointCloudData]


_blueprint1 = autoconnect(
    CameraModule.blueprint(),
    OdometryModule.blueprint(),
    PerceptionModule.blueprint(),
    PlannerModule.blueprint(),
    ControllerModule.blueprint(),
    VisualizerModule.blueprint(),
)

_blueprint2 = autoconnect(
    CameraModule.blueprint(),
    PlannerModule.blueprint(),
    ControllerModule.blueprint(),
    VisualizerModule.blueprint(),
)

_blueprint3 = autoconnect(
    CameraModule.blueprint(),
    OdometryModule.blueprint(),
    PerceptionModule.blueprint(),
    PlannerModule.blueprint(),
    PlannerModule2.blueprint(),  # intenional double
    ControllerModule.blueprint(),
    ControllerModule2.blueprint(),
    VisualizerModule.blueprint(),
)
