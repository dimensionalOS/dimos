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

import pytest

from dimos.sim2.camera import SimCameraConfig
from dimos.sim2.mujoco.renders import RenderQuality

# A camera's config builds its USD, which needs usd-core (absent on linux-aarch64).
pytest.importorskip("pxr")


def test_a_camera_left_alone_renders_as_the_view_does_by_default():
    assert SimCameraConfig().quality() == RenderQuality()


def test_a_cameras_quality_fields_reach_the_view():
    config = SimCameraConfig(shadows=False, shadow_size=1024, samples=0, tone_map="linear")
    assert config.quality() == RenderQuality(
        shadows=False, shadow_size=1024, samples=0, tone_map="linear"
    )
