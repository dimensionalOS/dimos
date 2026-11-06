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

"""The baked MID-360 USD layer: it parses, it carries the configured frames, and it grafts.

Nothing here checks the geometry numbers. The shell box and its offset are hand-set
placeholders, so a test over them would only restate the constant.
"""

from __future__ import annotations

import sys
from typing import Any

import pytest

from dimos.hardware.sensors.lidar.livox.module import (
    _IMU_TOKEN,
    _LIDAR_TOKEN,
    Mid360,
    Mid360Config,
    mid360_usd,
)


def _stage(usd: str) -> Any:
    from pxr import Sdf, Usd

    layer = Sdf.Layer.CreateAnonymous(".usda")
    assert layer.ImportFromString(usd), "the baked MID-360 USD is not a readable layer"
    return Usd.Stage.Open(layer)


def _links(stage: Any) -> set[str]:
    return {
        str(prim.GetAttribute("dimos:link").Get())
        for prim in stage.Traverse()
        if prim.GetAttribute("dimos:link") and prim.GetAttribute("dimos:link").HasAuthoredValue()
    }


def test_the_baked_layer_parses_and_is_rooted_at_the_sensor() -> None:
    pytest.importorskip("pxr")

    stage = _stage(mid360_usd("lidar_link", "imu_link"))

    assert stage.GetDefaultPrim().GetName() == "mid360"


def test_the_configured_frames_are_the_names_the_layer_carries() -> None:
    pytest.importorskip("pxr")

    stage = _stage(mid360_usd("front/lidar", "front/imu"))

    # dimos:link is what a UsdManager matches a mount frame on, so these are the names that
    # have to follow the config rather than the canonical ones baked into the prim names.
    assert _links(stage) == {"front/lidar", "front/lidar_body", "front/imu"}


def test_no_placeholder_survives_substitution() -> None:
    usd = mid360_usd("a", "b")

    assert _LIDAR_TOKEN not in usd
    assert _IMU_TOKEN not in usd


def test_the_defaults_describe_the_frames_the_driver_actually_stamps() -> None:
    # The layer is useless if it names frames no one publishes, so it tracks the config.
    pytest.importorskip("pxr")
    config = Mid360Config()

    stage = _stage(mid360_usd(config.frame_id, config.imu_frame_id))

    assert {config.frame_id, config.imu_frame_id} <= _links(stage)


def test_the_sensor_grafts_onto_the_frame_it_is_bolted_to() -> None:
    # The point of get_usd: a UsdManager puts this under one of its own links.
    pytest.importorskip("pxr")
    from dimos.robot.assets.usd import graft_usd

    root = """#usda 1.0
(
    defaultPrim = "robot"
)

def Xform "robot"
{
    custom string dimos:link = "base"

    def Xform "sensor_mount"
    {
        custom string dimos:link = "sensor_mount"
    }
}
"""

    stage = _stage(graft_usd(root, {"sensor_mount": mid360_usd("lidar_link", "imu_link")}))

    assert stage.GetPrimAtPath("/robot/sensor_mount/mid360/lidar_link")
    assert "lidar_link" in _links(stage)


def test_the_baked_layer_still_matches_the_urdf_it_came_from() -> None:
    """The constant is generated, so it can drift from its source without anyone noticing."""
    pytest.importorskip("pxr")
    from pathlib import Path

    from dimos.robot.assets.model import RobotModel
    from dimos.robot.assets.usd import urdf_to_usd

    urdf = Path(__file__).parent / "mid360.urdf"

    regenerated = urdf_to_usd(RobotModel.from_file(urdf).load())

    # mid360.urdf uses the canonical frame names, which is what the placeholders default to.
    assert mid360_usd("lidar_link", "imu_link") == regenerated, (
        "mid360.urdf and _MID360_USD have diverged; regenerate the constant (see mid360.urdf)"
    )


def test_describing_itself_costs_no_usd_dependency() -> None:
    # usd-core has no linux-aarch64 wheel, so a Jetson running this driver must never need it.
    # That is the whole reason the layer is baked instead of converted here.
    import subprocess

    code = (
        "import sys; import dimos.hardware.sensors.lidar.livox.module as m; "
        "m.mid360_usd('a', 'b'); "
        "assert 'pxr' not in sys.modules, 'importing Mid360 pulled in pxr'"
    )

    subprocess.run([sys.executable, "-c", code], check=True)


def test_the_sensor_satisfies_the_usd_manager_mount_contract() -> None:
    # The reason get_usd exists: UsdManager rejects a mount whose module cannot be asked.
    from dimos.robot.usd_manager import Mount

    mount = Mount.of(Mid360.blueprint(lidar_ip="192.168.1.155").namespace("front"))

    assert mount.module is Mid360
    assert mount.remote_name == "front/mid360"
    assert mount.frame_prefix == "front"
