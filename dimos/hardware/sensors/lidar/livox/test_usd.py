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

from pathlib import Path
import sys
from typing import Any, cast

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
    """Every link by the name it travels under: a link prim is one with RigidBodyAPI."""
    from pxr import UsdPhysics

    return {
        prim.GetDisplayName() or prim.GetName()
        for prim in stage.Traverse()
        if prim.HasAPI(UsdPhysics.RigidBodyAPI)
    }


def test_the_baked_layer_parses_and_is_rooted_at_the_sensor() -> None:
    pytest.importorskip("pxr")

    stage = _stage(mid360_usd("lidar_link", "imu_link"))

    assert stage.GetDefaultPrim().GetName() == "mid360"


def test_the_configured_frames_are_the_names_the_layer_carries() -> None:
    pytest.importorskip("pxr")

    stage = _stage(mid360_usd("front/lidar", "front/imu"))

    # displayName is what a UrdfManager matches a mount frame on, so these are the names that
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
    # The point of get_usd: a UrdfManager puts this under one of its own links.
    pytest.importorskip("pxr")
    from dimos.robot.assets.usd import graft_usd

    root = """#usda 1.0
(
    defaultPrim = "robot"
)

def Xform "robot" (
    prepend apiSchemas = ["PhysicsRigidBodyAPI"]
    displayName = "base"
)
{
    def Xform "sensor_mount" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI"]
        displayName = "sensor_mount"
    )
    {
    }
}
"""

    stage = _stage(graft_usd(root, {"sensor_mount": mid360_usd("lidar_link", "imu_link")}))

    assert stage.GetPrimAtPath("/robot/sensor_mount/mid360/lidar_link")
    assert "lidar_link" in _links(stage)


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


def test_the_sensor_satisfies_the_urdf_manager_mount_contract() -> None:
    # The reason get_usd exists: UrdfManager rejects a mount whose module cannot be asked.
    from dimos.robot.urdf_manager import Mount

    mount = Mount.of(Mid360.blueprint(lidar_ip="192.168.1.155").namespace("front"))

    assert mount.module is Mid360
    assert mount.remote_name == "front/mid360"
    assert mount.frame_prefix == "front"


def test_the_baked_layer_authors_no_custom_attributes() -> None:
    # Standard schemas only, same as everything urdf_to_usd emits: a consumer should need
    # nothing of ours to read this.
    usd = mid360_usd("lidar_link", "imu_link")

    assert "custom " not in usd
    assert "dimos:" not in usd


def _answer(config: Mid360Config) -> list[str]:
    """Mid360.get_usd without a Mid360.

    A sync @rpc is tagged in place rather than wrapped, so it can be called with anything
    carrying a config, which keeps these tests off the NativeModule's rust build.
    """
    from types import SimpleNamespace

    return cast("list[str]", Mid360.get_usd(cast("Any", SimpleNamespace(config=config))))


def test_get_usd_hands_over_the_two_configured_frames_in_order() -> None:
    # Worth covering because swapping the two frames would be silent: both are strings.
    pytest.importorskip("pxr")
    config = Mid360Config(frame_id="chassis/lidar", imu_frame_id="chassis/imu")

    usd = _answer(config)[1]

    assert _links(_stage(usd)) == {"chassis/lidar", "chassis/lidar_body", "chassis/imu"}


def test_get_usd_roots_its_assets_at_this_module_rather_than_the_build_machine() -> None:
    """The baked constant is written on a build machine, whose paths mean nothing on a robot.

    So the root is resolved at call time from this module's own location, which is the only
    directory this driver can point at and be right about wherever it is installed.
    """
    root = _answer(Mid360Config())[0]

    assert Path(root) == Path(__file__).parent
