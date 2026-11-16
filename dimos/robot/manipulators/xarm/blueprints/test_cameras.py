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

from __future__ import annotations

import json

import pytest

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.manipulation.calibration import store
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.robot.manipulators.xarm import cameras
from dimos.robot.manipulators.xarm.blueprints.cameras import xarm6_cameras


def _atoms(module_type):  # type: ignore[no-untyped-def]
    return [a for a in xarm6_cameras.active_blueprints if issubclass(a.module, module_type)]


def test_two_realsenses_run_side_by_side_with_the_chest_one_namespaced() -> None:
    names = sorted(a.name for a in _atoms(RealSenseCamera))
    assert names == ["chest/realsensecamera", "realsensecamera"]
    chest = next(a for a in _atoms(RealSenseCamera) if a.name.startswith("chest/"))
    assert chest.kwargs["frame_id_prefix"] == "chest"
    # its clouds are its own; its tf edges join the one tree
    assert (
        xarm6_cameras.remapping_map[("chest/realsensecamera", "pointcloud")] == "chest/pointcloud"
    )
    assert ("chest/realsensecamera", "tf") not in xarm6_cameras.remapping_map


def test_the_planner_publishes_both_camera_mounts() -> None:
    (planner_atom,) = _atoms(ManipulationModule)
    edges = {(t.frame_id, t.child_frame_id) for t in planner_atom.kwargs["static_transforms"]}
    assert edges == {("link6", "camera_link"), ("world", "chest/camera_link")}
    assert "link6" in planner_atom.kwargs["model"].tf_extra_links


def test_each_camera_takes_its_serial_from_the_command_line() -> None:
    # the namespaced instance is chest/realsensecamera; on the command line the slash is an
    # underscore
    parsed = BlueprintConfigParser(xarm6_cameras).parse(
        [
            "--xarmconnection.ip",
            "10.0.0.2",
            "--realsensecamera.serial-number",
            "111",
            "--chest_realsensecamera.serial-number",
            "222",
        ],
        environ={},
    )
    assert parsed.module_configs["realsensecamera"]["serial_number"] == "111"
    assert parsed.module_configs["chest/realsensecamera"]["serial_number"] == "222"


def test_a_run_on_this_machine_beats_the_shipped_chest_result(tmp_path, monkeypatch) -> None:
    monkeypatch.setattr(store, "HAND_EYE_DIR", tmp_path)
    shipped = json.loads((cameras.XARM_CALIBRATION_DIR / "xarm6_chest_camera.json").read_text())

    mount = cameras.xarm6_chest_camera()
    assert (mount.frame_id, mount.child_frame_id) == ("world", "chest/camera_link")
    assert mount.translation.x == pytest.approx(shipped["mount"]["translation_m"][0])

    local = dict(shipped)
    local["mount"] = {**shipped["mount"], "translation_m": [0.1, 0.2, 0.3]}
    (tmp_path / "xarm6_chest_camera.json").write_text(json.dumps(local))
    assert cameras.xarm6_chest_camera().translation.x == pytest.approx(0.1)

    # a failed re-run never replaces a passed result
    (tmp_path / "xarm6_chest_camera.json").write_text(json.dumps({**local, "passed": False}))
    assert cameras.xarm6_chest_camera().translation.x == pytest.approx(
        shipped["mount"]["translation_m"][0]
    )
