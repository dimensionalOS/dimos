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

from pathlib import Path
import time
from typing import Any

import pytest
from scipy.spatial.transform import Rotation

from dimos.manipulation.code_policy import backend
from dimos.manipulation.code_policy._test_helpers import FAKE_BACKEND
from dimos.manipulation.code_policy.api import RobotCallError, TraceLine
from dimos.manipulation.code_policy.backend import DimosBackend, MapStamps, program_box
from dimos.manipulation.code_policy.runner import run_program
from dimos.manipulation.manipulation_spec import PlanningGroupInfo
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

LIFT = """
import math


def run(robot, task):
    cube = robot.scene.objects(["cube"])[0]
    x, y, z = cube.position
    robot.scene.clear(cube.id)
    robot.gripper.open()
    robot.arm.move_pose([x, y, z + 0.1], [math.pi, 0.0, 0.0])
    robot.gripper.close()
    try:
        robot.arm.move_pose([x, y, -0.2])
    except Exception as error:
        robot.log(f"could not go lower: {error}")
    robot.arm.move_linear(dz=0.1)
    robot.scene.restore()
    robot.observe()
"""


def run(tmp_path: Path, program: str, timeout_s: float = 60.0) -> tuple[Path, list[TraceLine]]:
    path = tmp_path / "program.py"
    path.write_text(program)
    run_program(path, tmp_path / "trial", "lift the cube", timeout_s, FAKE_BACKEND)
    trace = (tmp_path / "trial" / "trace.jsonl").read_text()
    return tmp_path / "trial", [TraceLine.model_validate_json(line) for line in trace.splitlines()]


def test_every_call_is_traced_with_keyframes_around_motion(tmp_path: Path) -> None:
    trial, lines = run(tmp_path, LIFT)
    assert [(line.call, line.status) for line in lines] == [
        ("scene.objects", "OK"),
        ("scene.clear", "OK"),
        ("gripper.open", "OK"),
        ("arm.move_pose", "OK"),
        ("gripper.close", "OK"),
        ("arm.move_pose", "PLANNING_FAILED"),
        ("log", "OK"),
        ("arm.move_linear", "OK"),
        ("scene.restore", "OK"),
        ("observe", "OK"),
    ]
    assert lines[0].result == [
        {"id": "0", "name": "cube", "position": [0.4, 0.0, 0.02], "size": [0.04, 0.04, 0.04]}
    ]
    assert lines[1].args == {"object_id": "0", "margin": 0.03}
    assert lines[3].args["orientation"] == pytest.approx([1.0, 0.0, 0.0, 0.0])
    assert lines[5].args == {"position": [0.4, 0.0, -0.2], "orientation": None}
    assert "below the table" in lines[5].result
    assert "below the table" in lines[6].args["text"]
    motion = [line for line in lines if line.before is not None]
    assert [line.call for line in motion] == [
        "gripper.open",
        "arm.move_pose",
        "gripper.close",
        "arm.move_pose",
        "arm.move_linear",
    ]
    for line in motion:
        assert line.after is not None
        assert (trial / line.before).is_file() and (trial / line.after).is_file()
    assert (trial / lines[-1].result).is_file()


def test_a_raising_program_reports_its_traceback(tmp_path: Path) -> None:
    path = tmp_path / "program.py"
    trace = tmp_path / "trial" / "trace.jsonl"
    path.write_text(
        "def run(robot, task):\n    robot.arm.joints()\n    raise ValueError(task.text)\n"
    )
    outcome = run_program(path, tmp_path / "trial", "lift the cube", 60.0, FAKE_BACKEND)
    assert outcome.status == "raised"
    assert outcome.error.rstrip().endswith("ValueError: lift the cube")
    assert len(trace.read_text().splitlines()) == 1

    path.write_text("os = __import__('os')\n\n\ndef run(robot, task):\n    pass\n")
    outcome = run_program(path, tmp_path / "trial", "lift the cube", 60.0, FAKE_BACKEND)
    assert outcome.status == "raised"
    assert outcome.error.rstrip().endswith(
        "ImportError: program imports 'os'; only math may be imported"
    )
    assert trace.read_text() == ""


def test_a_program_is_killed_at_the_timeout(tmp_path: Path) -> None:
    path = tmp_path / "program.py"
    path.write_text("def run(robot, task):\n    while True:\n        pass\n")
    outcome = run_program(path, tmp_path / "trial", "lift the cube", 1.0, FAKE_BACKEND)
    assert outcome.status == "timeout"
    assert outcome.duration_s >= 1.0
    assert not (tmp_path / "trial" / "outcome.json").exists()


class Bench:
    """A scene holding a cube and a bowl, a box filter that keeps its boxes by
    name and, while ``mapping``, sends a map captured after each change, and an
    arm whose tip, tilted, goes wherever ``move_pose`` sends it."""

    info = PlanningGroupInfo("arm", (), "world", "tip", True, grasp_point=(0.0, 0.0, 0.08))

    def __init__(self) -> None:
        self.tip = PoseStamped(
            position=[0.3, 0.1, 0.2], orientation=Quaternion.from_euler(Vector3(2.5, 0.3, -0.7))
        )
        self.boxes: set[str] = set()
        self.maps = MapStamps()
        self.mapping = True

    def _send_map(self) -> None:
        if self.mapping:
            self.maps.on_map(PointCloud2(ts=time.time() + 1.0))

    def pose(self) -> PoseStamped:
        return self.tip

    def move_pose(
        self, position: tuple[float, float, float], *, orientation: tuple[float, ...]
    ) -> None:
        self.tip = PoseStamped(position=list(position), orientation=Quaternion(*orientation))

    def scan(self, names: list[str], window_s: float = 10.0) -> list[dict[str, Any]]:
        found = [("cube", 0.2), ("bowl", 0.36)]
        return [
            {"id": name, "label": name, "position": [x, 0.0, 0.02], "size": [0.04] * 3}
            for name, x in found
            if name in names
        ]

    def set_box(self, name: str, pose: object, size: object, frame_id: str | None = None) -> bool:
        self.boxes.add(name)
        self._send_map()
        return True

    def clear_box(self, name: str) -> bool:
        removed = name in self.boxes
        self.boxes.discard(name)
        self._send_map()
        return removed


def test_cleared_objects_stay_out_of_the_map_together_until_restore() -> None:
    bench = Bench()
    live = DimosBackend(bench, bench, bench, bench, bench.maps)
    live.objects(["cube", "bowl"])
    live.clear_object("cube", 0.03)
    live.clear_object("bowl", 0.03)
    assert bench.boxes == {program_box("cube"), program_box("bowl")}
    live.restore()
    assert bench.boxes == set()


def test_clearing_an_object_times_out_when_no_newer_map_arrives(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(backend, "MAP_TIMEOUT_S", 0.0)
    bench = Bench()
    bench.mapping = False
    live = DimosBackend(bench, bench, bench, bench, bench.maps)
    live.objects(["cube"])
    with pytest.raises(RobotCallError) as raised:
        live.clear_object("cube", 0.03)
    assert raised.value.status == "TIMEOUT"


def test_poses_are_the_grasp_point_and_round_trip_to_the_same_tip_pose() -> None:
    bench = Bench()
    live = DimosBackend(bench, bench, bench, bench, bench.maps)
    tip = bench.tip
    grasp = live.pose()
    expected = Rotation.from_quat(tip.orientation.to_tuple()).apply([0.0, 0.0, 0.08])
    assert grasp.position == pytest.approx(tuple(tip.position.to_tuple() + expected))
    live.move_pose((0.0, 0.0, 0.0), None)
    live.move_pose(grasp.position, grasp.orientation)
    assert bench.tip.position.to_tuple() == pytest.approx(tip.position.to_tuple())
    assert bench.tip.orientation.to_tuple() == pytest.approx(tip.orientation.to_tuple())
