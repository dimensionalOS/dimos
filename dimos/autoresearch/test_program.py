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

import pytest

from dimos.autoresearch.program import TraceLine, run_program

FAKE = "dimos.autoresearch._test_helpers:fake_backend"

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
    run_program(path, tmp_path / "trial", "lift the cube", timeout_s, FAKE)
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
    outcome = run_program(path, tmp_path / "trial", "lift the cube", 60.0, FAKE)
    assert outcome.status == "raised"
    assert outcome.error.rstrip().endswith("ValueError: lift the cube")
    assert len(trace.read_text().splitlines()) == 1

    path.write_text("os = __import__('os')\n\n\ndef run(robot, task):\n    pass\n")
    outcome = run_program(path, tmp_path / "trial", "lift the cube", 60.0, FAKE)
    assert outcome.status == "raised"
    assert outcome.error.rstrip().endswith(
        "ImportError: program imports 'os'; only math may be imported"
    )
    assert trace.read_text() == ""


def test_a_program_is_killed_at_the_timeout(tmp_path: Path) -> None:
    path = tmp_path / "program.py"
    path.write_text("def run(robot, task):\n    while True:\n        pass\n")
    outcome = run_program(path, tmp_path / "trial", "lift the cube", 1.0, FAKE)
    assert outcome.status == "timeout"
    assert outcome.duration_s >= 1.0
    assert not (tmp_path / "trial" / "outcome.json").exists()
