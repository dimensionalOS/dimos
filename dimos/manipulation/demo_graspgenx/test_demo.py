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

"""Hermetic coverage for the one-shot GraspGenX image demo."""

from pathlib import Path
import stat

import numpy as np
import pytest
from pytest_mock import MockerFixture

import dimos.manipulation.demo_graspgenx.demo as demo
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Header import Header

from . import __main__
from .demo import DemoResult, deployment_config, run_contributor_demo, run_demo
from .fixture import load_demo_clouds, load_scene_record


class FakeGraspProposer:
    def __init__(self) -> None:
        self.calls = 0

    def propose_grasps(self, object_pointcloud: PointCloud2) -> GraspCandidateArray:
        self.calls += 1
        center = object_pointcloud.pointcloud.get_center()
        candidates = [
            GraspCandidate(
                Pose(
                    {
                        "position": [float(center[0]), float(center[1]), float(center[2] + dz)],
                        "orientation": [0.0, 0.0, 0.0, 1.0],
                    }
                ),
                score,
            )
            for dz, score in ((0.16, 0.91), (0.15, 0.73), (0.14, 0.52))
        ]
        return GraspCandidateArray(
            Header(float(object_pointcloud.ts), "world"),
            candidates,
        )


def test_standard_data_fixture_is_deterministic() -> None:
    scene, object_cloud = load_demo_clouds()
    scene_again, object_again = load_demo_clouds()
    points, labels, metadata = load_scene_record()

    assert len(scene) == 3804
    assert len(object_cloud) == 3500
    assert metadata["counts"] == {
        "banana": 3500,
        "table": 256,
        "distractor": 48,
        "total": 3804,
    }
    assert metadata["timestamp"] == 1700000000.25
    assert scene.frame_id == object_cloud.frame_id == "world"
    assert scene.ts == object_cloud.ts == metadata["timestamp"]
    assert np.count_nonzero(labels == 0) == 3500
    np.testing.assert_array_equal(points, scene_again.points_f32())
    np.testing.assert_array_equal(object_cloud.points_f32(), object_again.points_f32())


def test_demo_runs_inference_once_and_writes_png(tmp_path: Path) -> None:
    proposer = FakeGraspProposer()
    output = tmp_path / "graspgenx.png"
    result = run_demo(proposer, output, gripper=deployment_config().gripper)

    assert proposer.calls == 1
    assert result == DemoResult(output.resolve(), 3804, 3500, 3, 0.91, "world")
    assert output.read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
    assert output.stat().st_size > 10_000
    assert stat.S_IMODE(output.stat().st_mode) == 0o644


def test_contributor_stops_adapter_after_render_failure(
    mocker: MockerFixture, tmp_path: Path
) -> None:
    adapter = mocker.patch.object(demo, "GraspGenXModule").return_value
    mocker.patch.object(demo, "run_demo", side_effect=RuntimeError("render"))

    with pytest.raises(RuntimeError, match="render"):
        run_contributor_demo(output_path=tmp_path / "failure.png")

    adapter.start.assert_called_once_with()
    adapter.stop.assert_called_once_with()


def test_one_config_drives_adapter_and_wireframe(mocker: MockerFixture, tmp_path: Path) -> None:
    base = deployment_config()
    config = base.model_copy(
        update={
            "gripper": base.gripper.model_copy(update={"offset_open": (0.0, 0.0, 0.2)}),
        }
    )
    adapter_class = mocker.patch.object(demo, "GraspGenXModule")
    run = mocker.patch.object(
        demo,
        "run_demo",
        return_value=DemoResult(tmp_path / "config.png", 1, 1, 1, 1.0, "world"),
    )

    run_contributor_demo(
        output_path=tmp_path / "config.png",
        config=config,
    )

    assert adapter_class.call_args.kwargs["gripper"] == config.gripper.model_dump()
    run.assert_called_once_with(
        adapter_class.return_value,
        tmp_path / "config.png",
        gripper=config.gripper,
    )


def test_python_module_entrypoint_is_direct_and_user_visible(
    mocker: MockerFixture, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    output = tmp_path / "entrypoint.png"
    run = mocker.patch.object(
        __main__,
        "run_contributor_demo",
        return_value=DemoResult(output, 1, 1, 3, 0.9, "world"),
    )

    assert __main__.main(["--output", str(output)]) == 0
    run.assert_called_once_with(output_path=output)
    assert f"candidates=3 image={output}" in capsys.readouterr().out


def test_empty_result_fails_explicitly(tmp_path: Path) -> None:
    class Empty:
        def propose_grasps(self, cloud: object) -> GraspCandidateArray:
            _, object_cloud = load_demo_clouds()
            return GraspCandidateArray(Header(object_cloud.ts, "world"), [])

    with pytest.raises(ValueError, match="no grasp candidates"):
        run_demo(
            Empty(),  # type: ignore[arg-type]
            tmp_path / "empty.png",
            gripper=deployment_config().gripper,
        )
