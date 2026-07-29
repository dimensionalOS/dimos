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

import dimos.manipulation.demo_graspgenx.demo as demo
from dimos.manipulation.grasping.grasp_gen_x import GraspGenXModule
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.std_msgs.Header import Header

from .demo import DemoResult, deployment_config, run_contributor_demo, run_demo
from .fixture import load_demo_clouds, load_scene_record


class FakeGraspProposer:
    def __init__(self) -> None:
        self.calls = 0

    def propose_grasps(self, object_pointcloud: object) -> GraspCandidateArray:
        self.calls += 1
        cloud = object_pointcloud  # type: ignore[union-attr]
        center = cloud.pointcloud.get_center()  # type: ignore[union-attr]
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
        return GraspCandidateArray(Header(float(cloud.ts), "world"), candidates)  # type: ignore[union-attr]


def fake_factory(_config: object) -> object:
    return object()


def fake_inference(_sampler: object, _points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    poses = np.repeat(np.eye(4, dtype=np.float32)[None, ...], 3, axis=0)
    poses[:, :3, 3] = np.asarray([0.25, 0.18, 0.16])
    return poses, np.asarray([0.91, 0.73, 0.52], dtype=np.float32)


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


def test_direct_adapter_config_and_cleanup() -> None:
    config = deployment_config()
    module = GraspGenXModule(config, factory=fake_factory, inference=fake_inference)
    _, object_cloud = load_demo_clouds()
    try:
        module.start()
        result = module.propose_grasps(object_cloud)
        assert len(result) == 3
    finally:
        module.stop()


def test_contributor_stops_adapter_after_render_failure(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    class Adapter:
        def __init__(self) -> None:
            self.stop_count = 0

        def start(self) -> None:
            return None

        def stop(self) -> None:
            self.stop_count += 1

        def propose_grasps(self, cloud: object) -> GraspCandidateArray:
            raise AssertionError("inference should not be reached")

    adapter = Adapter()
    monkeypatch.setattr(
        demo,
        "run_demo",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(RuntimeError("render")),
    )
    with pytest.raises(RuntimeError, match="render"):
        run_contributor_demo(
            output_path=tmp_path / "failure.png",
            module_factory=lambda _config: adapter,  # type: ignore[arg-type]
        )
    assert adapter.stop_count == 1


def test_one_config_drives_checkpoint_and_wireframe(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    base = deployment_config()
    config = base.model_copy(
        update={
            "checkpoint_path": "/custom/checkpoint",
            "gripper": base.gripper.model_copy(update={"offset_open": (0.0, 0.0, 0.2)}),
        }
    )
    captured: dict[str, object] = {}

    class Adapter:
        def start(self) -> None:
            return None

        def stop(self) -> None:
            return None

        def propose_grasps(self, cloud: object) -> GraspCandidateArray:
            raise AssertionError("replaced run_demo should handle this")

    def fake_run(proposer: object, output: Path, **kwargs: object) -> DemoResult:
        captured.update(kwargs)
        return DemoResult(output, 1, 1, 1, 1.0, "world")

    monkeypatch.setattr(demo, "run_demo", fake_run)
    run_contributor_demo(
        output_path=tmp_path / "config.png",
        config=config,
        module_factory=lambda supplied: captured.update(config=supplied) or Adapter(),
    )
    assert captured["config"] is config
    assert captured["gripper"] is config.gripper


def test_python_module_entrypoint_is_direct_and_user_visible(tmp_path: Path) -> None:
    from . import __main__

    output = tmp_path / "entrypoint.png"
    assert (
        __main__.main(
            ["--output", str(output)],
            module_factory=lambda config: GraspGenXModule(
                config,
                factory=fake_factory,
                inference=fake_inference,
            ),
        )
        == 0
    )
    assert output.read_bytes().startswith(b"\x89PNG\r\n\x1a\n")


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
