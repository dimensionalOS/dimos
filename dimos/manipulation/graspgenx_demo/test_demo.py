# Copyright 2026 Dimensional Inc.
"""Headless acceptance coverage for the direct-process contributor tool."""

import os
from pathlib import Path

import numpy as np
import pytest

import dimos.manipulation.graspgenx_demo.demo as demo
from dimos.manipulation.grasping.grasp_gen_x import GraspGenXModule
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.std_msgs.Header import Header

from .demo import DEFAULT_ROI, deployment_config, run_contributor_demo, run_demo
from .extractor import ObjectPointCloudExtractor
from .fixture import load_scene_record, load_ycb_scene
from .output import read_grasps, write_grasps


class FakeGraspProposer:
    def propose_grasps(self, object_pointcloud: object) -> GraspCandidateArray:
        cloud = object_pointcloud  # type: ignore[union-attr]
        center = cloud.pointcloud.get_center()  # type: ignore[union-attr]
        candidates = [
            GraspCandidate(
                Pose(
                    {
                        "position": [float(center[0]), float(center[1]), float(center[2] + 0.16)],
                        "orientation": [0.0, 0.0, 0.0, 1.0],
                    }
                ),
                0.91,
            ),
            GraspCandidate(
                Pose(
                    {
                        "position": [
                            float(center[0] + 0.01),
                            float(center[1]),
                            float(center[2] + 0.15),
                        ],
                        "orientation": [0.0, 0.0, 0.0, 1.0],
                    }
                ),
                0.73,
            ),
            GraspCandidate(
                Pose(
                    {
                        "position": [
                            float(center[0] - 0.01),
                            float(center[1]),
                            float(center[2] + 0.14),
                        ],
                        "orientation": [0.0, 0.0, 0.0, 1.0],
                    }
                ),
                0.52,
            ),
        ]
        return GraspCandidateArray(Header(float(cloud.ts), "world"), candidates)  # type: ignore[union-attr]


def fake_factory(_config: object) -> object:
    return object()


def fake_inference(_sampler: object, _points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    poses = np.repeat(np.eye(4, dtype=np.float32)[None, ...], 3, axis=0)
    poses[:, :3, 3] = np.asarray([0.25, 0.18, 0.16])
    return poses, np.asarray([0.91, 0.73, 0.52], dtype=np.float32)


def test_recorded_scene_and_roi_are_deterministic() -> None:
    first = load_ycb_scene()
    second = load_ycb_scene()
    crop = ObjectPointCloudExtractor(DEFAULT_ROI).extract(first)
    crop_again = ObjectPointCloudExtractor(DEFAULT_ROI).extract(second)
    points, labels, metadata = load_scene_record()
    assert len(first) == 3804
    assert len(crop) == 3500
    assert metadata.get("counts") == {"banana": 3500, "table": 256, "distractor": 48, "total": 3804}
    assert metadata["labels"]["encoding"] == {"0": "banana", "1": "table", "2": "distractor"}
    assert metadata["timestamp"] == 1700000000.25
    assert first.ts == metadata["timestamp"]
    assert metadata["source"]["source_obj_sha256"] == (
        "701947c1f376efdd82de92758f80ca61e823301848efbfd9d645a62053055494"
    )
    low, high = np.asarray(DEFAULT_ROI.minimum), np.asarray(DEFAULT_ROI.maximum)
    included = np.all((points >= low) & (points <= high), axis=1)
    assert np.all(labels[included] == 0)
    assert np.all(included[labels == 0])
    assert not np.any(included[labels != 0])
    assert crop.frame_id == "world" and crop.ts == first.ts
    np.testing.assert_array_equal(crop.points_f32(), crop_again.points_f32())


@pytest.mark.self_hosted
def test_real_graspgenx_cuda_smoke() -> None:
    """Run the real adapter on the stored banana scene when explicitly enabled."""
    checkpoint = os.environ.get("DIMOS_GRASPGENX_CHECKPOINT")
    if os.environ.get("DIMOS_GRASPGENX_GPU_SMOKE") != "1" or not checkpoint:
        pytest.skip("set DIMOS_GRASPGENX_GPU_SMOKE=1 and DIMOS_GRASPGENX_CHECKPOINT to opt in")
    checkpoint_root = Path(checkpoint).expanduser()
    if not (checkpoint_root / "gen").is_dir() or not (checkpoint_root / "dis").is_dir():
        pytest.skip("DIMOS_GRASPGENX_CHECKPOINT must contain local gen/ and dis/ directories")
    import torch  # type: ignore[import-not-found]

    if not torch.cuda.is_available():
        pytest.skip("CUDA is required for the opt-in GraspGenX smoke test")
    config = deployment_config()
    module = GraspGenXModule(config)
    try:
        module.start()
        sampler = module._sampler
        assert sampler is not None and sampler.model is not None
        assert next(sampler.model.parameters()).device.type == "cuda"
        crop = ObjectPointCloudExtractor(DEFAULT_ROI).extract(load_ycb_scene())
        result = module.propose_grasps(crop)
        assert 1 <= len(result) <= config.max_candidates
        assert result.header.frame_id == crop.frame_id
        assert result.header.timestamp == crop.ts
    finally:
        module.stop()


def test_fake_backend_smoke_and_yaml_round_trip(tmp_path: Path) -> None:
    output = tmp_path / "grasps.yaml"
    result = run_demo(FakeGraspProposer(), output)
    assert result.candidate_count == 3
    value = read_grasps(output)
    assert value["frame"] == "world"
    assert [g["score"] for g in value["grasps"]] == [0.91, 0.73, 0.52]


def test_runtime_logger_receives_configured_grasp_envelopes(tmp_path: Path) -> None:
    class Logger:
        def __init__(self) -> None:
            self.paths: list[str] = []

        def log(self, path: str, value: object, **kwargs: object) -> None:
            self.paths.append(path)

    logger = Logger()
    run_demo(
        FakeGraspProposer(),
        tmp_path / "glyphs.yaml",
        cuda=False,
        device="cpu",
        gripper=deployment_config().gripper,
        logger=logger,
    )
    assert logger.paths[-3:] == [
        "grasp_candidates/rank_01",
        "grasp_candidates/rank_02",
        "grasp_candidates/rank_03",
    ]


def test_direct_adapter_config_and_cleanup(tmp_path: Path) -> None:
    config = deployment_config()
    module = GraspGenXModule(config, factory=fake_factory, inference=fake_inference)
    try:
        module.start()
        result = module.propose_grasps(
            ObjectPointCloudExtractor(DEFAULT_ROI).extract(load_ycb_scene())
        )
        assert len(result) == 3
    finally:
        module.stop()


def test_contributor_result_is_complete_only_after_recording_finalize(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    from . import demo

    final_recording = tmp_path / "final.rrd"
    finalized = False

    class Logger:
        def log(self, path: str, value: object, **kwargs: object) -> None:
            return None

        def finalize(self) -> Path:
            nonlocal finalized
            finalized = True
            final_recording.write_bytes(b"rrd")
            return final_recording

        def abort(self) -> None:
            assert finalized

    monkeypatch.setattr(demo, "RerunLogger", lambda *_args, **_kwargs: Logger())
    result = run_contributor_demo(
        output_path=tmp_path / "grasps.yaml",
        recording_path=final_recording,
        viewer="rerun",
        rerun_open="none",
        module_factory=lambda config: GraspGenXModule(
            config, factory=fake_factory, inference=fake_inference
        ),
    )

    assert finalized
    assert result.recording_path == final_recording.resolve()
    assert result.visualization_complete is True


def test_invalid_options_and_logger_setup_do_not_construct_adapter(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    constructed = 0

    def factory(_config: object) -> object:
        nonlocal constructed
        constructed += 1
        return object()

    with pytest.raises(ValueError, match="rerun_open"):
        run_contributor_demo(
            output_path=tmp_path / "invalid.yaml",
            viewer="rerun",
            rerun_open="invalid",
            module_factory=factory,  # type: ignore[arg-type]
        )
    assert constructed == 0

    def fail_logger(*_args: object, **_kwargs: object) -> object:
        raise RuntimeError("logger setup")

    monkeypatch.setattr(demo, "RerunLogger", fail_logger)
    with pytest.raises(RuntimeError, match="logger setup"):
        run_contributor_demo(
            output_path=tmp_path / "logger.yaml",
            viewer="rerun",
            rerun_open="none",
            module_factory=factory,  # type: ignore[arg-type]
        )
    assert constructed == 0


def test_failure_after_construction_stops_adapter_once(
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
        "dimos.manipulation.graspgenx_demo.demo.run_demo",
        lambda *args, **kwargs: (_ for _ in ()).throw(RuntimeError("inference")),
    )
    with pytest.raises(RuntimeError, match="inference"):
        run_contributor_demo(
            output_path=tmp_path / "failure.yaml",
            viewer="none",
            module_factory=lambda _config: adapter,  # type: ignore[arg-type]
        )
    assert adapter.stop_count == 1


def test_one_config_drives_checkpoint_calibration_and_gripper(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    base = deployment_config()
    config = base.model_copy(
        update={
            "checkpoint_path": "/custom/checkpoint",
            "grasp_frame_to_tcp": (
                (0.0, -1.0, 0.0, 0.01),
                (1.0, 0.0, 0.0, 0.02),
                (0.0, 0.0, 1.0, 0.03),
                (0.0, 0.0, 0.0, 1.0),
            ),
            "gripper": base.gripper.model_copy(update={"offset_open": (0.0, 0.0, 0.2)}),
        }
    )
    captured: dict[str, object] = {}

    class Adapter:
        def start(self) -> None: ...

        def stop(self) -> None: ...

        def propose_grasps(self, cloud: object) -> GraspCandidateArray: ...

    def fake_run(proposer: object, output: Path, **kwargs: object) -> demo.DemoResult:
        captured.update(kwargs)
        return demo.DemoResult(output, 1, 1, 1, 1.0, "world")

    monkeypatch.setattr(demo, "run_demo", fake_run)
    run_contributor_demo(
        output_path=tmp_path / "config.yaml",
        viewer="none",
        config=config,
        module_factory=lambda supplied: captured.update(config=supplied) or Adapter(),  # type: ignore[arg-type]
    )
    assert captured["config"] is config
    assert captured["checkpoint"] == config.checkpoint_path
    assert captured["tcp_calibration"] == config.grasp_frame_to_tcp
    assert captured["gripper"] is config.gripper


def test_python_module_entrypoint_is_direct_and_user_visible(tmp_path: Path) -> None:
    from . import __main__

    output = tmp_path / "entrypoint.yaml"
    assert (
        __main__.main(
            ["--viewer", "none", "--output", str(output)],
            module_factory=lambda config: GraspGenXModule(
                config, factory=fake_factory, inference=fake_inference
            ),
        )
        == 0
    )
    assert read_grasps(output)["grasps"]
    assert not output.with_suffix(".rrd").exists()


def test_empty_result_fails_explicitly(tmp_path: Path) -> None:
    class Empty:
        def propose_grasps(self, cloud: object) -> GraspCandidateArray:
            return GraspCandidateArray()

    with pytest.raises(ValueError, match="no grasp candidates"):
        run_demo(Empty(), tmp_path / "empty.yaml")  # type: ignore[arg-type]


def test_serializer_round_trips_empty_array(tmp_path: Path) -> None:
    path = tmp_path / "empty.yaml"
    write_grasps(path, GraspCandidateArray(Header(1.25, "world"), []), "identity")
    assert read_grasps(path)["grasps"] == []
