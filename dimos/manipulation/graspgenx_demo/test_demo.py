# Copyright 2026 Dimensional Inc.
"""Headless acceptance coverage for the deterministic vertical slice."""

import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np
import pytest

from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.std_msgs.Header import Header

from .demo import DEFAULT_ROI, run_demo
from .extractor import ObjectPointCloudExtractor
from .fake_backend import FakeGraspProposer
from .fixture import load_scene_record, load_ycb_scene
from .output import read_grasps, write_grasps


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


def test_ycb_demo_blueprint_wires_one_shot_runner_without_mcp() -> None:
    from dimos.core.module import OneShotModule
    from dimos.manipulation.grasping.grasp_gen_x import GraspGenXModule
    from dimos.robot.manipulators.graspgenx_ycb_demo import (
        GraspGenXYCBDemoRunner,
        graspgenx_ycb_demo,
    )

    assert issubclass(GraspGenXYCBDemoRunner, OneShotModule)
    assert "_run_once" in GraspGenXYCBDemoRunner.__dict__
    assert "run_once" not in GraspGenXYCBDemoRunner.__dict__
    assert "start" not in GraspGenXYCBDemoRunner.__dict__

    assert [atom.module for atom in graspgenx_ycb_demo.blueprints] == [
        GraspGenXModule,
        GraspGenXYCBDemoRunner,
    ]
    assert len(graspgenx_ycb_demo.blueprints[0].module_refs) == 0
    runner = graspgenx_ycb_demo.blueprints[1]
    assert [ref.spec.__name__ for ref in runner.module_refs] == ["GraspGenSpec"]
    assert not any("Mcp" in atom.module.__name__ for atom in graspgenx_ycb_demo.blueprints)


@pytest.mark.self_hosted
def test_real_graspgenx_cuda_smoke() -> None:
    """Run the real adapter on the stored banana scene when explicitly enabled.

    Opt in with ``DIMOS_GRASPGENX_GPU_SMOKE=1`` and a local
    ``DIMOS_GRASPGENX_CHECKPOINT`` containing ``gen/`` and ``dis/``.  The
    checkpoint is deliberately required locally so this test cannot download
    model assets at runtime.
    """
    checkpoint = os.environ.get("DIMOS_GRASPGENX_CHECKPOINT")
    if os.environ.get("DIMOS_GRASPGENX_GPU_SMOKE") != "1" or not checkpoint:
        pytest.skip("set DIMOS_GRASPGENX_GPU_SMOKE=1 and DIMOS_GRASPGENX_CHECKPOINT to opt in")

    checkpoint_root = Path(checkpoint).expanduser()
    if not (checkpoint_root / "gen").is_dir() or not (checkpoint_root / "dis").is_dir():
        pytest.skip("DIMOS_GRASPGENX_CHECKPOINT must contain local gen/ and dis/ directories")

    import torch  # type: ignore[import-not-found]

    if not torch.cuda.is_available():
        pytest.skip("CUDA is required for the opt-in GraspGenX smoke test")

    from dimos.manipulation.grasping.grasp_gen_x import GraspGenXModule
    from dimos.robot.manipulators.graspgenx_ycb_demo import deployment_config

    config = deployment_config()
    assert config.checkpoint_path == checkpoint
    assert config.gripper.model_dump() == {
        "extents_open": (0.08, 0.045, 0.04),
        "offset_open": (0.0, 0.0, 0.135),
        "extents_half_open": (0.04, 0.045, 0.035),
        "offset_half_open": (0.0, 0.0, 0.118),
        "fingertip_depth": 0.15,
        "family": "revolute_3f",
    }
    assert config.grasp_frame_to_tcp == (
        (1.0, 0.0, 0.0, 0.0),
        (0.0, 1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )

    scene = load_ycb_scene()
    crop = ObjectPointCloudExtractor(DEFAULT_ROI).extract(scene)
    module = GraspGenXModule(config)
    try:
        module.start()
        sampler = module._sampler
        assert sampler is not None and sampler.model is not None
        model_device = next(sampler.model.parameters()).device
        assert model_device.type == "cuda"
        result = module.propose_grasps(crop)
        assert 1 <= len(result) <= config.max_candidates
        assert result.header.frame_id == "world" == crop.frame_id
        assert result.header.timestamp == crop.ts
        print(f"GraspGenX CUDA smoke candidates={len(result)}", flush=True)
        scores = np.asarray([candidate.score for candidate in result], dtype=np.float64)
        assert np.all(np.isfinite(scores))
        assert np.all(scores[:-1] >= scores[1:])
        for candidate in result:
            pose = candidate.pose
            values = np.asarray(
                [
                    pose.x,
                    pose.y,
                    pose.z,
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ],
                dtype=np.float64,
            )
            assert np.all(np.isfinite(values))
    finally:
        try:
            module.stop()
        except Exception:
            if sys.exc_info()[0] is None:
                raise


def test_fake_backend_smoke_and_yaml_round_trip(tmp_path: Path) -> None:
    output = tmp_path / "grasps.yaml"
    result = run_demo(FakeGraspProposer(), output)
    assert result.candidate_count == 3
    value = read_grasps(output)
    assert value["frame"] == "world"
    assert [g["score"] for g in value["grasps"]] == [0.91, 0.73, 0.52]


def test_runtime_logger_receives_configured_grasp_envelopes(tmp_path: Path) -> None:
    from dimos.robot.manipulators.graspgenx_ycb_demo import deployment_config

    class Logger:
        def __init__(self) -> None:
            self.paths: list[str] = []

        def log(self, path: str, value: object, **kwargs: object) -> None:
            self.paths.append(path)

    logger = Logger()
    run_demo(
        FakeGraspProposer(),
        tmp_path / "boxes.yaml",
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


def test_runner_roi_defaults_and_validation() -> None:
    from dimos.robot.manipulators.graspgenx_ycb_demo import DemoRunnerConfig, deployment_config

    config = DemoRunnerConfig(deployment=deployment_config())
    assert config.roi_minimum == DEFAULT_ROI.minimum
    assert config.roi_maximum == DEFAULT_ROI.maximum
    assert config.proposer_rpc_timeout == 600.0
    assert config.native_window_backend == "x11"
    with pytest.raises(ValueError, match="strictly less"):
        DemoRunnerConfig(
            deployment=deployment_config(),
            roi_minimum=(0.2, 0.1, 0.1),
            roi_maximum=(0.1, 0.3, 0.2),
        )

    with pytest.raises(ValueError, match="proposer_rpc_timeout"):
        DemoRunnerConfig(deployment=deployment_config(), proposer_rpc_timeout=0)
    with pytest.raises(ValueError, match="proposer_rpc_timeout"):
        DemoRunnerConfig(deployment=deployment_config(), proposer_rpc_timeout=float("inf"))


def test_empty_result_fails_explicitly(tmp_path: Path) -> None:
    class Empty:
        def propose_grasps(self, cloud):  # type: ignore[no-untyped-def]
            from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray

            return GraspCandidateArray()

    with pytest.raises(ValueError, match="no grasp candidates"):
        run_demo(Empty(), tmp_path / "empty.yaml")  # type: ignore[arg-type]


def test_serializer_round_trips_empty_array(tmp_path: Path) -> None:
    path = tmp_path / "empty.yaml"
    write_grasps(path, GraspCandidateArray(Header(1.25, "world"), []), "identity")
    assert read_grasps(path)["grasps"] == []


def test_cli_one_shot_worker_completion_exits_zero(tmp_path: Path) -> None:
    """Exercise the foreground CLI and spontaneous worker shutdown with fake inference."""
    output = tmp_path / "worker.yaml"
    script = f"""
from typer.main import get_command
import dimos.robot.get_all_blueprints as registry
from dimos.robot.cli.dimos import main
from dimos.manipulation.graspgenx_demo.test_support import fake_blueprint
registry.get_by_name_or_exit = lambda _name: fake_blueprint({str(output)!r})
get_command(main).main(args=["--viewer", "none", "run", "graspgenx-ycb-demo"], standalone_mode=True)
"""
    env = os.environ.copy()
    env["DIMOS_VIEWER"] = "none"
    env["LCM_DEFAULT_URL"] = "udpm://239.255.76.67:17669?ttl=0"
    env["XDG_STATE_HOME"] = str(tmp_path / "state")
    stdout_path = tmp_path / "worker.stdout"
    stderr_path = tmp_path / "worker.stderr"
    stdout_file = stdout_path.open("w+b")
    stderr_file = stderr_path.open("w+b")
    process = subprocess.Popen(
        [sys.executable, "-c", script],
        env=env,
        stdout=stdout_file,
        stderr=stderr_file,
        start_new_session=True,
    )
    stdout_file.close()
    stderr_file.close()
    deadline = time.monotonic() + 30

    def captured(path: Path) -> bytes:
        return path.read_bytes() if path.exists() else b""

    def failure(reason: str) -> AssertionError:
        state = f"returncode={process.poll()!r}, args={process.args!r}"
        return AssertionError(
            f"{reason}; {state}; stdout={captured(stdout_path).decode(errors='replace')!r}; "
            f"stderr={captured(stderr_path).decode(errors='replace')!r}"
        )

    succeeded = False
    try:
        while not output.exists() and process.poll() is None and time.monotonic() < deadline:
            time.sleep(0.1)
        if not output.exists():
            process.wait(timeout=1)
            raise failure("one-shot worker did not write output")

        remaining = max(0.1, deadline - time.monotonic())
        process.wait(timeout=remaining)
        if process.returncode != 0:
            raise failure("one-shot worker exited unsuccessfully")
        value = read_grasps(output)
        assert len(value["grasps"]) == 3
        succeeded = True
    finally:
        if not succeeded:
            _cleanup_process_group(process)


@pytest.mark.parametrize(
    ("viewer", "expected_recording", "expected_native"),
    [("rerun", True, True), ("none", False, False)],
)
def test_real_cli_global_config_reaches_runner(
    tmp_path: Path, viewer: str, expected_recording: bool, expected_native: bool
) -> None:
    """Exercise CLI → blueprint → worker config, including the parent hook."""
    output = tmp_path / f"{viewer}.yaml"
    marker = tmp_path / "native-viewer.args"
    executable = tmp_path / "bin" / "rerun"
    executable.parent.mkdir()
    executable.write_text(f"#!/bin/sh\nprintf '%s\\n' \"$*\" > {marker!s}\n")
    executable.chmod(0o755)
    script = f"""
from typer.main import get_command
import dimos.robot.get_all_blueprints as registry
from dimos.robot.cli.dimos import main
from dimos.manipulation.graspgenx_demo.test_support import fake_blueprint
registry.get_by_name_or_exit = lambda _name: fake_blueprint({str(output)!r})
get_command(main).main(args=['--viewer', {viewer!r}, '--rerun-open', 'native', 'run', 'graspgenx-ycb-demo'], standalone_mode=True)
"""
    env = os.environ.copy()
    env["PATH"] = f"{executable.parent}:{env['PATH']}"
    env["LCM_DEFAULT_URL"] = "udpm://239.255.76.67:17673?ttl=0"
    env["XDG_STATE_HOME"] = str(tmp_path / "state")
    process, stdout_path, stderr_path = _spawn_file_capture(script, env, tmp_path, viewer)
    try:
        process.wait(timeout=30)
        assert process.returncode == 0, stdout_path.read_text(
            errors="replace"
        ) + stderr_path.read_text(errors="replace")
        assert output.is_file()
        assert (output.with_suffix(".rrd").is_file()) is expected_recording
        assert marker.is_file() is expected_native
        if expected_native:
            assert marker.read_text().strip().endswith(f"{output.with_suffix('.rrd')}")
    finally:
        _cleanup_process_group(process)


def _cleanup_process_group(process: subprocess.Popen[bytes]) -> None:
    """Kill and reap a failed test command and every process in its session."""
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        process.wait(timeout=2)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait(timeout=5)


def _spawn_file_capture(
    script: str, env: dict[str, str], tmp_path: Path, stem: str
) -> tuple[subprocess.Popen[bytes], Path, Path]:
    """Run a CLI child without PIPEs that descendants can inherit forever."""
    stdout_path = tmp_path / f"{stem}.stdout"
    stderr_path = tmp_path / f"{stem}.stderr"
    stdout_file = stdout_path.open("w+b")
    stderr_file = stderr_path.open("w+b")
    process = subprocess.Popen(
        [sys.executable, "-c", script],
        env=env,
        stdout=stdout_file,
        stderr=stderr_file,
        start_new_session=True,
    )
    stdout_file.close()
    stderr_file.close()
    return process, stdout_path, stderr_path


def _spawn_group_fixture(tmp_path: Path, code: str) -> tuple[subprocess.Popen[bytes], Path]:
    child_pid = tmp_path / "child.pid"
    process = subprocess.Popen(
        [sys.executable, "-c", code.format(pid_path=str(child_pid))],
        start_new_session=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    return process, child_pid


def _wait_for_pid_file(path: Path) -> int:
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        if path.exists():
            return int(path.read_text())
        time.sleep(0.02)
    raise AssertionError("fixture child did not start")


def _pid_is_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    return True


def test_failed_subprocess_cleanup_kills_quick_nonzero_descendant(tmp_path: Path) -> None:
    code = (
        "import subprocess, pathlib, sys; "
        "child=subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(30)']); "
        "pathlib.Path({pid_path!r}).write_text(str(child.pid)); sys.exit(3)"
    )
    process, pid_path = _spawn_group_fixture(tmp_path, code)
    child_pid = _wait_for_pid_file(pid_path)
    process.wait(timeout=5)
    try:
        assert process.returncode == 3
        _cleanup_process_group(process)
        deadline = time.monotonic() + 5
        while _pid_is_alive(child_pid) and time.monotonic() < deadline:
            time.sleep(0.05)
        assert not _pid_is_alive(child_pid)
    finally:
        _cleanup_process_group(process)


def test_failed_subprocess_cleanup_kills_timeout_descendant(tmp_path: Path) -> None:
    code = (
        "import subprocess, pathlib, sys, time; "
        "child=subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(30)']); "
        "pathlib.Path({pid_path!r}).write_text(str(child.pid)); time.sleep(30)"
    )
    process, pid_path = _spawn_group_fixture(tmp_path, code)
    child_pid = _wait_for_pid_file(pid_path)
    try:
        with pytest.raises(subprocess.TimeoutExpired):
            process.wait(timeout=0.1)
        _cleanup_process_group(process)
        assert not _pid_is_alive(child_pid)
    finally:
        _cleanup_process_group(process)


def test_worker_timeout_is_deterministic_and_remote_exception_is_preserved(tmp_path: Path) -> None:
    output = tmp_path / "timeout.yaml"
    script = f"""
import dimos.robot.get_all_blueprints as registry
from dimos.robot.cli.dimos import main
from dimos.manipulation.graspgenx_demo.test_support import fake_blueprint
from typer.main import get_command
registry.get_by_name_or_exit = lambda _name: fake_blueprint({str(output)!r}, proposer_rpc_timeout=0.05)
get_command(main).main(args=["--viewer", "none", "run", "graspgenx-ycb-demo"], standalone_mode=True)
"""
    env = os.environ.copy()
    env["DIMOS_GRASPGENX_FAKE_DELAY"] = "0.5"
    env["LCM_DEFAULT_URL"] = "udpm://239.255.76.67:17670?ttl=0"
    env["XDG_STATE_HOME"] = str(tmp_path / "state-timeout")
    process, stdout_path, stderr_path = _spawn_file_capture(script, env, tmp_path, "timeout")
    try:
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            _cleanup_process_group(process)
        stdout = stdout_path.read_bytes()
        stderr = stderr_path.read_bytes()
        assert process.returncode != 0
        assert stdout or stderr
        assert not output.exists()
    finally:
        _cleanup_process_group(process)

    env["DIMOS_GRASPGENX_FAKE_DELAY"] = "0"
    env["DIMOS_GRASPGENX_FAKE_FAIL"] = "1"
    process, stdout_path, stderr_path = _spawn_file_capture(script, env, tmp_path, "remote-failure")
    try:
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            _cleanup_process_group(process)
        stdout = stdout_path.read_bytes()
        stderr = stderr_path.read_bytes()
        assert process.returncode != 0
        assert stdout or stderr
    finally:
        _cleanup_process_group(process)
