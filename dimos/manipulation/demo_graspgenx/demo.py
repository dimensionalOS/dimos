# Copyright 2026 Dimensional Inc.
"""The deterministic scene-to-YAML orchestration seam."""

from collections.abc import Callable
from dataclasses import dataclass, replace
from importlib import import_module
import math
import os
from pathlib import Path
from typing import Any, Protocol

import numpy as np

from dimos.manipulation.grasping.grasp_gen_spec import GraspGenSpec
from dimos.manipulation.grasping.grasp_gen_x import (
    GraspGenXConfig,
    GraspGenXModule,
    SweepVolumeGripperConfig,
)
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

from .extractor import AxisAlignedROI, ObjectPointCloudExtractor
from .fixture import fixture_roi, load_ycb_scene
from .output import write_grasps
from .visualization import (
    Logger,
    RerunLogger,
    launch_native_viewer,
    launch_web_viewer,
    log_scene,
    recording_path_for_yaml,
)

DEFAULT_ROI = fixture_roi()
IDENTITY_CALIBRATION = (
    (1.0, 0.0, 0.0, 0.0),
    (0.0, 1.0, 0.0, 0.0),
    (0.0, 0.0, 1.0, 0.0),
    (0.0, 0.0, 0.0, 1.0),
)


class Proposer(Protocol):
    def propose_grasps(self, object_pointcloud: PointCloud2) -> GraspCandidateArray: ...


@dataclass(frozen=True)
class DemoResult:
    output_path: Path
    scene_points: int
    object_points: int
    candidate_count: int
    best_score: float
    frame: str
    recording_path: Path | None = None
    visualization_complete: bool = False
    visualization_error: str | None = None


def deployment_config() -> GraspGenXConfig:
    """Build the fixed sweep-volume deployment without loading the checkpoint."""
    return GraspGenXConfig(
        checkpoint_path=os.environ.get(
            "DIMOS_GRASPGENX_CHECKPOINT", "/__missing_graspgenx_checkpoint__"
        ),
        gripper=SweepVolumeGripperConfig(
            extents_open=(0.08, 0.045, 0.04),
            offset_open=(0.0, 0.0, 0.135),
            extents_half_open=(0.04, 0.045, 0.035),
            offset_half_open=(0.0, 0.0, 0.118),
            fingertip_depth=0.15,
            family="revolute_3f",
        ),
        max_candidates=100,
    )


def _cuda_context() -> dict[str, object]:
    """Inspect the deployment runtime without making torch a demo dependency."""
    try:
        torch = import_module("torch")
    except ImportError:
        return {"available": False, "device": "unavailable (torch not installed)"}
    available = bool(torch.cuda.is_available())
    return {"available": available, "device": torch.cuda.get_device_name(0) if available else "cpu"}


def _validate(result: GraspCandidateArray) -> None:
    if not result.candidates:
        raise ValueError("proposer returned no grasp candidates")
    if result.header.frame_id != "world":
        raise ValueError("proposer returned a result outside world frame")
    scores = np.asarray([c.score for c in result.candidates], dtype=float)
    if not np.all(np.isfinite(scores)) or np.any(scores[:-1] < scores[1:]):
        raise ValueError("grasp scores must be finite and descending")
    for candidate in result.candidates:
        values = np.asarray(
            [
                candidate.pose.x,
                candidate.pose.y,
                candidate.pose.z,
                candidate.pose.orientation.x,
                candidate.pose.orientation.y,
                candidate.pose.orientation.z,
                candidate.pose.orientation.w,
            ],
            dtype=float,
        )
        if not np.all(np.isfinite(values)):
            raise ValueError("proposer returned a non-finite TCP pose")


def run_demo(
    proposer: GraspGenSpec,
    output_path: Path,
    *,
    roi: AxisAlignedROI = DEFAULT_ROI,
    checkpoint: str | None = None,
    device: str = "unspecified",
    cuda: bool | str | None = None,
    tcp_calibration: object = IDENTITY_CALIBRATION,
    gripper: Any | None = None,
    logger: Logger | None = None,
) -> DemoResult:
    """Run scene → crop → generic GraspGenSpec → deterministic YAML."""
    if not callable(getattr(proposer, "propose_grasps", None)):
        raise TypeError("proposer must implement GraspGenSpec.propose_grasps")
    scene = load_ycb_scene()
    crop = ObjectPointCloudExtractor(roi).extract(scene)
    runtime = _cuda_context() if cuda is None else {"available": cuda, "device": device}
    context = {
        "checkpoint": checkpoint or "unspecified",
        "device": runtime["device"],
        "cuda": runtime,
    }
    print(f"graspgenx-ycb-demo deployment={context}", flush=True)
    result = proposer.propose_grasps(crop)
    print(
        "graspgenx-ycb-demo inference "
        f"candidates={len(result)} "
        f"best_score={result.candidates[0].score if result.candidates else None}",
        flush=True,
    )
    _validate(result)
    if result.header.frame_id != crop.frame_id or result.header.timestamp != crop.ts:
        raise ValueError("proposer changed the crop frame or timestamp")
    write_grasps(output_path, result, tcp_calibration)
    recording_path: Path | None = None
    visualization_complete = logger is None
    visualization_error: str | None = None
    if logger is not None:
        # The runner owns flush/disconnect/rename. Logging errors must escape so
        # its finally block can abort the recording and preserve any old RRD.
        log_scene(logger, scene, crop, result, gripper)
    diagnostics: dict[str, Any] = {
        "checkpoint_root": checkpoint or "unspecified",
        "cuda": context["cuda"],
        "device": runtime["device"],
        "model": type(proposer).__name__,
        "scene_points": len(scene),
        "object_points": len(crop),
        "candidates": len(result),
        "best_score": result.candidates[0].score,
        "world_frame": result.header.frame_id,
        "tcp_calibration": tcp_calibration,
        "output": str(output_path),
        "recording": str(recording_path) if recording_path is not None else None,
        "visualization_complete": visualization_complete,
        "visualization_error": visualization_error,
    }
    print(
        "graspgenx-ycb-demo " + " ".join(f"{key}={value}" for key, value in diagnostics.items()),
        flush=True,
    )
    return DemoResult(
        output_path,
        len(scene),
        len(crop),
        len(result),
        result.candidates[0].score,
        result.header.frame_id,
        recording_path,
        visualization_complete,
        visualization_error,
    )


def run_contributor_demo(
    *,
    output_path: Path,
    recording_path: Path | None = None,
    viewer: str = "rerun",
    rerun_open: str = "native",
    native_window_backend: str = "x11",
    flush_timeout: float = 10.0,
    config: GraspGenXConfig | None = None,
    module_factory: Callable[[GraspGenXConfig], GraspGenXModule] = GraspGenXModule,
) -> DemoResult:
    """Run the real adapter directly, then publish and optionally open results."""
    if viewer not in {"rerun", "none"}:
        raise ValueError("viewer must be 'rerun' or 'none'")
    if rerun_open not in {"none", "native", "web", "both"}:
        raise ValueError("rerun_open must be none, native, web, or both")
    if native_window_backend not in {"x11", "wayland", "auto"}:
        raise ValueError("native_window_backend must be x11, wayland, or auto")
    if not math.isfinite(flush_timeout) or flush_timeout <= 0:
        raise ValueError("flush_timeout must be finite and positive")
    if not callable(module_factory):
        raise TypeError("module_factory must be callable")
    output_path = output_path.expanduser().resolve()
    final_recording = recording_path_for_yaml(output_path, recording_path)
    active_config = config if config is not None else deployment_config()
    logger = (
        RerunLogger(final_recording, flush_timeout=flush_timeout) if viewer == "rerun" else None
    )
    adapter: GraspGenXModule | None = None
    result: DemoResult | None = None
    recording: Path | None = None
    primary_error: BaseException | None = None
    try:
        adapter = module_factory(active_config)
        adapter.start()
        result = run_demo(
            adapter,
            output_path,
            roi=DEFAULT_ROI,
            checkpoint=active_config.checkpoint_path,
            device=os.environ.get("DIMOS_GRASPGENX_DEVICE", "cuda"),
            cuda=None,
            tcp_calibration=active_config.grasp_frame_to_tcp,
            gripper=active_config.gripper,
            logger=logger,
        )
        if logger is not None:
            recording = logger.finalize()
    except BaseException as exc:
        primary_error = exc
        raise
    finally:
        cleanup_error: Exception | None = None
        try:
            if logger is not None:
                logger.abort()
        except Exception as exc:
            cleanup_error = exc
        try:
            if adapter is not None:
                adapter.stop()
        except Exception:
            if primary_error is None and cleanup_error is None:
                raise
        if primary_error is None and cleanup_error is not None:
            raise cleanup_error
    assert result is not None
    result = replace(
        result,
        recording_path=recording,
        visualization_complete=logger is None or recording is not None,
    )
    if recording is not None and rerun_open != "none":
        if rerun_open in {"native", "both"}:
            launch_native_viewer(recording, window_backend=native_window_backend)
        if rerun_open in {"web", "both"}:
            launch_web_viewer(recording)
    return result
