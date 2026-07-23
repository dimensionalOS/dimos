# Copyright 2026 Dimensional Inc.
"""The deterministic scene-to-YAML orchestration seam."""

from dataclasses import dataclass
from importlib import import_module
from pathlib import Path
from typing import Any, Protocol

import numpy as np

from dimos.manipulation.grasping.grasp_gen_spec import GraspGenSpec
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

from .extractor import AxisAlignedROI, ObjectPointCloudExtractor
from .fixture import fixture_roi, load_ycb_scene
from .output import write_grasps
from .visualization import Logger, log_scene

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


@dataclass(frozen=True)
class ParentCompletion:
    """Pickleable value handed to the CLI parent after worker cleanup."""

    recording_path: str | None
    rerun_open: str = "native"
    native_window_backend: str = "x11"


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
    proposer_rpc_timeout: float | None = None,
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
    if proposer_rpc_timeout is None:
        result = proposer.propose_grasps(crop)
    else:
        # RpcCall consumes this control kwarg locally; ordinary in-process
        # proposers retain the original call shape above.
        result = proposer.propose_grasps(  # type: ignore[call-arg]
            crop,
            rpc_timeout=proposer_rpc_timeout,  # type: ignore[call-arg]
        )
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
