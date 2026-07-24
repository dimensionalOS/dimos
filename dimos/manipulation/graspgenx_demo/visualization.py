# Copyright 2026 Dimensional Inc.
"""Dependency-light event construction and lazy, repo-standard Rerun logging."""

from __future__ import annotations

from dataclasses import dataclass
import errno
from importlib import import_module
import os
from pathlib import Path
import shlex
import shutil
from typing import Any, Protocol

import numpy as np

from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class Logger(Protocol):
    def log(self, path: str, value: Any, **kwargs: Any) -> None: ...


class SweepVolumeLike(Protocol):
    """The geometry needed here, kept structural to avoid an import cycle."""

    extents_open: tuple[float, float, float]
    offset_open: tuple[float, float, float]
    extents_half_open: tuple[float, float, float]
    offset_half_open: tuple[float, float, float]


FORK_STEM_INDEX = 0
FORK_BRIDGE_INDEX = 1
FORK_LEFT_RAIL_INDEX = 2
FORK_RIGHT_RAIL_INDEX = 3
FORK_STRIP_COUNT = 4
FORK_RADIUS = 0.003


@dataclass(frozen=True)
class VisualizationEvent:
    path: str
    kind: str
    data: dict[str, Any]


def _rotation(q: Any) -> np.ndarray:
    x, y, z, w = (float(q.x), float(q.y), float(q.z), float(q.w))
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def _score_colors(scores: np.ndarray) -> np.ndarray:
    """Map the displayed score vector to color-blind-safe, salient RGB colors."""
    if not len(scores):
        return np.empty((0, 3), dtype=np.uint8)
    low, high = float(scores.min()), float(scores.max())
    if high == low:
        # Candidates are already rank-ordered; preserve rank as the tie-breaker
        # so rank_01 remains the highest-score glyph in the displayed scene.
        t = np.linspace(1.0, 0.0, len(scores))
    else:
        t = (scores - low) / (high - low)
    # Viridis is color-blind safe and increases in perceived lightness from
    # indigo to yellow. Piecewise interpolation keeps that intended ordering
    # while producing deterministic 8-bit colors without extra dependencies.
    palette = np.asarray(
        [[68, 1, 84], [59, 82, 139], [33, 145, 140], [94, 201, 98], [253, 231, 37]],
        dtype=float,
    )
    positions = np.linspace(0.0, 1.0, len(palette))
    colors = np.column_stack([np.interp(t, positions, palette[:, channel]) for channel in range(3)])
    return np.rint(colors).astype(np.uint8)


def build_scene_event_data(
    raw: PointCloud2,
    crop: PointCloud2,
    grasps: GraspCandidateArray,
    gripper: SweepVolumeLike | None = None,
) -> tuple[VisualizationEvent, ...]:
    """Build deterministic world-space events without importing Rerun."""
    raw_points, _ = raw.as_numpy()
    crop_points, _ = crop.as_numpy()
    top = list(grasps.candidates[:5])
    scores = np.asarray([float(candidate.score) for candidate in top], dtype=float)
    candidate_colors = _score_colors(scores)
    events: list[VisualizationEvent] = [
        VisualizationEvent("scene/raw", "points", {"points": raw_points, "color": [130, 136, 140]}),
        VisualizationEvent(
            "scene/banana", "points", {"points": crop_points, "color": [245, 191, 31]}
        ),
    ]
    for index, candidate in enumerate(top):
        p, q = candidate.pose.position, candidate.pose.orientation
        translation = np.array([p.x, p.y, p.z], dtype=float)
        rotation = _rotation(q)
        rotation[np.abs(rotation) < 1e-15] = 0.0
        if gripper is None:
            continue
        color = (
            int(candidate_colors[index][0]),
            int(candidate_colors[index][1]),
            int(candidate_colors[index][2]),
            255,
        )
        events.append(
            VisualizationEvent(
                f"grasp_candidates/rank_{index + 1:02d}",
                "glyph",
                {
                    "strips": _fork_strips(gripper, translation, rotation),
                    "color": color,
                    "radii": np.full(FORK_STRIP_COUNT, FORK_RADIUS, dtype=np.float32),
                    "colors": np.repeat(
                        np.asarray(color, dtype=np.uint8)[None, :], FORK_STRIP_COUNT, axis=0
                    ),
                    "score": float(candidate.score),
                },
            )
        )
    return tuple(events)


def _fork_strips_local(gripper: SweepVolumeLike) -> tuple[np.ndarray, ...]:
    """Build the abstract fork in its candidate-local X-Z profile plane.

    The profile is deliberately not a 3D gripper model: X is the opening
    span, Z is the mouth direction, and Y is exactly zero everywhere. The TCP
    itself is local origin, while configured sweep offsets locate the rear and
    open ends of the proposal.
    """
    open_extents = np.asarray(gripper.extents_open, dtype=np.float64)
    half_extents = np.asarray(gripper.extents_half_open, dtype=np.float64)
    open_offset = np.asarray(gripper.offset_open, dtype=np.float64)
    half_offset = np.asarray(gripper.offset_half_open, dtype=np.float64)

    rear_center = np.array([half_offset[0], 0.0, half_offset[2] - half_extents[2] / 2.0])
    open_center = np.array([open_offset[0], 0.0, open_offset[2] + open_extents[2] / 2.0])
    if open_center[2] <= rear_center[2]:
        raise ValueError("configured sweep profiles must open toward increasing local +Z")

    rear_half_width = max(float(half_extents[0]) / 2.0, 1e-6)
    open_half_width = max(float(open_extents[0]) / 2.0, rear_half_width)
    rear_left = rear_center + np.array([-rear_half_width, 0.0, 0.0])
    rear_right = rear_center + np.array([rear_half_width, 0.0, 0.0])
    open_left = open_center + np.array([-open_half_width, 0.0, 0.0])
    open_right = open_center + np.array([open_half_width, 0.0, 0.0])

    return (
        np.asarray([rear_center, [0.0, 0.0, 0.0]], dtype=np.float64),
        np.asarray([rear_left, rear_right], dtype=np.float64),
        np.asarray([rear_left, open_left], dtype=np.float64),
        np.asarray([rear_right, open_right], dtype=np.float64),
    )


def _fork_strips(
    gripper: SweepVolumeLike, translation: np.ndarray, rotation: np.ndarray
) -> tuple[np.ndarray, ...]:
    """Transform the planar fork by the candidate's complete rigid pose."""
    local_strips = _fork_strips_local(gripper)
    return tuple(
        np.asarray((rotation @ strip.T).T + translation, dtype=np.float32) for strip in local_strips
    )


def log_scene(
    logger: Logger,
    raw: PointCloud2,
    crop: PointCloud2,
    grasps: GraspCandidateArray,
    gripper: SweepVolumeLike | None = None,
) -> None:
    """Log the scene and one abstract planar fork per top-five candidate."""
    points3d, line_strips3d = _archetypes()
    for event in build_scene_event_data(raw, crop, grasps, gripper):
        if event.kind == "points":
            try:
                points = points3d(points=event.data["points"], colors=event.data["color"])
            except TypeError:
                # Older Rerun bindings accept the point array positionally.
                points = points3d(event.data["points"], colors=event.data["color"])
            logger.log(event.path, points)
        elif event.kind == "glyph":
            kwargs = {
                "strips": event.data["strips"],
                "colors": event.data["colors"],
                "radii": event.data["radii"],
            }
            try:
                glyph = line_strips3d(**kwargs)
            except TypeError:
                kwargs.pop("radii")
                glyph = line_strips3d(**kwargs)
            logger.log(event.path, glyph)


class RerunLogger:
    """Write a complete, replayable Rerun 0.32 recording without a live viewer."""

    def __init__(self, final_path: Path, flush_timeout: float = 10.0) -> None:
        if flush_timeout <= 0 or not np.isfinite(flush_timeout):
            raise ValueError("Rerun flush timeout must be finite and positive")
        self.final_path = final_path.expanduser().resolve()
        self.partial_path = Path(f"{self.final_path}.partial").resolve()
        if self.partial_path == self.final_path:
            raise ValueError("Rerun final and partial paths must be distinct")
        self.partial_path.unlink(missing_ok=True)
        self.final_path.parent.mkdir(parents=True, exist_ok=True)
        self._disconnected = False
        self._state = "opening"
        try:
            rerun = import_module("rerun")
            self._recording = rerun.RecordingStream("graspgenx-ycb-demo")
            # This must precede every event, including the first archetype construction.
            self._state = "saving"
            self._recording.save(str(self.partial_path), default_blueprint=_default_blueprint())
        except Exception:
            try:
                self.partial_path.unlink(missing_ok=True)
            except OSError:
                pass
            raise
        self._flush_timeout = float(flush_timeout)
        self._state = "open"

    def log(self, path: str, value: Any, **kwargs: Any) -> None:
        if self._state != "open":
            raise RuntimeError(f"cannot log to Rerun recording in state {self._state}")
        self._recording.log(path, value, **kwargs)

    def finalize(self) -> Path:
        """Flush, disconnect, then publish the recording atomically."""
        if self._state == "published":
            return self.final_path
        if self._state != "open":
            raise RuntimeError(f"cannot finalize Rerun recording in state {self._state}")
        previous: Path | None = None
        try:
            self._state = "flushing"
            self._recording.flush(timeout_sec=self._flush_timeout)
            self._state = "disconnecting"
            self._recording.disconnect()
            self._state = "renaming"
            if self.final_path.exists():
                previous = self.final_path.with_name(f".{self.final_path.name}.previous")
                previous.unlink(missing_ok=True)
                shutil.copy2(self.final_path, previous)
            os.replace(self.partial_path, self.final_path)
            if not self.final_path.is_file():
                raise RuntimeError(f"Rerun recording was not created: {self.final_path}")
            self._state = "published"
            if previous is not None:
                try:
                    previous.unlink(missing_ok=True)
                except OSError:
                    pass
            return self.final_path
        except Exception:
            if previous is not None and previous.is_file():
                try:
                    os.replace(previous, self.final_path)
                except OSError:
                    pass
            elif self._state == "renaming":
                try:
                    self.final_path.unlink(missing_ok=True)
                except OSError:
                    pass
            self.abort()
            raise

    def _close_best_effort(self) -> None:
        if self._disconnected:
            return
        self._disconnected = True
        try:
            self._recording.disconnect()
        except Exception:
            pass

    def abort(self) -> None:
        """Close a failed recording without publishing a partial or final file."""
        if self._state == "published":
            return
        self._close_best_effort()
        try:
            self.partial_path.unlink(missing_ok=True)
        except OSError:
            pass
        self._state = "aborted"

    def __enter__(self) -> RerunLogger:
        return self

    def __exit__(self, _type: Any, _value: Any, _traceback: Any) -> None:
        if self._state != "published":
            self.abort()


def recording_path_for_yaml(output_path: Path, recording_path: Path | None = None) -> Path:
    """Resolve the configured absolute RRD destination for a YAML result."""
    yaml_path = output_path.expanduser().resolve()
    final = (
        recording_path.expanduser().resolve()
        if recording_path is not None
        else yaml_path.with_suffix(".rrd")
    )
    partial = Path(f"{final}.partial").resolve()
    if len({yaml_path, final, partial}) != 3:
        raise ValueError(
            "YAML, finalized RRD, and RRD partial paths must be distinct; "
            f"got yaml={yaml_path}, final={final}, partial={partial}"
        )
    return final


def launch_native_viewer(recording_path: Path, *, window_backend: str = "auto") -> bool:
    """Launch native Rerun with an explicit, child-only display backend choice."""
    if window_backend not in {"x11", "wayland", "auto"}:
        raise ValueError(f"unsupported native window backend: {window_backend!r}")
    return _launch_viewer(recording_path, web=False, window_backend=window_backend)


def launch_web_viewer(recording_path: Path) -> bool:
    """Open a finalized recording in Rerun's 0.32 web viewer."""
    return _launch_viewer(recording_path, web=True, window_backend="auto")


def _launch_viewer(recording_path: Path, *, web: bool, window_backend: str) -> bool:
    executable: str | None = None
    null_fd: int | None = None
    command_args = ["rerun", str(recording_path)] + (["--web-viewer"] if web else [])
    manual_args = command_args
    if not web and window_backend == "x11":
        manual_args = ["env", "-u", "WAYLAND_DISPLAY", *command_args]
    elif not web and window_backend == "wayland":
        manual_args = ["env", "-u", "DISPLAY", *command_args]
    command = shlex.join(manual_args)
    try:
        executable = shutil.which("rerun")
    except Exception as exc:
        print(
            f"graspgenx-ycb-demo viewer setup failed ({exc!r}); open manually: {command}",
            flush=True,
        )
        return False
    if executable is None:
        print(f"graspgenx-ycb-demo viewer unavailable; open manually: {command}", flush=True)
        return False
    try:
        environment = os.environ.copy()
        environment.pop("DIMOS_RUN_ID", None)
        if not web and window_backend == "x11":
            if not environment.get("DISPLAY"):
                print(
                    f"graspgenx-ycb-demo viewer requires DISPLAY; open manually: {command}",
                    flush=True,
                )
                return False
            environment.pop("WAYLAND_DISPLAY", None)
        elif not web and window_backend == "wayland":
            if not environment.get("WAYLAND_DISPLAY"):
                print(
                    f"graspgenx-ycb-demo viewer requires WAYLAND_DISPLAY; open manually: {command}",
                    flush=True,
                )
                return False
            environment.pop("DISPLAY", None)
        null_fd = os.open(os.devnull, os.O_RDWR)
        actions: list[tuple[int, ...]] = [
            (os.POSIX_SPAWN_DUP2, null_fd, 0),
            (os.POSIX_SPAWN_DUP2, null_fd, 1),
            (os.POSIX_SPAWN_DUP2, null_fd, 2),
        ]
        # posix_spawn has no close_fds flag. Explicitly close every inherited
        # descriptor so the native viewer cannot retain worker/coordinator pipes.
        for entry in os.listdir("/proc/self/fd"):
            descriptor = int(entry)
            if descriptor > 2 and descriptor != null_fd:
                actions.append((os.POSIX_SPAWN_CLOSE, descriptor))
        actions.append((os.POSIX_SPAWN_CLOSE, null_fd))
        spawn_args = (executable, [executable, str(recording_path)], environment)
        if web:
            spawn_args = (executable, [executable, *command_args[1:]], environment)
        try:
            os.posix_spawn(*spawn_args, file_actions=actions, setsid=True)
        except (OSError, NotImplementedError, TypeError) as exc:
            # Some Linux/Python builds expose the keyword but do not support
            # POSIX_SPAWN_SETSID at runtime.  A private process group still
            # lets the viewer outlive the CLI without inheriting its group.
            if not isinstance(exc, (NotImplementedError, TypeError)) and getattr(
                exc, "errno", None
            ) not in {
                errno.ENOSYS,
                errno.EINVAL,
                errno.ENOTSUP,
            }:
                raise
            os.posix_spawn(*spawn_args, file_actions=actions, setpgroup=0)
        return True
    except (OSError, TypeError, NotImplementedError, ValueError) as exc:
        print(
            f"graspgenx-ycb-demo viewer launch failed ({exc}); open manually: {command}",
            flush=True,
        )
        return False
    finally:
        if null_fd is not None:
            try:
                os.close(null_fd)
            except Exception:
                pass


def _default_blueprint() -> Any:
    rerun_blueprint = import_module("rerun.blueprint")
    return rerun_blueprint.Blueprint(
        rerun_blueprint.Spatial3DView(
            origin="/",
            contents=["/scene/**", "/grasp_candidates/rank_01"],
            name="Scene and Best Grasp",
        )
    )


def _archetypes() -> tuple[Any, Any]:
    rerun = import_module("rerun")
    return rerun.Points3D, rerun.LineStrips3D
