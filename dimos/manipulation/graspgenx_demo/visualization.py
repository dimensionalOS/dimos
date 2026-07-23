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
    t = (scores - low) / (high - low or 1.0)
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
    top = list(grasps.candidates[:20])
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
        events.append(
            VisualizationEvent(
                f"grasps/{index:02d}/tcp",
                "transform",
                {"translation": translation, "rotation": rotation, "score": float(candidate.score)},
            )
        )
        # R is local→world, so each row in Arrows3D is one of R's columns.
        events.append(
            VisualizationEvent(
                f"grasps/{index:02d}/axes",
                "axes",
                {
                    "origins": np.repeat(translation[None, :], 3, axis=0),
                    "vectors": rotation.T * 0.035,
                    "colors": np.repeat(candidate_colors[index][None, :], 3, axis=0),
                    "score": float(candidate.score),
                },
            )
        )
    if top and gripper is not None:
        best = top[0]
        p, q = best.pose.position, best.pose.orientation
        translation = np.array([p.x, p.y, p.z], dtype=float)
        rotation = _rotation(q)
        for name, extents, offset in (
            ("open", gripper.extents_open, gripper.offset_open),
            ("half-open", gripper.extents_half_open, gripper.offset_half_open),
        ):
            offset_array = np.asarray(offset, dtype=float)
            events.append(
                VisualizationEvent(
                    f"grasps/00/sweep/{name}",
                    "box",
                    {
                        "center": translation + rotation @ offset_array,
                        "local_center": offset_array,
                        "rotation": rotation,
                        "sizes": np.asarray(extents, dtype=float),
                        "score": float(best.score),
                    },
                )
            )
    return tuple(events)


def log_scene(
    logger: Logger,
    raw: PointCloud2,
    crop: PointCloud2,
    grasps: GraspCandidateArray,
    gripper: SweepVolumeLike | None = None,
) -> None:
    """Log raw scene, crop, top-20 axes, and best-candidate sweep volumes."""
    points3d, transform3d, arrows3d, boxes3d = _archetypes()
    for event in build_scene_event_data(raw, crop, grasps, gripper):
        if event.kind == "points":
            try:
                points = points3d(points=event.data["points"], colors=event.data["color"])
            except TypeError:
                # Older Rerun bindings accept the point array positionally.
                points = points3d(event.data["points"], colors=event.data["color"])
            logger.log(event.path, points)
        elif event.kind == "transform":
            logger.log(
                event.path,
                transform3d(translation=event.data["translation"], mat3x3=event.data["rotation"]),
            )
        elif event.kind == "axes":
            logger.log(
                event.path,
                arrows3d(
                    origins=event.data["origins"],
                    vectors=event.data["vectors"],
                    colors=event.data["colors"],
                ),
            )
        elif event.kind == "box":
            logger.log(
                event.path.replace("grasps/00/sweep/", "grasps/00/tcp/sweep/"),
                boxes3d(
                    # Boxes3D expects a batch dimension even for one box.
                    centers=np.asarray([event.data["local_center"]]),
                    sizes=np.asarray([event.data["sizes"]]),
                ),
            )


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
            self._recording.save(str(self.partial_path))
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


def _archetypes() -> tuple[Any, Any, Any, Any]:
    rerun = import_module("rerun")
    return rerun.Points3D, rerun.Transform3D, rerun.Arrows3D, rerun.Boxes3D
