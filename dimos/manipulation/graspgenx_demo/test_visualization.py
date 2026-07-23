# Copyright 2026 Dimensional Inc.
"""Focused, dependency-light tests for the GraspGenX visualization seam."""

import os
from pathlib import Path
import pickle
from types import SimpleNamespace
from typing import Any, cast

import numpy as np

from .visualization import (
    RerunLogger,
    _score_colors,
    build_scene_event_data,
    launch_native_viewer,
    launch_web_viewer,
    log_scene,
)


class Cloud:
    frame_id = "world"
    ts = 1.0

    def __init__(self, points: list[list[float]]) -> None:
        self.points = np.asarray(points, dtype=np.float32)

    def as_numpy(self) -> tuple[np.ndarray, None]:
        return self.points, None


def _candidate(score: float, position: list[float]) -> Any:
    return SimpleNamespace(
        score=score,
        pose=SimpleNamespace(
            position=SimpleNamespace(x=position[0], y=position[1], z=position[2]),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=2**-0.5, w=2**-0.5),
        ),
    )


def _inputs() -> tuple[Cloud, Cloud, Any, Any]:
    gripper = SimpleNamespace(
        extents_open=(0.2, 0.3, 0.4),
        offset_open=(0.1, 0.0, 0.0),
        extents_half_open=(0.5, 0.6, 0.7),
        offset_half_open=(0.0, 0.2, 0.0),
    )
    candidates = SimpleNamespace(
        candidates=[_candidate(0.9, [1.0, 2.0, 3.0]), _candidate(0.4, [4.0, 5.0, 6.0])]
    )
    return Cloud([[0, 0, 0]]), Cloud([[1, 1, 1]]), candidates, gripper


def test_event_data_uses_rotation_columns_and_best_boxes_only() -> None:
    raw, crop, grasps, gripper = _inputs()
    events = build_scene_event_data(cast("Any", raw), cast("Any", crop), grasps, gripper)
    axes = next(event for event in events if event.kind == "axes")
    np.testing.assert_allclose(
        axes.data["vectors"], np.array([[0, 0.035, 0], [-0.035, 0, 0], [0, 0, 0.035]])
    )
    assert axes.data["colors"].shape == (3, 3)
    np.testing.assert_array_equal(axes.data["colors"], np.tile(axes.data["colors"][0], (3, 1)))
    second_axes = [event for event in events if event.kind == "axes"][1]
    assert not np.array_equal(axes.data["colors"][0], second_axes.data["colors"][0])
    boxes = [event for event in events if event.kind == "box"]
    assert [event.path for event in boxes] == ["grasps/00/sweep/open", "grasps/00/sweep/half-open"]
    np.testing.assert_allclose(boxes[0].data["center"], [1.0, 2.1, 3.0])
    np.testing.assert_array_equal(boxes[0].data["sizes"], [0.2, 0.3, 0.4])
    np.testing.assert_allclose(boxes[1].data["center"], [0.8, 2.0, 3.0])


def test_score_colors_are_candidate_specific_and_monotonic() -> None:
    colors = _score_colors(np.asarray([0.1, 0.5, 0.9]))
    assert len({tuple(color) for color in colors}) == 3
    assert np.all(np.diff(colors.mean(axis=1)) > 0)


def test_score_colors_are_deterministic_for_ties_and_equal_ranges() -> None:
    scores = np.asarray([0.4, 0.4, 0.4])
    colors = _score_colors(scores)
    np.testing.assert_array_equal(colors, _score_colors(scores.copy()))
    assert np.all(colors == colors[0])


def test_log_scene_uses_archetypes_without_score_keyword(monkeypatch: Any) -> None:
    raw, crop, grasps, gripper = _inputs()
    calls: list[tuple[str, Any, dict[str, Any]]] = []

    class Archetype:
        def __init__(self, **kwargs: Any) -> None:
            self.kwargs = kwargs

    monkeypatch.setattr(
        "dimos.manipulation.graspgenx_demo.visualization._archetypes",
        lambda: (Archetype, Archetype, Archetype, Archetype),
    )
    logger = SimpleNamespace(log=lambda path, value, **kwargs: calls.append((path, value, kwargs)))
    log_scene(cast("Any", logger), cast("Any", raw), cast("Any", crop), grasps, gripper)
    assert len(calls) == 2 + 2 * 2 + 2
    assert all("score" not in kwargs for _, _, kwargs in calls)
    assert [path for path, _, _ in calls[-2:]] == [
        "grasps/00/tcp/sweep/open",
        "grasps/00/tcp/sweep/half-open",
    ]


def test_recording_save_logs_flush_disconnect_and_rename_in_order(
    monkeypatch: Any, tmp_path: Any
) -> None:
    calls: list[str] = []

    class Recording:
        def __init__(self, _application_id: str) -> None:
            pass

        def save(self, path: str) -> None:
            calls.append(f"save:{path}")

        def log(self, path: str, value: Any, **kwargs: Any) -> None:
            calls.append(f"log:{path}")

        def flush(self, timeout_sec: float) -> None:
            calls.append(f"flush:{timeout_sec}")

        def disconnect(self) -> None:
            calls.append("disconnect")

    monkeypatch.setitem(
        __import__("sys").modules, "rerun", SimpleNamespace(RecordingStream=Recording)
    )
    from .visualization import recording_path_for_yaml

    final = recording_path_for_yaml(tmp_path / "foo.yaml")
    logger = RerunLogger(final)
    logger.log("scene/raw", object())

    def replace(partial: Any, destination: Any) -> None:
        calls.append("replace")
        destination.write_bytes(b"rrd")

    monkeypatch.setattr("os.replace", replace)
    logger.finalize()
    assert [call.split(":")[0] for call in calls] == [
        "save",
        "log",
        "flush",
        "disconnect",
        "replace",
    ]


def test_native_viewer_falls_back_when_spawn_setsid_is_unavailable(monkeypatch: Any) -> None:
    calls: list[dict[str, Any]] = []

    def spawn(*args: Any, **kwargs: Any) -> None:
        if "setsid" in kwargs:
            raise NotImplementedError("posix_spawn: setsid unavailable on this platform")
        calls.append({"args": args, "kwargs": kwargs})

    monkeypatch.setattr("shutil.which", lambda _name: "/usr/bin/rerun")
    monkeypatch.setattr("os.posix_spawn", spawn)
    monkeypatch.setenv("DIMOS_RUN_ID", "must-not-reach-viewer")
    assert launch_native_viewer(Path("/tmp/final.rrd"))
    assert len(calls) == 1
    assert calls[0]["args"][1] == ["/usr/bin/rerun", "/tmp/final.rrd"]
    assert calls[0]["kwargs"]["setpgroup"] == 0


def test_native_window_backend_controls_only_child_environment(monkeypatch: Any) -> None:
    calls: list[tuple[tuple[Any, ...], dict[str, Any]]] = []

    monkeypatch.setattr("shutil.which", lambda _name: "/usr/bin/rerun")
    monkeypatch.setattr("os.posix_spawn", lambda *args, **kwargs: calls.append((args, kwargs)))
    monkeypatch.setenv("DISPLAY", ":99")
    monkeypatch.setenv("WAYLAND_DISPLAY", "wayland-9")
    monkeypatch.setenv("DIMOS_RUN_ID", "worker-only")
    parent_environment = os.environ.copy()

    assert launch_native_viewer(Path("/tmp/space recording.rrd"), window_backend="x11")
    assert launch_native_viewer(Path("/tmp/space recording.rrd"), window_backend="wayland")
    assert launch_native_viewer(Path("/tmp/space recording.rrd"), window_backend="auto")

    x11_environment = calls[0][0][2]
    wayland_environment = calls[1][0][2]
    auto_environment = calls[2][0][2]
    assert x11_environment["DISPLAY"] == ":99"
    assert "WAYLAND_DISPLAY" not in x11_environment
    assert wayland_environment["WAYLAND_DISPLAY"] == "wayland-9"
    assert "DISPLAY" not in wayland_environment
    assert auto_environment["DISPLAY"] == ":99"
    assert auto_environment["WAYLAND_DISPLAY"] == "wayland-9"
    assert "DIMOS_RUN_ID" not in auto_environment
    assert dict(os.environ) == parent_environment


def test_native_window_backend_missing_display_is_nonfatal_with_exact_fallback(
    monkeypatch: Any, capsys: Any
) -> None:
    monkeypatch.setattr("shutil.which", lambda _name: "/usr/bin/rerun")
    monkeypatch.delenv("DISPLAY", raising=False)
    monkeypatch.delenv("WAYLAND_DISPLAY", raising=False)
    assert not launch_native_viewer(Path("/tmp/space recording.rrd"), window_backend="x11")
    assert "env -u WAYLAND_DISPLAY rerun '/tmp/space recording.rrd'" in capsys.readouterr().out


def test_parent_completion_is_pickleable_and_web_does_not_change_display_environment(
    monkeypatch: Any,
) -> None:
    from .demo import ParentCompletion

    completion = ParentCompletion("/tmp/final.rrd", "both", "x11")
    assert pickle.loads(pickle.dumps(completion)) == completion
    calls: list[tuple[tuple[Any, ...], dict[str, Any]]] = []
    monkeypatch.setattr("shutil.which", lambda _name: "/usr/bin/rerun")
    monkeypatch.setattr("os.posix_spawn", lambda *args, **kwargs: calls.append((args, kwargs)))
    monkeypatch.setenv("DISPLAY", ":99")
    monkeypatch.setenv("WAYLAND_DISPLAY", "wayland-9")
    assert launch_web_viewer(Path("/tmp/final.rrd"))
    environment = calls[0][0][2]
    assert environment["DISPLAY"] == ":99"
    assert environment["WAYLAND_DISPLAY"] == "wayland-9"
    assert calls[0][0][1][-1] == "--web-viewer"
