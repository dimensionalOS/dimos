# Copyright 2026 Dimensional Inc.
"""Focused, dependency-light tests for the GraspGenX visualization seam."""

import os
from pathlib import Path
from types import SimpleNamespace
from typing import Any, cast

import numpy as np

from .visualization import (
    FORK_BRIDGE_INDEX,
    FORK_LEFT_RAIL_INDEX,
    FORK_RADIUS,
    FORK_RIGHT_RAIL_INDEX,
    FORK_STEM_INDEX,
    FORK_STRIP_COUNT,
    RerunLogger,
    _default_blueprint,
    _fork_strips_local,
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


def _candidate(
    score: float,
    position: list[float],
    orientation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
) -> Any:
    return SimpleNamespace(
        score=score,
        pose=SimpleNamespace(
            position=SimpleNamespace(x=position[0], y=position[1], z=position[2]),
            orientation=SimpleNamespace(
                x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3]
            ),
        ),
    )


def _inputs() -> tuple[Cloud, Cloud, Any, Any]:
    gripper = SimpleNamespace(
        extents_open=(0.2, 0.3, 0.4),
        offset_open=(0.1, 0.0, 0.0),
        extents_half_open=(0.1, 0.15, 0.2),
        offset_half_open=(0.0, 0.2, 0.0),
    )
    candidates = SimpleNamespace(
        candidates=[
            _candidate(0.9, [1.0, 2.0, 3.0]),
            _candidate(0.4, [4.0, 5.0, 6.0], (2**-0.5, 0.0, 0.0, 2**-0.5)),
        ]
    )
    return Cloud([[0, 0, 0]]), Cloud([[1, 1, 1]]), candidates, gripper


def test_event_data_builds_five_planar_forks_with_rigid_3d_transforms() -> None:
    raw, crop, grasps, gripper = _inputs()
    grasps.candidates.extend(
        _candidate(score, [7.0, 8.0, 9.0]) for score in (0.3, 0.2, 0.1, 0.0, -0.1)
    )
    events = build_scene_event_data(cast("Any", raw), cast("Any", crop), grasps, gripper)
    glyphs = [event for event in events if event.kind == "glyph"]
    assert [event.path for event in glyphs] == [
        f"grasp_candidates/rank_{rank:02d}" for rank in range(1, 6)
    ]
    assert all(len(event.data["strips"]) == FORK_STRIP_COUNT for event in glyphs)
    assert all(event.data["strips"][FORK_STEM_INDEX].shape == (2, 3) for event in glyphs)
    assert all(len(event.data["radii"]) == FORK_STRIP_COUNT for event in glyphs)
    assert all(np.all(event.data["radii"] == FORK_RADIUS) for event in glyphs)
    assert all(
        np.all(event.data["colors"] == np.asarray(event.data["color"], dtype=np.uint8))
        for event in glyphs
    )
    assert all(event.data["color"][3] == 255 for event in glyphs)
    assert len({event.data["color"] for event in glyphs}) == 5
    assert all(event.kind not in {"mesh", "axes", "box", "transform"} for event in events)
    strips = glyphs[0].data["strips"]
    local = _fork_strips_local(gripper)
    assert all(np.allclose(strip[:, 1], 0.0) for strip in local)
    rear_width = abs(local[FORK_BRIDGE_INDEX][1, 0] - local[FORK_BRIDGE_INDEX][0, 0])
    mouth_width = abs(local[FORK_RIGHT_RAIL_INDEX][-1, 0] - local[FORK_LEFT_RAIL_INDEX][-1, 0])
    assert mouth_width > rear_width
    assert all(
        strips[index][1, 2] > strips[index][0, 2]
        for index in (FORK_LEFT_RAIL_INDEX, FORK_RIGHT_RAIL_INDEX)
    )
    assert np.allclose(local[FORK_STEM_INDEX][0], local[FORK_BRIDGE_INDEX].mean(axis=0))
    assert np.allclose(local[FORK_STEM_INDEX][1], [0.0, 0.0, 0.0])
    # The second candidate rotates the local X-Z plane into world X-Y.
    rotation_x_90 = np.asarray([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]])
    for first_strip, second_strip in zip(
        glyphs[0].data["strips"], glyphs[1].data["strips"], strict=True
    ):
        local_strip = first_strip - [1.0, 2.0, 3.0]
        expected = (rotation_x_90 @ local_strip.T).T + np.asarray([4.0, 5.0, 6.0])
        np.testing.assert_allclose(second_strip, expected, rtol=0.0, atol=1e-6)
    assert np.ptp(glyphs[1].data["strips"][FORK_LEFT_RAIL_INDEX][:, 1]) > 0


def test_score_colors_are_candidate_specific_and_monotonic() -> None:
    colors = _score_colors(np.asarray([0.1, 0.5, 0.9]))
    assert len({tuple(color) for color in colors}) == 3
    assert np.all(np.diff(colors.mean(axis=1)) > 0)


def test_highest_rank_is_viridis_yellow_when_scores_tie() -> None:
    scores = np.asarray([0.4, 0.4, 0.4])
    colors = _score_colors(scores)
    np.testing.assert_array_equal(colors, _score_colors(scores.copy()))
    assert tuple(colors[0]) == (253, 231, 37)
    assert len({tuple(color) for color in colors}) == len(scores)


def test_default_blueprint_shows_scene_and_rank_one_only() -> None:
    view = _default_blueprint().root_container.contents[0]
    assert view.name == "Scene and Best Grasp"
    assert view.origin == "/"
    assert view.contents == ["/scene/**", "/grasp_candidates/rank_01"]


def test_log_scene_uses_archetypes_without_score_keyword(monkeypatch: Any) -> None:
    raw, crop, grasps, gripper = _inputs()
    calls: list[tuple[str, Any, dict[str, Any]]] = []

    class Archetype:
        def __init__(self, **kwargs: Any) -> None:
            self.kwargs = kwargs

    monkeypatch.setattr(
        "dimos.manipulation.graspgenx_demo.visualization._archetypes",
        lambda: (Archetype, Archetype),
    )
    logger = SimpleNamespace(log=lambda path, value, **kwargs: calls.append((path, value, kwargs)))
    log_scene(cast("Any", logger), cast("Any", raw), cast("Any", crop), grasps, gripper)
    assert len(calls) == 2 + 2
    assert all("score" not in kwargs for _, _, kwargs in calls)
    assert all("strips" in value.kwargs and "colors" in value.kwargs for _, value, _ in calls[-2:])
    assert [path for path, _, _ in calls[-2:]] == [
        "grasp_candidates/rank_01",
        "grasp_candidates/rank_02",
    ]


def test_recording_save_logs_flush_disconnect_and_rename_in_order(
    monkeypatch: Any, tmp_path: Any
) -> None:
    calls: list[str] = []

    class Recording:
        def __init__(self, _application_id: str) -> None:
            pass

        def save(self, path: str, **kwargs: Any) -> None:
            calls.append(f"save:{path}")
            assert "default_blueprint" in kwargs

        def log(self, path: str, value: Any, **kwargs: Any) -> None:
            calls.append(f"log:{path}")

        def flush(self, timeout_sec: float) -> None:
            calls.append(f"flush:{timeout_sec}")

        def disconnect(self) -> None:
            calls.append("disconnect")

    monkeypatch.setitem(
        __import__("sys").modules, "rerun", SimpleNamespace(RecordingStream=Recording)
    )
    monkeypatch.setattr(
        "dimos.manipulation.graspgenx_demo.visualization._default_blueprint",
        lambda: "scene-and-best-grasp",
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


def test_web_viewer_does_not_change_display_environment(monkeypatch: Any) -> None:
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
