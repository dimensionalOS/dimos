# Copyright 2025-2026 Dimensional Inc.
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

from dataclasses import dataclass
from typing import Any

import pytest

from dimos.visualization.rerun.bridge import RerunBridgeModule


@dataclass(frozen=True)
class _BoxFactory:
    def __call__(self, rerun: Any) -> Any:
        return rerun.Boxes3D(half_sizes=[1.0, 1.0, 1.0])


@dataclass
class _ResettableOverride:
    reset_count: int = 0

    def __call__(self, message: Any) -> Any:
        return message

    def reset(self) -> None:
        self.reset_count += 1


def test_replace_static_entities_rotates_recording_with_only_latest_scene(mocker) -> None:
    robot = _BoxFactory()
    old_scene = _BoxFactory()
    new_scene = _BoxFactory()
    bridge = RerunBridgeModule(
        static={
            "world/robot": robot,
            "world/scene/old-world": old_scene,
        }
    )
    restart = mocker.patch.object(bridge, "_restart_recording")

    try:
        result = bridge.replace_static_entities(
            "world/scene",
            {"world/scene/new-world": new_scene},
        )
    finally:
        bridge.stop()

    expected = {
        "world/robot": robot,
        "world/scene/new-world": new_scene,
    }
    restart.assert_called_once_with(expected)
    assert bridge._active_static_entities == expected
    assert result == {
        "root_path": "world/scene",
        "entity_count": 1,
        "recording_rotated": True,
    }


def test_replace_static_entities_rejects_entities_outside_subtree() -> None:
    bridge = RerunBridgeModule()

    try:
        with pytest.raises(ValueError, match="must be inside"):
            bridge.replace_static_entities(
                "world/scene",
                {"world/robot": _BoxFactory()},
            )
    finally:
        bridge.stop()


def test_recording_restart_releases_old_store_before_logging_new_static(mocker) -> None:
    events: list[str] = []
    override = _ResettableOverride()
    bridge = RerunBridgeModule(
        rerun_open="native",
        visual_override={"world/tf": override, "world/other_tf": override},
    )
    old_recording = mocker.Mock()
    old_recording.disconnect.side_effect = lambda: events.append("disconnect")
    bridge._recording = old_recording
    bridge._owns_recording_server = True

    stop_viewer = mocker.patch.object(
        bridge,
        "_stop_native_viewer",
        side_effect=lambda: events.append("stop-viewer"),
    )
    mocker.patch.object(bridge, "_wait_for_recording_port")
    mocker.patch.object(
        bridge,
        "_start_recording",
        side_effect=lambda: events.append("start-recording") or "rerun+http://localhost:9999/proxy",
    )
    mocker.patch.object(bridge, "_send_blueprint")
    mocker.patch.object(
        bridge,
        "_log_static_entities",
        side_effect=lambda _entities: events.append("log-static"),
    )
    mocker.patch.object(
        bridge,
        "_spawn_native_viewer",
        side_effect=lambda _uri: events.append("start-viewer") or True,
    )

    try:
        bridge._restart_recording({"world/scene/new-world": _BoxFactory()})
    finally:
        stop_viewer.side_effect = None
        bridge.stop()

    assert events == [
        "stop-viewer",
        "disconnect",
        "start-recording",
        "log-static",
        "start-viewer",
    ]
    assert override.reset_count == 1


def test_live_messages_are_dropped_instead_of_queued_during_replacement(mocker) -> None:
    bridge = RerunBridgeModule()
    bridge._min_intervals = {}
    convert = mocker.patch.object(bridge, "_visual_override_for_entity_path")
    restart = mocker.patch.object(bridge, "_restart_recording")

    def replace(_entities: dict[str, Any]) -> None:
        assert bridge._recording_paused is True
        bridge._on_message(object(), "dimos/tf")

    restart.side_effect = replace
    try:
        bridge.replace_static_entities(
            "world/scene",
            {"world/scene/new-world": _BoxFactory()},
        )
    finally:
        bridge.stop()

    convert.assert_not_called()
    assert bridge._active_recording_writes == 0
