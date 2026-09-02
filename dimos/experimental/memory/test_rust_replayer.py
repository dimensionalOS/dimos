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

from pathlib import Path
from typing import Any

import pytest

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core.stream import Out
from dimos.experimental.memory.rust_replayer import (
    RustReplayer,
    RustReplayerConfig,
    RustSqliteReplayStoreConfig,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Image import Image


class SampleRustReplayer(RustReplayer):
    color_image: Out[Image]
    odometry: Out[PoseStamped]


class FakeTransport:
    def __init__(self, channel: str) -> None:
        self.channel = channel

    def stop(self) -> None:
        pass


def connect(replayer: RustReplayer, **channels: str) -> None:
    for name, channel in channels.items():
        getattr(replayer, name).transport = FakeTransport(channel)  # type: ignore[assignment]


def test_specs_are_typed_and_explicitly_remapped(tmp_path: Path) -> None:
    replayer = SampleRustReplayer(
        store=RustSqliteReplayStoreConfig(path=str(tmp_path / "recording.db")),
        stream_remapping={"odometry": "go2_odom"},
    )
    connect(replayer, color_image="/camera", odometry="/odom")

    try:
        assert [spec.model_dump() for spec in replayer._stream_specs()] == [
            {
                "port": "color_image",
                "name": "color_image",
                "payload_type": "dimos.msgs.sensor_msgs.Image.Image",
            },
            {
                "port": "odometry",
                "name": "go2_odom",
                "payload_type": "dimos.msgs.geometry_msgs.PoseStamped.PoseStamped",
            },
        ]
    finally:
        replayer.stop()


def test_replay_window_rejects_ambiguous_start() -> None:
    with pytest.raises(ValueError, match="mutually exclusive"):
        RustReplayerConfig(seek=1.0, from_timestamp=2.0)


def test_mcap_replay_configuration_is_rejected() -> None:
    with pytest.raises(ValueError, match="Extra inputs are not permitted"):
        RustReplayerConfig.model_validate({"store": {"kind": "mcap", "path": "recording.mcap"}})


@pytest.mark.parametrize("speed", [0.0, -1.0, float("inf")])
def test_speed_must_be_positive_and_finite(speed: float) -> None:
    with pytest.raises(ValueError, match="speed"):
        RustReplayerConfig(speed=speed)


@pytest.mark.parametrize("field", ["seek", "duration", "from_timestamp"])
def test_window_values_must_be_finite(field: str) -> None:
    with pytest.raises(ValueError, match="finite"):
        RustReplayerConfig(**{field: float("inf")})


def test_start_resolves_and_sends_only_native_fields(tmp_path: Path) -> None:
    path = tmp_path / "recording.db"
    path.touch()
    replayer = SampleRustReplayer(store=RustSqliteReplayStoreConfig(path=str(path)))
    connect(replayer, odometry="/odom")
    try:
        replayer.config.streams = replayer._stream_specs()
        replayer.config.store.path = str(replayer._resolve_store_path())

        config: dict[str, Any] = replayer.config.to_config_dict()
        assert set(config) == {
            "store",
            "speed",
            "seek",
            "duration",
            "from_timestamp",
            "loop",
            "streams",
        }
        assert "stream_remapping" not in config
        assert config["store"] == {"path": str(path.resolve())}
        assert replayer._argv({"odometry": "/odom"}) == [replayer.config.executable]
    finally:
        replayer.stop()


def test_default_build_uses_the_live_cargo_workspace() -> None:
    config = RustReplayerConfig()

    assert config.cwd == str(DIMOS_PROJECT_ROOT)
    assert config.build_command == "cargo build --release -p dimos-memory-replayer"
    assert config.executable == str(
        DIMOS_PROJECT_ROOT / "target" / "release" / "dimos-memory-replayer"
    )
