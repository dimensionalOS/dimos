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
from types import SimpleNamespace
from typing import Any

import pytest

from dimos.core.global_config import global_config
from dimos.core.transport import LCMTransport, ZenohTransport
from dimos.experimental.memory import rust_cli_recorder
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Image import Image
from dimos.protocol.pubsub.impl.zenohpubsub import Topic as ZenohTopic


def _lcm(channel: str, payload_type: type[Any] = PoseStamped) -> LCMTransport[Any]:
    return LCMTransport(channel, payload_type)


def _zenoh(channel: str, payload_type: type[Any] = PoseStamped) -> ZenohTransport[Any]:
    return ZenohTransport(ZenohTopic(channel, payload_type))


@pytest.fixture(autouse=True)
def recording_config(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(global_config, "record", "sqlite")
    monkeypatch.setattr(global_config, "record_engine", "rust")
    monkeypatch.setattr(global_config, "record_topics", "*")
    monkeypatch.setattr(global_config, "record_encoding_threads", 4)
    monkeypatch.setattr(rust_cli_recorder, "recording_dir", lambda: tmp_path)


def test_plan_uses_actual_lcm_channels_and_memory_codecs(tmp_path: Path) -> None:
    odom = _lcm("/wire/odom")
    camera = _lcm("/wire/camera", Image)
    plan = rust_cli_recorder.make_plan(
        {
            ("odom", PoseStamped): odom,
            ("color_image", Image): camera,
        }
    )

    assert plan.backend == "lcm"
    assert plan.topics == {"stream_0": odom.channel, "stream_1": camera.channel}
    assert [(stream.name, stream.codec) for stream in plan.streams] == [
        ("odom", "lcm"),
        ("color_image", "jpeg"),
    ]
    assert plan.path == tmp_path / "memory.db"


def test_replay_does_not_build_the_native_recorder(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(global_config, "replay", True)
    monkeypatch.setattr(
        rust_cli_recorder.subprocess,
        "Popen",
        lambda *args, **kwargs: pytest.fail("replay must not build the recorder"),
    )

    rust_cli_recorder.prepare_rust_recorder()


def test_plan_uses_mcap_artifact_for_zenoh(tmp_path: Path) -> None:
    global_config.record = "mcap"

    plan = rust_cli_recorder.make_plan({("odom", PoseStamped): _zenoh("dimos/odom/PoseStamped")})

    assert plan.backend == "zenoh"
    assert plan.path == tmp_path / "memory.mcap"


def test_plan_rejects_unsupported_transport_before_creating_artifact(
    tmp_path: Path,
) -> None:
    with pytest.raises(ValueError, match=r"unsupported selections: odom \(SimpleNamespace\)"):
        rust_cli_recorder.make_plan({("odom", PoseStamped): SimpleNamespace(channel="/wire/odom")})

    assert list(tmp_path.iterdir()) == []


def test_plan_skips_non_lcm_payload_and_requires_one_recordable_stream() -> None:
    with pytest.raises(ValueError, match="selected no Rust-recordable streams"):
        rust_cli_recorder.make_plan({("values", dict): _lcm("/values")})


def test_plan_rejects_mixed_transport_sessions() -> None:
    with pytest.raises(ValueError, match="mixed LCM and Zenoh"):
        rust_cli_recorder.make_plan(
            {
                ("odom", PoseStamped): _lcm("/odom"),
                ("camera", Image): _zenoh("dimos/camera/Image"),
            }
        )


def test_sqlite_artifact_registration_matches_the_native_plan(tmp_path: Path) -> None:
    plan = rust_cli_recorder.make_plan(
        {("odom", PoseStamped): _lcm("/odom"), ("camera", Image): _lcm("/camera")}
    )

    rust_cli_recorder._prepare_artifact(plan)

    with SqliteStore(path=str(tmp_path / "memory.db"), must_exist=True) as store:
        assert store.list_streams() == ["camera", "odom"]
