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

from collections.abc import Iterator
from pathlib import Path

import pytest
import pytest_mock

from dimos.experimental.imitation.collection.recorder import NativeCollectionRecorder
from dimos.experimental.memory.rust_recorder import RustMcapStoreConfig


@pytest.fixture
def recorder(tmp_path: Path) -> Iterator[NativeCollectionRecorder]:
    instance = NativeCollectionRecorder(store={"path": str(tmp_path / "collection.db")})
    yield instance
    instance.stop()


def test_native_collection_profile_resolves_dataprep_codecs(
    recorder: NativeCollectionRecorder,
    mocker: pytest_mock.MockerFixture,
) -> None:
    for name in ("color_image", "coordinator_joint_state", "status"):
        getattr(recorder, name).transport = mocker.MagicMock(channel=f"dimos/{name}")

    specs = recorder._stream_specs()

    assert [(spec.name, spec.codec) for spec in specs] == [
        ("color_image", "jpeg"),
        ("coordinator_joint_state", "lcm"),
        ("status", "lcm"),
    ]
    assert recorder.config.record_tf is False
    assert "record_tf" not in recorder.config.to_config_dict()


def test_native_collection_profile_rejects_non_sqlite_store(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="RustSqliteStoreConfig"):
        NativeCollectionRecorder(store=RustMcapStoreConfig(path=str(tmp_path / "collection.mcap")))
